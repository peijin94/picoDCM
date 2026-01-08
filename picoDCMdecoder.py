# dcm_modbus_sniffer.py
# MicroPython (Raspberry Pi Pico / RP2040)
#
# Sniffs MODBUS RTU traffic for the EOVSA Downconverter Module (DCM) and decodes
# common requests/responses (incl. DCM attenuator write reg 0x0000 high/low byte).
#
# Key fixes vs your crash / CRC issues:
#   1) No "del buf[i]" on bytearray (RP2040 MP build often doesn't support item deletion).
#      We use indices and rebuild buffers safely.
#   2) Use EVEN parity (per DCM doc: 8 data bits, even parity, 1 stop).
#   3) Frame boundary is tiny at 1.8432 Mbps; we detect RTU frames using idle-gap timing,
#      then split combined chunks by CRC scanning + heuristics.

from machine import UART, Pin
import time

# ---------------- CRC16 (MODBUS) ----------------

def crc16_modbus(data: bytes) -> int:
    """Return CRC16/MODBUS as int (0..65535). Append to frame LSB first."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

def check_crc(frame: bytes) -> bool:
    if len(frame) < 4:
        return False
    rx_lo = frame[-2]
    rx_hi = frame[-1]
    rx = rx_lo | (rx_hi << 8)
    calc = crc16_modbus(frame[:-2])
    return rx == calc

# ---------------- Pretty formatting ----------------

def hex_bytes(b: bytes, sep=" ") -> str:
    return sep.join("{:02X}".format(x) for x in b)

def u16_be(b0: int, b1: int) -> int:
    return (b0 << 8) | b1

def is_plausible_addr(a: int) -> bool:
    # MODBUS RTU: 0=broadcast, 1..247 normal
    return 0 <= a <= 247

def is_plausible_func(f: int) -> bool:
    # Public function codes 0..127, exception responses set MSB (>=0x80).
    return (0 <= f <= 0x7F) or (0x80 <= f <= 0xFF)

# ---------------- Frame splitter ----------------

class ModbusFrameSplitter:
    """
    Take incoming bytes (possibly containing multiple RTU frames stuck together)
    and split into valid frames using:
      - known-length heuristics for common function codes
      - fallback CRC scanning
    """

    def __init__(self, max_frame=260):
        self.max_frame = max_frame

    def _candidate_lengths(self, buf: bytes, i: int):
        """Yield likely frame lengths from position i."""
        if i + 2 > len(buf):
            return
        addr = buf[i]
        func = buf[i + 1]
        if not is_plausible_addr(addr) or not is_plausible_func(func):
            return

        # Common requests/responses:
        # 0x06 Write Single Register: request & response are 8 bytes
        if func == 0x06:
            yield 8

        # 0x03/0x04 Read regs:
        # - request: 8 bytes
        # - response: 5 + bytecount, where bytecount is buf[i+2]
        if func in (0x03, 0x04):
            yield 8
            if i + 3 <= len(buf):
                bc = buf[i + 2]
                # response length = addr(1)+func(1)+bc(1)+data(bc)+crc(2)
                yield 5 + bc

        # 0x10 Write Multiple Registers:
        # - request: 9 + bytecount, where bytecount is buf[i+6]
        # - response: 8 bytes
        if func == 0x10:
            yield 8
            if i + 7 <= len(buf):
                bc = buf[i + 6]
                yield 9 + bc

        # Exception response: addr, func|0x80, exception_code, CRC => 5 bytes
        if func & 0x80:
            yield 5

        # Always allow a few generic sizes (small frames)
        for L in (4, 5, 6, 7, 8, 9, 10, 11, 12):
            yield L

    def split(self, chunk: bytes):
        """
        Split chunk into frames.
        Returns (frames, leftover_bytes).
        """
        frames = []
        i = 0
        n = len(chunk)

        while i < n:
            # Skip until we find plausible addr/func
            if i + 2 > n:
                break
            addr = chunk[i]
            func = chunk[i + 1]
            if not (is_plausible_addr(addr) and is_plausible_func(func)):
                i += 1
                continue

            found = None

            # 1) Try heuristic lengths first (fast)
            tried = set()
            for L in self._candidate_lengths(chunk, i):
                if L in tried:
                    continue
                tried.add(L)
                if L < 4 or L > self.max_frame:
                    continue
                j = i + L
                if j <= n:
                    frame = chunk[i:j]
                    if check_crc(frame):
                        found = frame
                        break

            # 2) Fallback: CRC scan for an end within a reasonable window
            if found is None:
                max_end = min(n, i + self.max_frame)
                # minimum frame size is 4 bytes
                for j in range(i + 4, max_end + 1):
                    frame = chunk[i:j]
                    if check_crc(frame):
                        found = frame
                        break

            if found is None:
                # Not enough bytes yet or garbage; keep a tail for next time
                break

            frames.append(found)
            i += len(found)

        leftover = chunk[i:] if i < n else b""
        return frames, leftover

# ---------------- DCM command decoder ----------------

class DCMDecoder:
    """
    Decode common MODBUS RTU frames and print human-readable meaning.
    DCM-specific hint from doc excerpt: function 0x06 writing register 0x0000
    where data hi/lo are ATN1/ATN2 (0..15 dB) (as seen in extracted table text).
    """

    def decode(self, frame: bytes) -> str:
        addr = frame[0]
        func = frame[1]
        payload = frame[2:-2]
        crc_lo, crc_hi = frame[-2], frame[-1]
        crc = crc_lo | (crc_hi << 8)

        # Exception response
        if func & 0x80 and len(payload) >= 1:
            exc = payload[0]
            return f"addr={addr} func=0x{func:02X} EXCEPTION=0x{exc:02X} raw=[{hex_bytes(frame)}]"

        # Write Single Register (0x06): addr func reg_hi reg_lo val_hi val_lo crc_lo crc_hi
        if func == 0x06 and len(payload) == 4:
            reg = u16_be(payload[0], payload[1])
            val = u16_be(payload[2], payload[3])

            # DCM hint: reg 0x0000 packs ATN1 (hi byte) and ATN2 (lo byte)
            if reg == 0x0000:
                atn1 = (val >> 8) & 0xFF
                atn2 = val & 0xFF
                return (f"DCM WRITE_SINGLE_REG addr={addr} reg=0x{reg:04X} "
                        f"ATN1={atn1}dB ATN2={atn2}dB  raw=[{hex_bytes(frame)}]")
            return f"WRITE_SINGLE_REG addr={addr} reg=0x{reg:04X} val=0x{val:04X} ({val}) raw=[{hex_bytes(frame)}]"

        # Read Holding/Input Registers request: (0x03/0x04) payload = start_hi start_lo qty_hi qty_lo
        if func in (0x03, 0x04) and len(payload) == 4:
            start = u16_be(payload[0], payload[1])
            qty = u16_be(payload[2], payload[3])
            kind = "READ_HOLDING_REGS" if func == 0x03 else "READ_INPUT_REGS"
            return f"{kind} addr={addr} start=0x{start:04X} qty={qty} raw=[{hex_bytes(frame)}]"

        # Read Holding/Input Registers response: payload = bytecount + data
        if func in (0x03, 0x04) and len(payload) >= 1:
            bc = payload[0]
            data = payload[1:]
            if bc == len(data) and (bc % 2 == 0):
                regs = []
                for k in range(0, bc, 2):
                    regs.append(u16_be(data[k], data[k + 1]))
                kind = "READ_HOLDING_REGS_RESP" if func == 0x03 else "READ_INPUT_REGS_RESP"
                return f"{kind} addr={addr} regs={regs} raw=[{hex_bytes(frame)}]"
            # fallback
            return f"READ_REGS_RESP addr={addr} func=0x{func:02X} bc={bc} data=[{hex_bytes(data)}] raw=[{hex_bytes(frame)}]"

        # Write Multiple Registers (0x10)
        if func == 0x10:
            # response is 4 bytes payload: start_hi start_lo qty_hi qty_lo
            if len(payload) == 4:
                start = u16_be(payload[0], payload[1])
                qty = u16_be(payload[2], payload[3])
                return f"WRITE_MULT_REGS_RESP addr={addr} start=0x{start:04X} qty={qty} raw=[{hex_bytes(frame)}]"
            # request: start(2) qty(2) bytecount(1) data(bytecount)
            if len(payload) >= 5:
                start = u16_be(payload[0], payload[1])
                qty = u16_be(payload[2], payload[3])
                bc = payload[4]
                data = payload[5:]
                return f"WRITE_MULT_REGS_REQ addr={addr} start=0x{start:04X} qty={qty} bc={bc} data=[{hex_bytes(data)}] raw=[{hex_bytes(frame)}]"

        # Default
        return f"MODBUS addr={addr} func=0x{func:02X} payload=[{hex_bytes(payload)}] crc=0x{crc:04X} raw=[{hex_bytes(frame)}]"

# ---------------- UART sniffer loop ----------------

class ModbusSniffer:
    def __init__(
        self,
        uart_id=0,
        tx_pin=0,
        rx_pin=1,
        baud=1843200,
        inter_frame_us=120,      # try ~100-200us first at this baud
        poll_us=50,
        max_read=512
    ):
        # EVEN parity is critical per your DCM doc (8E1).
        # In MicroPython: parity=0 (even), 1 (odd), None (no parity)
        self.uart = UART(
            uart_id,
            baudrate=baud,
            bits=8,
            parity=0,     # EVEN
            stop=1,
            tx=Pin(tx_pin),
            rx=Pin(rx_pin),
            timeout=0,        # non-blocking
            timeout_char=0
        )

        self.inter_frame_us = inter_frame_us
        self.poll_us = poll_us
        self.max_read = max_read

        self._rx_buf = bytearray()
        self._collecting = False
        self._last_rx_us = time.ticks_us()

        self.splitter = ModbusFrameSplitter()
        self.decoder = DCMDecoder()

        print("DCM / MODBUS RTU sniffer")
        print("  baud:", baud)
        print("  UART:", uart_id, "TX pin:", tx_pin, "RX pin:", rx_pin)
        print("  framing: 8E1 (EVEN parity)")
        print("  CRC: MODBUS (append LSB then MSB)")
        print("  inter_frame_us:", inter_frame_us)
        print("Waiting for frames...")

    def _read_uart(self):
        n = self.uart.any()
        if not n:
            return None
        # read at most max_read bytes to keep latency down
        data = self.uart.read(min(n, self.max_read))
        return data

    def run_forever(self):
        while True:
            data = self._read_uart()
            now = time.ticks_us()

            if data:
                self._rx_buf.extend(data)
                self._collecting = True
                self._last_rx_us = now

            if self._collecting:
                idle = time.ticks_diff(now, self._last_rx_us)
                if idle >= self.inter_frame_us:
                    # treat current buffer as one "chunk" that may contain multiple frames
                    chunk = bytes(self._rx_buf)
                    self._rx_buf = bytearray()
                    self._collecting = False

                    frames, leftover = self.splitter.split(chunk)

                    # keep leftover (partial) for next round
                    if leftover:
                        self._rx_buf.extend(leftover)
                        self._collecting = True
                        self._last_rx_us = time.ticks_us()

                    for fr in frames:
                        print(self.decoder.decode(fr))

            # tiny poll sleep; do NOT sleep ms(10) at this baud
            time.sleep_us(self.poll_us)

# ---------------- Main ----------------

# Adjust pins to your wiring.
# Pico UART0 defaults: TX=GP0, RX=GP1
# Pico UART1 defaults: TX=GP4, RX=GP5 (but you can remap)
sniffer = ModbusSniffer(
    uart_id=0,
    tx_pin=0,
    rx_pin=1,
    baud=1843200,
    inter_frame_us=120,  # if you still see merged frames, try 80..150; if broken, try 200..400
    poll_us=40,
    max_read=512
)

sniffer.run_forever()