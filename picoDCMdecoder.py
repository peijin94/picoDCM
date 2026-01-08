# dcm_modbus_sniffer_min.py
# MicroPython (RP2040 / Raspberry Pi Pico)
#
# MODBUS RTU sniffer + DCM command decoder (minimal output).
# - No debug logging; prints only decoded frames.
# - Uses idle-gap detection to cut chunks, then CRC-based splitting.
# - UART framing: 8E1 (EVEN parity) as per your DCM doc excerpt.

from machine import UART, Pin
import time


# ---------------- CRC16 (MODBUS RTU) ----------------

def crc16_modbus(data: bytes) -> int:
    """CRC16/MODBUS polynomial 0xA001, init 0xFFFF. Appended LSB then MSB."""
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
    rx = frame[-2] | (frame[-1] << 8)   # LSB then MSB on the wire
    return rx == crc16_modbus(frame[:-2])


# ---------------- Helpers ----------------

def hex_bytes(b: bytes, sep=" ") -> str:
    return sep.join("{:02X}".format(x) for x in b)


def u16_be(b0: int, b1: int) -> int:
    return (b0 << 8) | b1


def is_plausible_addr(a: int) -> bool:
    # MODBUS RTU: 0=broadcast, 1..247 unicast
    return 0 <= a <= 247


def is_plausible_func(f: int) -> bool:
    # Function code 0..127; exception responses set MSB (0x80..0xFF)
    return (0 <= f <= 0x7F) or (0x80 <= f <= 0xFF)


# ---------------- Frame splitter ----------------

class ModbusFrameSplitter:
    """
    Split a byte chunk (may contain multiple RTU frames concatenated) into
    valid frames using:
      1) known-length heuristics for common function codes
      2) fallback CRC scan
    """

    def __init__(self, max_frame=260):
        self.max_frame = max_frame

    def _candidate_lengths(self, buf: bytes, i: int):
        if i + 2 > len(buf):
            return
        addr = buf[i]
        func = buf[i + 1]
        if not is_plausible_addr(addr) or not is_plausible_func(func):
            return

        # 0x06 Write Single Register: request/response fixed 8 bytes
        if func == 0x06:
            yield 8

        # 0x03/0x04 Read regs:
        # request fixed 8 bytes
        # response length = 5 + bytecount (bytecount at buf[i+2])
        if func in (0x03, 0x04):
            yield 8
            if i + 3 <= len(buf):
                bc = buf[i + 2]
                yield 5 + bc

        # 0x10 Write Multiple Registers:
        # response fixed 8 bytes
        # request length = 9 + bytecount (bytecount at buf[i+6])
        if func == 0x10:
            yield 8
            if i + 7 <= len(buf):
                bc = buf[i + 6]
                yield 9 + bc

        # Exception response: addr, func|0x80, exception_code, CRC => 5 bytes
        if func & 0x80:
            yield 5

        # Small generic lengths as fallback
        for L in (4, 5, 6, 7, 8, 9, 10, 11, 12):
            yield L

    def split(self, chunk: bytes):
        frames = []
        i = 0
        n = len(chunk)

        while i < n:
            if i + 2 > n:
                break

            addr = chunk[i]
            func = chunk[i + 1]

            if not (is_plausible_addr(addr) and is_plausible_func(func)):
                i += 1
                continue

            found = None

            # 1) Try heuristic lengths first
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

            # 2) Fallback CRC scan
            if found is None:
                max_end = min(n, i + self.max_frame)
                for j in range(i + 4, max_end + 1):
                    frame = chunk[i:j]
                    if check_crc(frame):
                        found = frame
                        break

            if found is None:
                # keep tail as leftover (likely partial frame)
                break

            frames.append(found)
            i += len(found)

        leftover = chunk[i:] if i < n else b""
        return frames, leftover


# ---------------- DCM / MODBUS decoder ----------------

class DCMDecoder:
    """
    Minimal human-readable decode.
    DCM-specific: reg 0x0000 on function 0x06 interpreted as ATN1 (hi byte), ATN2 (lo byte).
    """

    def decode(self, frame: bytes) -> str:
        addr = frame[0]
        func = frame[1]
        payload = frame[2:-2]

        # Exception response
        if func & 0x80 and len(payload) >= 1:
            exc = payload[0]
            return f"addr={addr:2} | func=0x{func:04X} | funcName=EXCEPTION | payload=0x{exc:02X}"

        # 0x06 Write Single Register
        if func == 0x06 and len(payload) == 4:
            reg = u16_be(payload[0], payload[1])
            val = u16_be(payload[2], payload[3])

            if reg == 0x0000:
                atn1 = (val >> 8) & 0xFF
                atn2 = val & 0xFF
                return f"addr={addr:2} | func=0x{func:04X} | funcName=WR1 | payload=reg=0x{reg:04X} ATN1={atn1}dB ATN2={atn2}dB"

            return f"addr={addr:2} | func=0x{func:04X} | funcName=WR1 | payload=reg=0x{reg:04X} val=0x{val:04X}"

        # 0x03/0x04 Read regs request
        if func in (0x03, 0x04) and len(payload) == 4:
            start = u16_be(payload[0], payload[1])
            qty = u16_be(payload[2], payload[3])
            func_name = "RDH" if func == 0x03 else "RDI"
            return f"addr={addr:2} | func=0x{func:04X} | funcName={func_name} | payload=start=0x{start:04X} qty={qty}"

        # 0x03/0x04 Read regs response: bytecount + data
        if func in (0x03, 0x04) and len(payload) >= 1:
            bc = payload[0]
            data = payload[1:]
            if bc == len(data) and (bc % 2 == 0):
                regs = []
                for k in range(0, bc, 2):
                    regs.append(u16_be(data[k], data[k + 1]))
                func_name = "RDH_RESP" if func == 0x03 else "RDI_RESP"
                regs_str = " ".join(f"0x{r:04X}" for r in regs)
                return f"addr={addr:2} | func=0x{func:04X} | funcName={func_name} | payload={regs_str}"
            func_name = "RDH_RESP" if func == 0x03 else "RDI_RESP"
            return f"addr={addr:2} | func=0x{func:04X} | funcName={func_name} | payload=bc={bc} data={hex_bytes(data)}"

        # 0x10 Write Multiple Registers
        if func == 0x10:
            # response payload 4 bytes: start, qty
            if len(payload) == 4:
                start = u16_be(payload[0], payload[1])
                qty = u16_be(payload[2], payload[3])
                return f"addr={addr:2} | func=0x{func:04X} | funcName=WRM_RESP | payload=start=0x{start:04X} qty={qty}"

            # request: start(2) qty(2) bc(1) data(bc)
            if len(payload) >= 5:
                start = u16_be(payload[0], payload[1])
                qty = u16_be(payload[2], payload[3])
                bc = payload[4]
                data = payload[5:]
                return f"addr={addr:2} | func=0x{func:04X} | funcName=WRM | payload=start=0x{start:04X} qty={qty} bc={bc} data={hex_bytes(data)}"

        # Default
        return f"addr={addr:2} | func=0x{func:04X} | funcName=UNKNOWN | payload={hex_bytes(payload)}"


# ---------------- UART sniffer ----------------

class ModbusSniffer:
    def __init__(
        self,
        uart_id=0,
        tx_pin=0,
        rx_pin=1,
        baud=1843200,
        inter_frame_us=120,  # tune 80..400 depending on wiring / buffering
        poll_us=40,
        max_read=512
    ):
        # parity=0 => EVEN
        self.uart = UART(
            uart_id,
            baudrate=baud,
            bits=8,
            parity=0,
            stop=1,
            tx=Pin(tx_pin),
            rx=Pin(rx_pin),
            timeout=0,
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

    def _read_uart(self):
        n = self.uart.any()
        if not n:
            return None
        return self.uart.read(min(n, self.max_read))

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
                    chunk = bytes(self._rx_buf)
                    self._rx_buf = bytearray()
                    self._collecting = False

                    frames, leftover = self.splitter.split(chunk)

                    if leftover:
                        self._rx_buf.extend(leftover)
                        self._collecting = True
                        self._last_rx_us = time.ticks_us()

                    for fr in frames:
                        print(self.decoder.decode(fr))

            time.sleep_us(self.poll_us)


# ---------------- Main ----------------

sniffer = ModbusSniffer(
    uart_id=0,
    tx_pin=0,
    rx_pin=1,
    baud=1843200,
    inter_frame_us=120,
    poll_us=40,
    max_read=512
)

sniffer.run_forever()