"""
picoMODBUStest.py - MODBUS RTU Test Module
Receives and prints MODBUS messages for debugging/testing

This module:
- Receives MODBUS RTU messages from RS-485
- Validates CRC
- Prints message details (address, function code, data)
- Does NOT process commands or send responses
"""

from machine import UART, Pin
import time

# Constants
MODBUS_BAUD = 1843200  # 1.8432 Mbps per EOVSA-BE-DC-DOC-002-A

# Address pins (AD0-AD4) - same as picoDCMbone.py
AD0_PIN = 3   # GP3
AD1_PIN = 4   # GP4
AD2_PIN = 5   # GP5
AD3_PIN = 6   # GP6
AD4_PIN = 7   # GP7


def read_modbus_address():
    """
    Read MODBUS address from address pins (AD0-AD4)
    Formula: MB_address = ((Port_B & 0x1E) | ((Port_B & 0x20)>>5))
    """
    ad0 = Pin(AD0_PIN, Pin.IN, Pin.PULL_DOWN)
    ad1 = Pin(AD1_PIN, Pin.IN, Pin.PULL_DOWN)
    ad2 = Pin(AD2_PIN, Pin.IN, Pin.PULL_DOWN)
    ad3 = Pin(AD3_PIN, Pin.IN, Pin.PULL_DOWN)
    ad4 = Pin(AD4_PIN, Pin.IN, Pin.PULL_DOWN)
    
    port_b = 0
    if ad0.value():  # PB5 (AD0)
        port_b |= 0x20
    if ad1.value():  # PB1 (AD1)
        port_b |= 0x02
    if ad2.value():  # PB2 (AD2)
        port_b |= 0x04
    if ad3.value():  # PB3 (AD3)
        port_b |= 0x08
    if ad4.value():  # PB4 (AD4)
        port_b |= 0x10
    
    # Debug calculation
    part1 = port_b & 0x1E
    part2 = (port_b & 0x20) >> 5
    mb_address = part1 | part2
    
    return mb_address, ad0.value(), ad1.value(), ad2.value(), ad3.value(), ad4.value(), port_b, part1, part2


# ---------------- Modbus RTU helpers ----------------
# MODBUS CRC lookup tables (from Rabbit C low_level_modbus_slave.lib.c)
MODBUS_CRC_MSB = bytes([
    0x00,0xC1,0x81,0x40,	0x01,0xC0,0x80,0x41,	0x01,0xC0,0x80,0x41,	0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41,	0x00,0xC1,0x81,0x40,	0x00,0xC1,0x81,0x40,	0x01,0xC0,0x80,0x41,
    0x01,0xC0,0x80,0x41,	0x00,0xC1,0x81,0x40,	0x00,0xC1,0x81,0x40,	0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,	0x01,0xC0,0x80,0x41,	0x01,0xC0,0x80,0x41,	0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41,	0x00,0xC1,0x81,0x40,	0x00,0xC1,0x81,0x40,	0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,	0x01,0xC0,0x80,0x41,	0x01,0xC0,0x80,0x41,	0x00,0xC1,0x81,0x40,
    0x00,0xC1,0x81,0x40,	0x01,0xC0,0x80,0x41,	0x01,0xC0,0x80,0x41,	0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41,	0x00,0xC1,0x81,0x40, 0x00,0xC1,0x81,0x40,	0x01,0xC0,0x80,0x41,
    0x01,0xC0,0x80,0x41,	0x00,0xC1,0x81,0x40,	0x00,0xC1,0x81,0x40,	0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,	0x01,0xC0,0x80,0x41,	0x01,0xC0,0x80,0x41,	0x00,0xC1,0x81,0x40,
    0x00,0xC1,0x81,0x40,	0x01,0xC0,0x80,0x41,	0x01,0xC0,0x80,0x41,	0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41,	0x00,0xC1,0x81,0x40,	0x00,0xC1,0x81,0x40,	0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,	0x01,0xC0,0x80,0x41,	0x01,0xC0,0x80,0x41,	0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41,	0x00,0xC1,0x81,0x40,	0x00,0xC1,0x81,0x40,	0x01,0xC0,0x80,0x41,
    0x01,0xC0,0x80,0x41,	0x00,0xC1,0x81,0x40,	0x00,0xC1,0x81,0x40,	0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40,	0x01,0xC0,0x80,0x41,	0x01,0xC0,0x80,0x41,	0x00,0xC1,0x81,0x40
])

MODBUS_CRC_LSB = bytes([
    0x00,0xC0,0xC1,0x01, 0xC3,0x03,0x02,0xC2,	0xC6,0x06,0x07,0xC7,	0x05,0xC5,0xC4,0x04,
    0xCC,0x0C,0x0D,0xCD,	0x0F,0xCF,0xCE,0x0E, 0x0A,0xCA,0xCB,0x0B,	0xC9,0x09,0x08,0xC8,
    0xD8,0x18,0x19,0xD9, 0x1B,0xDB,0xDA,0x1A,	0x1E,0xDE,0xDF,0x1F, 0xDD,0x1D,0x1C,0xDC,
    0x14,0xD4,0xD5,0x15,	0xD7,0x17,0x16,0xD6, 0xD2,0x12,0x13,0xD3,	0x11,0xD1,0xD0,0x10,
    0xF0,0x30,0x31,0xF1,	0x33,0xF3,0xF2,0x32,	0x36,0xF6,0xF7,0x37,	0xF5,0x35,0x34,0xF4,
    0x3C,0xFC,0xFD,0x3D,	0xFF,0x3F,0x3E,0xFE,	0xFA,0x3A,0x3B,0xFB,	0x39,0xF9,0xF8,0x38,
    0x28,0xE8,0xE9,0x29,	0xEB,0x2B,0x2A,0xEA,	0xEE,0x2E,0x2F,0xEF,	0x2D,0xED,0xEC,0x2C,
    0xE4,0x24,0x25,0xE5,	0x27,0xE7,0xE6,0x26,	0x22,0xE2,0xE3,0x23,	0xE1,0x21,0x20,0xE0,
    0xA0,0x60,0x61,0xA1,	0x63,0xA3,0xA2,0x62,	0x66,0xA6,0xA7,0x67,	0xA5,0x65,0x64,0xA4,
    0x6C,0xAC,0xAD,0x6D,	0xAF,0x6F,0x6E,0xAE,	0xAA,0x6A,0x6B,0xAB,	0x69,0xA9,0xA8,0x68,
    0x78,0xB8,0xB9,0x79,	0xBB,0x7B,0x7A,0xBA,	0xBE,0x7E,0x7F,0xBF,	0x7D,0xBD,0xBC,0x7C,
    0xB4,0x74,0x75,0xB5,	0x77,0xB7,0xB6,0x76,	0x72,0xB2,0xB3,0x73,	0xB1,0x71,0x70,0xB0,
    0x50,0x90,0x91,0x51,	0x93,0x53,0x52,0x92,	0x96,0x56,0x57,0x97,	0x55,0x95,0x94,0x54,
    0x9C,0x5C,0x5D,0x9D,	0x5F,0x9F,0x9E,0x5E,	0x5A,0x9A,0x9B,0x5B,	0x99,0x59,0x58,0x98,
    0x88,0x48,0x49,0x89,	0x4B,0x8B,0x8A,0x4A,	0x4E,0x8E,0x8F,0x4F,	0x8D,0x4D,0x4C,0x8C,
    0x44,0x84,0x85,0x45,	0x87,0x47,0x46,0x86,	0x82,0x42,0x43,0x83,	0x41,0x81,0x80,0x40
])


def crc16_modbus(data: bytes) -> int:
    """
    Calculate MODBUS CRC16 using standard polynomial 0xA001.
    Returns CRC16 value. CRC is appended LSB first, then MSB on the wire.
    """
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
    """Check if frame has valid CRC"""
    if len(frame) < 4:
        return False
    rx_crc = frame[-2] | (frame[-1] << 8)  # LSB then MSB on the wire
    calc_crc = crc16_modbus(frame[:-2])
    return rx_crc == calc_crc


def crc16(data: bytes) -> int:
    """Alias for crc16_modbus for backward compatibility"""
    return crc16_modbus(data)


# MODBUS function code names
FUNCTION_NAMES = {
    0x01: "Read Coil Status",
    0x02: "Read Input Status",
    0x03: "Read Holding Registers",
    0x04: "Read Input Registers",
    0x05: "Write Single Coil",
    0x06: "Write Single Register",
    0x07: "Read Exception Status",
    0x08: "Diagnostics",
    0x0B: "Fetch Comm Event Counter",
    0x0C: "Fetch Comm Event Log",
    0x0F: "Write Multiple Coils",
    0x10: "Write Multiple Registers",
    0x11: "Report Slave ID",
    0x14: "Read General Reference",
    0x15: "Write General Reference",
    0x16: "Mask Write Register",
    0x17: "Read/Write Multiple Registers",
    0x18: "Read FIFO Queue",
    0x43: "Load Attenuator Set Value",
    0x44: "Load Attenuator Table",
    0x45: "Event Trigger",
}


def parse_modbus_message(data):
    """Parse and return MODBUS message details"""
    if len(data) < 2:
        return None
    
    addr = data[0]
    func = data[1]
    
    info = {
        'address': addr,
        'function': func,
        'function_name': FUNCTION_NAMES.get(func, f"Unknown (0x{func:02X})"),
        'raw_data': data,
        'data_bytes': data[2:] if len(data) > 2 else []
    }
    
    # Parse common parameters based on function code
    if len(data) >= 6:
        if func in [0x01, 0x02, 0x03, 0x04, 0x0F, 0x10, 0x17]:
            # These functions have starting address and quantity
            start_addr = (data[2] << 8) | data[3]
            quantity = (data[4] << 8) | data[5]
            info['start_address'] = start_addr
            info['quantity'] = quantity
        
        elif func in [0x05, 0x06]:
            # Write single coil/register
            addr = (data[2] << 8) | data[3]
            value = (data[4] << 8) | data[5]
            info['register_address'] = addr
            info['value'] = value
        
        elif func == 0x16:
            # Mask write register
            addr = (data[2] << 8) | data[3]
            and_mask = (data[4] << 8) | data[5]
            or_mask = (data[6] << 8) | data[7] if len(data) >= 8 else 0
            info['register_address'] = addr
            info['and_mask'] = and_mask
            info['or_mask'] = or_mask
    
    return info


def decode_modbus_message(frame: bytes) -> (int, str):
    """
    Decode MODBUS frame to formatted string (matching picoDCMdecoder.py format).
    Returns: (addr, formatted_string) where formatted_string is:
    addr=XX | func=0xXXXX | funcName=NAME | payload=...
    """
    if len(frame) < 4:
        return 0, f"INVALID (too short: {len(frame)} bytes)"
    
    addr = frame[0]
    func = frame[1]
    payload = frame[2:-2]
    
    # Exception response
    if func & 0x80 and len(payload) >= 1:
        exc = payload[0]
        return addr, f"addr={addr:2} | func=0x{func:04X} | funcName=EXCEPTION | payload=0x{exc:02X}"
    
    # 0x06 Write Single Register
    if func == 0x06 and len(payload) == 4:
        reg = (payload[0] << 8) | payload[1]
        val = (payload[2] << 8) | payload[3]
        
        if reg == 0x0000:
            atn1 = (val >> 8) & 0xFF
            atn2 = val & 0xFF
            return addr, f"addr={addr:2} | func=0x{func:04X} | funcName=WR1 | payload=reg=0x{reg:04X} ATN1={atn1} ATN2={atn2}"
        
        return addr, f"addr={addr:2} | func=0x{func:04X} | funcName=WR1 | payload=reg=0x{reg:04X} val=0x{val:04X}"
    
    # 0x03/0x04 Read regs request
    if func in (0x03, 0x04) and len(payload) == 4:
        start = (payload[0] << 8) | payload[1]
        qty = (payload[2] << 8) | payload[3]
        func_name = "RDH" if func == 0x03 else "RDI"
        return addr, f"addr={addr:2} | func=0x{func:04X} | funcName={func_name} | payload=start=0x{start:04X} qty={qty}"
    
    # 0x03/0x04 Read regs response: bytecount + data
    if func in (0x03, 0x04) and len(payload) >= 1:
        bc = payload[0]
        data = payload[1:]
        if bc == len(data) and (bc % 2 == 0):
            regs = []
            for k in range(0, bc, 2):
                regs.append((data[k] << 8) | data[k + 1])
            func_name = "RDH_RESP" if func == 0x03 else "RDI_RESP"
            regs_str = " ".join(f"0x{r:04X}" for r in regs)
            return addr, f"addr={addr:2} | func=0x{func:04X} | funcName={func_name} | payload={regs_str}"
        func_name = "RDH_RESP" if func == 0x03 else "RDI_RESP"
        hex_data = " ".join(f"{b:02X}" for b in data)
        return addr, f"addr={addr:2} | func=0x{func:04X} | funcName={func_name} | payload=bc={bc} data={hex_data}"
    
    # 0x10 Write Multiple Registers
    if func == 0x10:
        # response payload 4 bytes: start, qty
        if len(payload) == 4:
            start = (payload[0] << 8) | payload[1]
            qty = (payload[2] << 8) | payload[3]
            return addr, f"addr={addr:2} | func=0x{func:04X} | funcName=WRM_RESP | payload=start=0x{start:04X} qty={qty}"
        
        # request: start(2) qty(2) bc(1) data(bc)
        if len(payload) >= 5:
            start = (payload[0] << 8) | payload[1]
            qty = (payload[2] << 8) | payload[3]
            bc = payload[4]
            data = payload[5:]
            hex_data = " ".join(f"{b:02X}" for b in data)
            return addr, f"addr={addr:2} | func=0x{func:04X} | funcName=WRM | payload=start=0x{start:04X} qty={qty} bc={bc} data={hex_data}"
    
    # Default - use function name from lookup table if available
    func_name = FUNCTION_NAMES.get(func, "UNKNOWN")
    hex_payload = " ".join(f"{b:02X}" for b in payload)
    return addr, f"addr={addr:2} | func=0x{func:04X} | funcName={func_name} | payload={hex_payload}"


# Global buffer for accumulating received data
_rx_buf = bytearray()
_collecting = False
_last_rx_us = 0

# Frame detection parameters (matching picoDCMdecoder.py)
# At 1843200 baud: 3.5 character times ≈ 19 µs, use 120 µs to be safe
INTER_FRAME_US = 120
POLL_US = 40

def is_plausible_addr(a: int) -> bool:
    """Check if address is valid MODBUS address (0-247)"""
    return 0 <= a <= 247


def is_plausible_func(f: int) -> bool:
    """Check if function code is valid (0-127 or 0x80-0xFF for exceptions)"""
    return (0 <= f <= 0x7F) or (0x80 <= f <= 0xFF)


class ModbusFrameSplitter:
    """
    Split a byte chunk (may contain multiple RTU frames) into valid frames
    using heuristics and CRC validation (from picoDCMdecoder.py)
    """
    
    def __init__(self, max_frame=260):
        self.max_frame = max_frame
    
    def _candidate_lengths(self, buf: bytes, i: int):
        """Yield likely frame lengths from position i"""
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
        """Split chunk into frames. Returns (frames, leftover_bytes)"""
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
            
            # 2) Fallback CRC scan
            if found is None:
                max_end = min(n, i + self.max_frame)
                for j in range(i + 4, max_end + 1):
                    frame = chunk[i:j]
                    if check_crc(frame):
                        found = frame
                        break
            
            if found is None:
                # Not enough bytes yet or garbage; keep tail for next time
                break
            
            frames.append(found)
            i += len(found)
        
        leftover = chunk[i:] if i < n else b""
        return frames, leftover


# Global frame splitter instance
_frame_splitter = ModbusFrameSplitter()

def modbus_receive(uart, max_bytes=512):
    """
    Receive MODBUS message from UART using frame splitting approach.
    Returns (message_data, crc_valid, raw_bytes) or (None, None, None) if no message
    """
    global _rx_buf, _collecting, _last_rx_us
    
    now = time.ticks_us()
    
    # Read available bytes from UART
    n = uart.any()
    if n > 0:
        data = uart.read(min(n, max_bytes))
        if data:
            _rx_buf.extend(data)
            _collecting = True
            _last_rx_us = now
    
    if _collecting:
        idle = time.ticks_diff(now, _last_rx_us)
        if idle >= INTER_FRAME_US:
            # Frame group complete - split into individual frames
            chunk = bytes(_rx_buf)
            _rx_buf = bytearray()
            _collecting = False
            
            frames, leftover = _frame_splitter.split(chunk)
            
            # Keep leftover (partial frame) for next round
            if leftover:
                _rx_buf.extend(leftover)
                _collecting = True
                _last_rx_us = time.ticks_us()
            
            # Return first frame if available
            if frames:
                raw_msg = frames[0]
                payload = raw_msg[:-2]  # Remove CRC
                crc_valid = check_crc(raw_msg)
                return payload, crc_valid, raw_msg
    
    return None, None, None


def main():
    """Main test loop - receive and print MODBUS messages"""
    # Read and display MODBUS address from hardware pins
    mb_addr, _, _, _, _, _, _, _, _ = read_modbus_address()
    print(" MB address: ", mb_addr)
    
    # Set up MODBUS UART (8E1: 8 bits, EVEN parity, 1 stop bit)
    modbus_uart = UART(0, MODBUS_BAUD, tx=Pin(0), rx=Pin(1), bits=8, parity=0, stop=1)
    
    # Set up RS-485 DE/RE control pin (GP2)
    de_pin = Pin(2, Pin.OUT)
    de_pin.value(0)  # Start in receive mode
    
    message_count = 0
    valid_count = 0
    invalid_count = 0
    last_debug_time = time.ticks_ms()
    raw_byte_count = 0
    loop_count = 0
    
    try:
        while True:
            loop_count += 1
            
            # Receive message
            msg_data, crc_valid, raw_bytes = modbus_receive(modbus_uart)
            
            if msg_data is not None and raw_bytes is not None:
                message_count += 1
                raw_byte_count += len(raw_bytes)
                if crc_valid:
                    valid_count += 1
                    # Decode and print message
                    addr, decoded = decode_modbus_message(raw_bytes)
                    if addr == mb_addr:
                        print(decoded)
                else:
                    invalid_count += 1
                    # Print invalid message with CRC debug info (only for messages to our address)
                    addr, decoded = decode_modbus_message(raw_bytes)
                    if addr == mb_addr:
                        print(decoded + " [CRC_INVALID]")
                        if len(raw_bytes) >= 4:
                            payload = raw_bytes[:-2]
                            calc_crc = crc16_modbus(payload)
                            rx_crc = raw_bytes[-2] | (raw_bytes[-1] << 8)
                            print(f"  [CRC_DEBUG] calc=0x{calc_crc:04X} rx=0x{rx_crc:04X}")
            
            # Debug output every 5 seconds
            current_time = time.ticks_ms()
            elapsed = time.ticks_diff(current_time, last_debug_time)
            if elapsed > 5000:
                uart_has_data = modbus_uart.any()
                rate = (raw_byte_count * 1000) // max(1, elapsed) if elapsed > 0 else 0
                print(f"[DEBUG] Loop={loop_count}, UART.any()={uart_has_data}, Total raw bytes received: {raw_byte_count} ({rate} bytes/sec), Messages: {message_count}")
                if uart_has_data:
                    print(f"  [DEBUG] WARNING: UART has {uart_has_data} bytes waiting!")
                last_debug_time = current_time
            
            # Small delay to prevent tight loop (matching picoDCMdecoder.py poll_us)
            time.sleep_us(POLL_US)
    
    except KeyboardInterrupt:
        print()
        print("Test stopped by user")
        print(f"Final Statistics:")
        print(f"  Total messages:   {message_count}")
        print(f"  Valid CRC:        {valid_count}")
        print(f"  Invalid CRC:      {invalid_count}")


if __name__ == "__main__":
    main()
