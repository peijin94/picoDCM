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


def crc16(data: bytes) -> int:
    """
    Calculate MODBUS CRC16 using lookup tables (matches Rabbit C implementation).
    From low_level_modbus_slave.lib.c MODBUS_CRC()
    """
    c_hi = 0xFF  # CRC MSB accumulator
    c_lo = 0xFF  # CRC LSB accumulator
    
    for byte in data:
        w = c_hi ^ byte  # Next table index
        c_hi = c_lo ^ MODBUS_CRC_MSB[w]  # Next CRC MSB
        c_lo = MODBUS_CRC_LSB[w]  # Next CRC LSB
    
    return (c_hi << 8) | c_lo


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


def print_modbus_message(msg_info, crc_valid=True, raw_bytes=None):
    """Print formatted MODBUS message information"""
    print("-" * 60)
    print(f"MODBUS Message (CRC: {'VALID' if crc_valid else 'INVALID'}):")
    print(f"  Address:      {msg_info['address']} (0x{msg_info['address']:02X})")
    print(f"  Function:     0x{msg_info['function']:02X} - {msg_info['function_name']}")
    
    # Print specific parameters
    if 'start_address' in msg_info:
        print(f"  Start Addr:   {msg_info['start_address']} (0x{msg_info['start_address']:04X})")
        print(f"  Quantity:     {msg_info['quantity']} (0x{msg_info['quantity']:04X})")
    
    if 'register_address' in msg_info:
        print(f"  Reg Address:  {msg_info['register_address']} (0x{msg_info['register_address']:04X})")
    
    if 'value' in msg_info:
        print(f"  Value:        {msg_info['value']} (0x{msg_info['value']:04X})")
    
    if 'and_mask' in msg_info:
        print(f"  AND Mask:     0x{msg_info['and_mask']:04X}")
        print(f"  OR Mask:      0x{msg_info['or_mask']:04X}")
    
    # Print raw bytes (including CRC if available)
    if raw_bytes:
        print(f"  Raw Bytes:    ", end="")
        for i, b in enumerate(raw_bytes):
            if i > 0 and i % 16 == 0:
                print(f"\n                 ", end="")
            print(f"{b:02X} ", end="")
        print()
    
    # Print raw bytes from msg_info (without CRC)
    raw_data = msg_info['raw_data']
    print(f"  Data Length:  {len(raw_data)} bytes")
    print(f"  Data Bytes:   ", end="")
    for i, b in enumerate(raw_data):
        if i > 0 and i % 16 == 0:
            print(f"\n                 ", end="")
        print(f"{b:02X} ", end="")
    print()
    
    print("-" * 60)
    print()


# Global buffer for accumulating received data (matching Rabbit C serRead approach)
# From MODBUS_SLAVE_DCM.lib.c: ByteCount = serRead( DataAddress, 200 );
# serRead uses Serial_timeout (5 byte times or 2msec) to detect frame end
# The serial driver waits for timeout after last byte before returning
_rx_buf = bytearray()
_rx_last_byte_time = 0

# Frame timeout: MODBUS RTU end-of-frame detection
# At 1843200 baud: 1 character time (start+8+stop ≈ 10 bits) = 10/1843200 ≈ 5.4 µs
# MODBUS RTU end-of-frame = 3.5 character times ≈ 19 µs
# Use 50 µs to be safe (allows for some timing variation)
FRAME_TIMEOUT_US = 50
_rx_last_byte_time = 0

# Calculate timeout: 5 byte times or 2ms, whichever is greater
# At 1843200 baud: 1 byte time = 11 bits / 1843200 = ~6us, so 5 byte times = ~30us
# So use 2ms (2000us) as the timeout
FRAME_TIMEOUT_US = 2000  # 2ms timeout matching Serial_timeout logic

def extract_messages_from_frame(frame):
    """
    Extract individual MODBUS messages from a potentially concatenated frame
    Returns list of (payload, crc_valid, raw_msg) tuples
    """
    messages = []
    idx = 0
    
    while idx < len(frame) - 3:  # Need at least 4 bytes (addr + func + 2 CRC)
        # Try to find a valid MODBUS message starting at idx
        # We need to guess the message length based on function code
        if len(frame) - idx < 4:
            break
        
        addr = frame[idx]
        func = frame[idx + 1]
        
        # Determine expected message length based on function code
        msg_len = None
        if func in [0x01, 0x02]:  # Read coils/inputs
            if len(frame) - idx >= 8:  # addr + func + start(2) + count(2) + CRC(2)
                count = (frame[idx + 4] << 8) | frame[idx + 5]
                byte_count = (count + 7) // 8
                msg_len = 6 + byte_count + 2  # header(6) + data + CRC(2)
        elif func in [0x03, 0x04]:  # Read registers
            if len(frame) - idx >= 8:
                count = (frame[idx + 4] << 8) | frame[idx + 5]
                msg_len = 6 + count * 2 + 2  # header(6) + data(count*2) + CRC(2)
        elif func in [0x05, 0x06]:  # Write single
            msg_len = 8  # addr + func + addr(2) + value(2) + CRC(2)
        elif func in [0x0F, 0x10]:  # Write multiple
            if len(frame) - idx >= 7:
                byte_count = frame[idx + 6]
                msg_len = 7 + byte_count + 2  # header(7) + data + CRC(2)
        elif func == 0x17:  # Read/write multiple
            if len(frame) - idx >= 11:
                read_count = (frame[idx + 4] << 8) | frame[idx + 5]
                write_byte_count = frame[idx + 10]
                msg_len = 11 + write_byte_count + 2  # header(11) + write data + CRC(2)
        elif func == 0x16:  # Mask write
            msg_len = 10  # addr + func + addr(2) + and(2) + or(2) + CRC(2)
        elif func == 0x43:  # Load attenuator (same as 0x06)
            msg_len = 8  # addr + func + addr(2) + value(2) + CRC(2)
        elif func == 0x44:  # Load attenuator table (same as 0x10)
            if len(frame) - idx >= 7:
                byte_count = frame[idx + 6]
                msg_len = 7 + byte_count + 2  # header(7) + data + CRC(2)
        elif func == 0xC0:  # Unknown function, might be broadcast or custom
            # Try to determine length - might be variable
            # For now, try minimum length and validate CRC
            if len(frame) - idx >= 8:
                # Try to find pattern: might be followed by another message
                # Look for next valid address byte pattern
                msg_len = 8  # Guess: addr + func + data(4) + CRC(2)
            else:
                msg_len = None
        else:
            # Unknown function, try minimum length
            msg_len = 4  # addr + func + CRC(2)
        
        if msg_len is None or idx + msg_len > len(frame):
            # Can't determine message length, skip this byte and try again
            idx += 1
            continue
        
        # Extract the message
        raw_msg = frame[idx:idx + msg_len]
        if len(raw_msg) < 4:
            idx += 1
            continue
        
        # Validate CRC
        # When USE_MODBUS_CRC is defined, CRC is at frame[ByteCount] (MSB) and frame[ByteCount+1] (LSB)
        # From MODBUS_SLAVE_DCM.lib.c lines 634-638:
        #   CalcCRC = MODBUS_CRC(DataAddress, ByteCount);  // Calculate on payload
        #   RxCRC = DataAddress[ByteCount+1] & 0x00FF;     // LSB at ByteCount+1
        #   i = DataAddress[ByteCount]<<8;                 // MSB at ByteCount
        #   RxCRC = RxCRC | ( i & 0xFF00 );                // Merge: (MSB << 8) | LSB
        payload = raw_msg[:-2]
        rx_crc_msb = raw_msg[-2]  # MSB from frame[ByteCount]
        rx_crc_lsb = raw_msg[-1]  # LSB from frame[ByteCount+1]
        rx_crc = (rx_crc_msb << 8) | rx_crc_lsb
        calc_crc = crc16(payload)
        crc_valid = (calc_crc == rx_crc)
        
        
        messages.append((payload, crc_valid, raw_msg))
        idx += msg_len
    
    return messages


def modbus_receive(uart, max_bytes=200):
    """
    Receive MODBUS message from UART (matches Rabbit C MODBUS_Serial_Rx logic).
    From MODBUS_SLAVE_DCM.lib.c line 619: ByteCount = serRead( DataAddress, 200 );
    
    Simply reads available bytes from UART buffer (up to max_bytes), then validates CRC.
    Returns (message_data, crc_valid, raw_bytes) or (None, None, None) if no message
    """
    global _rx_buf, _rx_last_byte_time
    
    current_time = time.ticks_us()
    
    # Read available bytes from UART (matching serRead behavior)
    # serRead waits for timeout after last byte before returning (frame detection)
    n = uart.any()
    if n > 0:
        chunk = uart.read(min(n, max_bytes))
        if chunk:
            _rx_buf.extend(chunk)
            _rx_last_byte_time = current_time  # Update timestamp when we receive bytes
    
    # Check if we have a complete frame (timeout elapsed since last byte)
    # This matches serRead's frame detection: wait for timeout before processing
    if len(_rx_buf) >= 4:  # Minimum: addr + func + 2 CRC bytes
        # Check if frame is complete (no bytes received for timeout period)
        frame_complete = False
        if _rx_last_byte_time > 0:
            elapsed = time.ticks_diff(current_time, _rx_last_byte_time)
            if elapsed >= FRAME_TIMEOUT_US:
                frame_complete = True  # Timeout elapsed, can process buffer
        else:
            # No timestamp yet, check buffer size as heuristic
            if len(_rx_buf) >= max_bytes:
                frame_complete = True  # Buffer full, process what we have
        
        if not frame_complete:
            # Still receiving data, wait for timeout (frame not complete yet)
            return None, None, None
        
        # Frame group is complete (timeout elapsed or buffer full)
        # Scan buffer to find FIRST valid message by checking CRC at different lengths
        # Messages may be concatenated, so we need to find boundaries
        frame = bytes(_rx_buf)
        
        # Try to find first valid message starting from shortest possible length (4 bytes)
        # Scan from 4 bytes up to the buffer size
        for msg_len in range(4, len(frame) + 1):
            if msg_len > len(frame):
                break
            
            byte_count = msg_len - 2  # Adjust for CRC
            if byte_count < 2:  # Need at least addr + func
                continue
            
            # Validate CRC exactly like C code (lines 634-638 with USE_MODBUS_CRC)
            # Note: MODBUS spec says CRC is transmitted LSB first, then MSB.
            # However, with USE_MODBUS_CRC defined, the C code reads them in reverse:
            # - DataAddress[ByteCount] = MSB (line 636)
            # - DataAddress[ByteCount+1] = LSB (line 635)
            # This matches the MODBUS_CRC() function's output byte order.
            payload = frame[:byte_count]
            calc_crc = crc16(payload)  # Calculate CRC on payload (line 634)
            rx_crc_lsb = frame[byte_count + 1] & 0x00FF  # LSB at DataAddress[ByteCount+1] (line 635)
            rx_crc_msb_word = (frame[byte_count] << 8) & 0xFF00  # MSB at DataAddress[ByteCount] (line 636)
            rx_crc = rx_crc_lsb | rx_crc_msb_word  # Merge bytes (line 638)
            
            # Check CRC (line 649)
            if calc_crc == rx_crc:
                # Valid message found! Extract it and remove from buffer
                raw_msg = frame[:msg_len]
                _rx_buf = _rx_buf[msg_len:]  # Remove processed message
                if len(_rx_buf) == 0:
                    _rx_last_byte_time = 0  # Reset timestamp when buffer empty
                return payload, True, raw_msg
        
        # No valid CRC found in entire buffer - this shouldn't happen often
        # Return first minimum-length message for debugging (may be partial/invalid)
        if len(_rx_buf) >= 4:
            payload = frame[:2]
            raw_msg = frame[:4]
            _rx_buf = _rx_buf[4:]  # Remove 4 bytes to prevent buffer overflow
            if len(_rx_buf) == 0:
                _rx_last_byte_time = 0
            return payload, False, raw_msg
        
        # Buffer empty or corrupted
        _rx_buf = bytearray()
        _rx_last_byte_time = 0
        return None, None, None
    
    return None, None, None
    
    return None, None, None


def test_crc():
    """Test CRC calculation with known values"""
    print("=" * 60)
    print("CRC Calculation Test")
    print("=" * 60)
    
    # Test with a simple message: Address 1, Function 3, Register 0x0000, Count 1
    # Known good MODBUS message: 01 03 00 00 00 01 <CRC>
    test_payload = bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x01])
    calc_crc = crc16(test_payload)
    print(f"Test payload: {test_payload.hex()}")
    print(f"Calculated CRC: 0x{calc_crc:04X} ({calc_crc:04d})")
    print(f"  MSB: 0x{(calc_crc >> 8) & 0xFF:02X}, LSB: 0x{calc_crc & 0xFF:02X}")
    
    # Standard MODBUS CRC for this message should be 0x84 0x0A (LSB first) = 0x0A84
    # But with USE_MODBUS_CRC (MSB first), it should be different
    print(f"\nNote: With USE_MODBUS_CRC (MSB first), CRC bytes are: 0x{(calc_crc >> 8) & 0xFF:02X} 0x{calc_crc & 0xFF:02X}")
    print()


def loopback_test():
    """Test UART by sending and receiving data (connect TX to RX for loopback)"""
    print("=" * 60)
    print("UART Loopback Test")
    print("=" * 60)
    print("Connect GP0 (TX) to GP1 (RX) directly for this test")
    print()
    
    uart = UART(0, MODBUS_BAUD, tx=Pin(0), rx=Pin(1), bits=8, parity=None, stop=1)
    
    test_data = b'\x0E\x04\x00\x00\x00\x01'  # Address 14, read input register
    crc = crc16(test_data)
    test_msg = test_data + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
    
    print(f"Sending: {test_msg.hex()}")
    uart.write(test_msg)
    
    time.sleep_ms(100)
    
    if uart.any():
        received = uart.read(256)
        print(f"Received: {received.hex()}")
        if received == test_msg:
            print("LOOPBACK TEST PASSED!")
        else:
            print("LOOPBACK TEST FAILED - data mismatch")
    else:
        print("LOOPBACK TEST FAILED - no data received")
        print("Check: Is GP0 connected to GP1?")
    print()


def main():
    """Main test loop - receive and print MODBUS messages"""
    print("=" * 60)
    print("MODBUS RTU Test Module")
    print("=" * 60)
    print(f"Baud Rate: {MODBUS_BAUD}")
    print("GP0: TX_E, GP1: RX_E, GP2: DRV_EN (RS-485 DE/RE)")
    print()
    
    # Read and display MODBUS address from hardware pins
    mb_addr, ad0, ad1, ad2, ad3, ad4, port_b, part1, part2 = read_modbus_address()
    print(f"MODBUS Address from pins: {mb_addr}")
    print(f"  AD0 (GP3): {ad0}")
    print(f"  AD1 (GP4): {ad1}")
    print(f"  AD2 (GP5): {ad2}")
    print(f"  AD3 (GP6): {ad3}")
    print(f"  AD4 (GP7): {ad4}")
    print(f"  Port_B: 0x{port_b:02X} (0b{port_b:08b})")
    print(f"  Calculation: (0x{port_b:02X} & 0x1E) | ((0x{port_b:02X} & 0x20) >> 5)")
    print(f"              = 0x{part1:02X} | 0x{part2:02X} = {mb_addr}")
    print()
    
    # Test CRC calculation
    test_crc()
    
    print("Waiting for MODBUS messages (monitoring all addresses)...")
    print("Press Ctrl+C to stop")
    print()
    
    # Set up MODBUS UART
    modbus_uart = UART(0, MODBUS_BAUD, tx=Pin(0), rx=Pin(1), bits=8, parity=None, stop=1)
    
    # Set up RS-485 DE/RE control pin (GP2)
    de_pin = Pin(2, Pin.OUT)
    de_pin.value(0)  # Start in receive mode
    
    message_count = 0
    valid_count = 0
    invalid_count = 0
    last_debug_time = time.ticks_ms()
    raw_byte_count = 0
    loop_count = 0
    
    print("[DEBUG] Starting receive loop...")
    
    try:
        while True:
            loop_count += 1
            
            # Receive message
            msg_data, crc_valid, raw_bytes = modbus_receive(modbus_uart)
            
            if msg_data is not None:
                message_count += 1
                if raw_bytes:
                    raw_byte_count += len(raw_bytes)
                if crc_valid:
                    valid_count += 1
                else:
                    invalid_count += 1
                
                # Parse message
                msg_info = parse_modbus_message(msg_data)
                
                if msg_info:
                    print(f"[Message #{message_count}]", end=" ")
                    print_modbus_message(msg_info, crc_valid, raw_bytes)
                    # Debug CRC for invalid messages
                    if not crc_valid and raw_bytes and len(raw_bytes) >= 4:
                        payload = raw_bytes[:-2]
                        calc_crc = crc16(payload)
                        rx_crc_msb = raw_bytes[-2]
                        rx_crc_lsb = raw_bytes[-1]
                        rx_crc = (rx_crc_msb << 8) | rx_crc_lsb
                        print(f"  [CRC DEBUG] Payload: {payload.hex()}")
                        print(f"  [CRC DEBUG] CalcCRC=0x{calc_crc:04X}, RxCRC=0x{rx_crc:04X} (MSB=0x{rx_crc_msb:02X}, LSB=0x{rx_crc_lsb:02X})")
                else:
                    print(f"[Message #{message_count}] Failed to parse message")
                    if raw_bytes:
                        print(f"  Raw bytes: {raw_bytes.hex()}")
                        # Try to validate CRC anyway
                        if len(raw_bytes) >= 4:
                            payload = raw_bytes[:-2]
                            calc_crc = crc16(payload)
                            rx_crc_msb = raw_bytes[-2]
                            rx_crc_lsb = raw_bytes[-1]
                            rx_crc = (rx_crc_msb << 8) | rx_crc_lsb
                            print(f"  [CRC DEBUG] Payload: {payload.hex()}")
                            print(f"  [CRC DEBUG] CalcCRC=0x{calc_crc:04X}, RxCRC=0x{rx_crc:04X} (MSB=0x{rx_crc_msb:02X}, LSB=0x{rx_crc_lsb:02X})")
            
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
            
            # Small delay to prevent tight loop
            time.sleep_ms(10)
    
    except KeyboardInterrupt:
        print()
        print("Test stopped by user")
        print(f"Final Statistics:")
        print(f"  Total messages:   {message_count}")
        print(f"  Valid CRC:        {valid_count}")
        print(f"  Invalid CRC:      {invalid_count}")


if __name__ == "__main__":
    main()
