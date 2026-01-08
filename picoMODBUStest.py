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
def crc16(data: bytes) -> int:
    """Calculate MODBUS CRC16 using standard polynomial 0xA001"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


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


# Global buffer for frame detection (like GP1test.py persistent buffer approach)
_frame_buf = bytearray()
_frame_last_rx = 0

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
        payload = raw_msg[:-2]
        rx_crc = (raw_msg[-2] << 8) | raw_msg[-1]  # MSB first, then LSB (Rabbit USE_MODBUS_CRC)
        calc_crc = crc16(payload)
        crc_valid = (calc_crc == rx_crc)
        
        messages.append((payload, crc_valid, raw_msg))
        idx += msg_len
    
    return messages


def modbus_receive(uart, quiet_us=100):  # ~100us quiet period (5x the 3.5 char time of ~20us for reliability)
    """
    Receive MODBUS message from UART using persistent buffer (exactly like GP1test.py)
    Uses microsecond timing for better frame detection
    Returns (message_data, crc_valid, raw_bytes) or (None, None, None) if no message
    """
    global _frame_buf, _frame_last_rx
    
    # Check for new data
    n = uart.any()
    if n:
        chunk = uart.read(n)
        if chunk:
            _frame_buf.extend(chunk)
            _frame_last_rx = time.ticks_us()
            if len(_frame_buf) > 256:
                _frame_buf[:] = b''  # Reset safely if too long
                return None, None, None
    
    # Check if we've had enough silence to complete a frame
    if _frame_buf and time.ticks_diff(time.ticks_us(), _frame_last_rx) > quiet_us:
        frame = bytes(_frame_buf)
        _frame_buf[:] = b''  # Reset buffer
        
        if len(frame) < 4:  # Minimum: addr + func + 2 CRC bytes
            return None, None, None
        
        # Try to extract individual MODBUS messages from the frame
        # (frames may contain multiple concatenated messages)
        messages = extract_messages_from_frame(frame)
        
        if messages:
            # Return the first message found (prioritize valid CRC if available)
            # Store remaining messages in a queue for next call
            for i, (payload, crc_valid, raw_msg) in enumerate(messages):
                if crc_valid:  # Prefer valid CRC messages
                    # Store remaining messages for next call
                    if i + 1 < len(messages):
                        remaining = b''.join([msg for _, _, msg in messages[i+1:]])
                        _frame_buf[:] = remaining
                        _frame_last_rx = time.ticks_us()  # Reset timer for remaining data
                    return payload, crc_valid, raw_msg
            # If no valid CRC, return first message anyway
            if len(messages) > 1:
                remaining = b''.join([msg for _, _, msg in messages[1:]])
                _frame_buf[:] = remaining
                _frame_last_rx = time.ticks_us()
            return messages[0]
        
        # Fallback: treat entire frame as single message
        rx_crc = (frame[-2] << 8) | frame[-1]  # MSB first, then LSB (Rabbit USE_MODBUS_CRC)
        payload = frame[:-2]
        calc_crc = crc16(payload)
        crc_valid = (calc_crc == rx_crc)
        return payload, crc_valid, frame
    
    return None, None, None


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
                else:
                    print(f"[Message #{message_count}] Failed to parse message")
                    if raw_bytes:
                        print(f"  Raw bytes: {raw_bytes.hex()}")
            
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
