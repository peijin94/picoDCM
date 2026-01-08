"""
DCM Module for Raspberry Pi Pico (MicroPython)
Migrated from Rabbit Single Board Computer C code

This module implements a MODBUS RTU slave for the EOVSA Downconverter Module (DCM)
with support for:
- MODBUS RTU over RS-485
- Digital I/O
- Analog inputs (ADC)
- Attenuator control (normal and table modes)
- Serial communication with optical receivers
"""

from machine import UART, Pin, ADC
import struct
import time

# Constants
BUFFER_LEN = 132
FIRMWARE_REV = 6
MODBUS_BAUD = 1843200

# MODBUS Error codes
MB_SUCCESS = 0
MB_BADFUNC = 0x01
MB_BADADDR = 0x02
MB_BADDATA = 0x03
MB_SLAVEFAILURE = 0x04
MB_BUSY = 0x06
MB_NORESP = 0x0B
MB_DEVNOTSET = 0x10
MB_TIMEOUT = -1
MB_CRC_ERROR = -5

# MODBUS CRC lookup tables
MODBUS_CRC_MSB = bytes([
    0x00,0xC1,0x81,0x40, 0x01,0xC0,0x80,0x41, 0x01,0xC0,0x80,0x41, 0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41, 0x00,0xC1,0x81,0x40, 0x00,0xC1,0x81,0x40, 0x01,0xC0,0x80,0x41,
    0x01,0xC0,0x80,0x41, 0x00,0xC1,0x81,0x40, 0x00,0xC1,0x81,0x40, 0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40, 0x01,0xC0,0x80,0x41, 0x01,0xC0,0x80,0x41, 0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41, 0x00,0xC1,0x81,0x40, 0x00,0xC1,0x81,0x40, 0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40, 0x01,0xC0,0x80,0x41, 0x01,0xC0,0x80,0x41, 0x00,0xC1,0x81,0x40,
    0x00,0xC1,0x81,0x40, 0x01,0xC0,0x80,0x41, 0x01,0xC0,0x80,0x41, 0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41, 0x00,0xC1,0x81,0x40, 0x00,0xC1,0x81,0x40, 0x01,0xC0,0x80,0x41,
    0x01,0xC0,0x80,0x41, 0x00,0xC1,0x81,0x40, 0x00,0xC1,0x81,0x40, 0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40, 0x01,0xC0,0x80,0x41, 0x01,0xC0,0x80,0x41, 0x00,0xC1,0x81,0x40,
    0x00,0xC1,0x81,0x40, 0x01,0xC0,0x80,0x41, 0x01,0xC0,0x80,0x41, 0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41, 0x00,0xC1,0x81,0x40, 0x00,0xC1,0x81,0x40, 0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40, 0x01,0xC0,0x80,0x41, 0x01,0xC0,0x80,0x41, 0x00,0xC1,0x81,0x40,
    0x01,0xC0,0x80,0x41, 0x00,0xC1,0x81,0x40, 0x00,0xC1,0x81,0x40, 0x01,0xC0,0x80,0x41,
    0x01,0xC0,0x80,0x41, 0x00,0xC1,0x81,0x40, 0x00,0xC1,0x81,0x40, 0x01,0xC0,0x80,0x41,
    0x00,0xC1,0x81,0x40, 0x01,0xC0,0x80,0x41, 0x01,0xC0,0x80,0x41, 0x00,0xC1,0x81,0x40
])

MODBUS_CRC_LSB = bytes([
    0x00,0xC0,0xC1,0x01, 0xC3,0x03,0x02,0xC2, 0xC6,0x06,0x07,0xC7, 0x05,0xC5,0xC4,0x04,
    0xCC,0x0C,0x0D,0xCD, 0x0F,0xCF,0xCE,0x0E, 0x0A,0xCA,0xCB,0x0B, 0xC9,0x09,0x08,0xC8,
    0xD8,0x18,0x19,0xD9, 0x1B,0xDB,0xDA,0x1A, 0x1E,0xDE,0xDF,0x1F, 0xDD,0x1D,0x1C,0xDC,
    0x14,0xD4,0xD5,0x15, 0xD7,0x17,0x16,0xD6, 0xD2,0x12,0x13,0xD3, 0x11,0xD1,0xD0,0x10,
    0xF0,0x30,0x31,0xF1, 0x33,0xF3,0xF2,0x32, 0x36,0xF6,0xF7,0x37, 0xF5,0x35,0x34,0xF4,
    0x3C,0xFC,0xFD,0x3D, 0xFF,0x3F,0x3E,0xFE, 0xFA,0x3A,0x3B,0xFB, 0x39,0xF9,0xF8,0x38,
    0x28,0xE8,0xE9,0x29, 0xEB,0x2B,0x2A,0xEA, 0xEE,0x2E,0x2F,0xEF, 0x2D,0xED,0xEC,0x2C,
    0xE4,0x24,0x25,0xE5, 0x27,0xE7,0xE6,0x26, 0x22,0xE2,0xE3,0x23, 0xE1,0x21,0x20,0xE0,
    0xA0,0x60,0x61,0xA1, 0x63,0xA3,0xA2,0x62, 0x66,0xA6,0xA7,0x67, 0xA5,0x65,0x64,0xA4,
    0x6C,0xAC,0xAD,0x6D, 0xAF,0x6F,0x6E,0xAE, 0xAA,0x6A,0x6B,0xAB, 0x69,0xA9,0xA8,0x68,
    0x78,0xB8,0xB9,0x79, 0xBB,0x7B,0x7A,0xBA, 0xBE,0x7E,0x7F,0xBF, 0x7D,0xBD,0xBC,0x7C,
    0xB4,0x74,0x75,0xB5, 0x77,0xB7,0xB6,0x76, 0x72,0xB2,0xB3,0x73, 0xB1,0x71,0x70,0xB0,
    0x50,0x90,0x91,0x51, 0x93,0x53,0x52,0x92, 0x96,0x56,0x57,0x97, 0x55,0x95,0x94,0x54,
    0x9C,0x5C,0x5D,0x9D, 0x5F,0x9F,0x9E,0x5E, 0x5A,0x9A,0x9B,0x5B, 0x99,0x59,0x58,0x98,
    0x88,0x48,0x49,0x89, 0x4B,0x8B,0x8A,0x4A, 0x4E,0x8E,0x8F,0x4F, 0x8D,0x4D,0x4C,0x8C,
    0x44,0x84,0x85,0x45, 0x87,0x47,0x46,0x86, 0x82,0x42,0x43,0x83, 0x41,0x81,0x80,0x40
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


def modbus_crc(data):
    """Alias for crc16_modbus for backward compatibility"""
    return crc16_modbus(data)


def check_crc(frame: bytes) -> bool:
    """Check if frame has valid CRC"""
    if len(frame) < 4:
        return False
    rx_crc = frame[-2] | (frame[-1] << 8)  # LSB then MSB on the wire
    calc_crc = crc16_modbus(frame[:-2])
    return rx_crc == calc_crc


def is_plausible_addr(a: int) -> bool:
    """Check if address is valid MODBUS address (0-247)"""
    return 0 <= a <= 247


def is_plausible_func(f: int) -> bool:
    """Check if function code is valid (0-127 or 0x80-0xFF for exceptions)"""
    return (0 <= f <= 0x7F) or (0x80 <= f <= 0xFF)


class ModbusFrameSplitter:
    """
    Split a byte chunk (may contain multiple RTU frames) into valid frames
    using heuristics and CRC validation (from picoMODBUStest.py)
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


class DCMHardware:
    """Hardware abstraction for DCM board"""
    
    def __init__(self, modbus_uart_id=0, modbus_de_pin=None,
                 addr_pins=None, atten_pins=None,
                 dig_in_pin=None, opt_rx1_uart_id=None, opt_rx2_uart_id=None):
        """
        Initialize DCM hardware
        
        Args:
            modbus_uart_id: UART ID for MODBUS (default 0, typically UART0)
            modbus_de_pin: Pin number for RS-485 DE/RE control (default None, use Pin(2) for PE2 equivalent)
            addr_pins: List of pins for reading MODBUS address (default None, will try to read from hardware)
            atten_pins: List of 8 pins for attenuator output (default None, will use GPIO pins)
            dig_in_pin: Pin for digital input (default None)
            opt_rx1_uart_id: UART ID for optical receiver 1 (default None)
            opt_rx2_uart_id: UART ID for optical receiver 2 (default None)
        """
        # MODBUS UART and RS-485 control (8E1: 8 bits, EVEN parity, 1 stop bit)
        self.modbus_uart = UART(modbus_uart_id, MODBUS_BAUD, tx=Pin(0), rx=Pin(1), bits=8, parity=0, stop=1)
        if modbus_de_pin is None:
            self.de_pin = Pin(2, Pin.OUT)  # PE2 equivalent
        else:
            self.de_pin = Pin(modbus_de_pin, Pin.OUT)
        self.de_pin.value(0)  # Start in receive mode
        
        # Address pins - read MODBUS address from hardware
        # On Rabbit: PB1=AD1, PB2=AD2, PB3=AD3, PB4=AD4, PB5=AD0
        # Address = (PB5<<4) | (PB1) | (PB2<<1) | (PB3<<2) | (PB4<<3)
        if addr_pins is None:
            # Default: try to read from pins if available
            # User should configure based on actual hardware
            self.mb_address = 1  # Default address
        else:
            port_b = 0
            for i, pin_num in enumerate([5, 1, 2, 3, 4]):  # PB5, PB1, PB2, PB3, PB4
                if i < len(addr_pins):
                    pin = Pin(addr_pins[i], Pin.IN, Pin.PULL_DOWN)
                    if pin.value():
                        if i == 0:  # PB5 (AD0)
                            port_b |= 0x20
                        else:  # PB1-PB4 (AD1-AD4)
                            port_b |= (1 << i)
            # Calculate address: (PB5>>5) | (PB1-PB4 masked and shifted)
            self.mb_address = ((port_b & 0x1E) | ((port_b & 0x20) >> 5))
        
        # Attenuator control - 8-bit output (Port A equivalent)
        if atten_pins is None:
            # Default: use GPIO pins 16-23 for 8-bit attenuator output
            self.atten_pins = [Pin(i, Pin.OUT) for i in range(16, 24)]
        else:
            self.atten_pins = [Pin(p, Pin.OUT) for p in atten_pins]
        
        # Digital input (PC3 equivalent for LO2_LOCK monitor)
        if dig_in_pin is None:
            self.dig_in_pin = Pin(3, Pin.IN, Pin.PULL_DOWN)
        else:
            self.dig_in_pin = Pin(dig_in_pin, Pin.IN, Pin.PULL_DOWN)
        
        # Optical receiver UARTs (Serial C and D)
        if opt_rx1_uart_id is not None:
            self.opt_rx1_uart = UART(opt_rx1_uart_id, 9600, tx=Pin(4), rx=Pin(5))
        else:
            self.opt_rx1_uart = None
        
        if opt_rx2_uart_id is not None:
            self.opt_rx2_uart = UART(opt_rx2_uart_id, 9600, tx=Pin(6), rx=Pin(7))
        else:
            self.opt_rx2_uart = None
        
        # ADC (if needed - can be added later)
        # self.adc = ADC(Pin(26))  # Example ADC pin
        
        # Initialize state variables
        self.atten_set = 0xFF  # Initial attenuation: 15dB
        self.atten_state = 0xFF
        self.atten_mode = 0  # 0=normal, 1=table
        self.atten_index = 0
        self.atten_offset = 0
        self.atten_table = [0xFF] * 50  # 50 entries, all initialized to 15dB
        
        self.dig_out_value = 0
        
        # Optical receiver message buffers
        self.opt_rx1_msg = bytearray(BUFFER_LEN)
        self.opt_rx2_msg = bytearray(BUFFER_LEN)
        self.opt_rx1_msg[:] = b'opt_rx1 status buffer is empty\x00' + b'\x00' * (BUFFER_LEN - 33)
        self.opt_rx2_msg[:] = b'opt_rx2 status buffer is empty\x00' + b'\x00' * (BUFFER_LEN - 33)
        
        # Initialize attenuator output
        self._write_attenuator(self.atten_state)
    
    def _write_attenuator(self, value):
        """Write 8-bit value to attenuator output pins"""
        for i in range(8):
            self.atten_pins[i].value((value >> i) & 1)
    
    def dig_in(self, channel):
        """Read digital input channel (0 = LO2_LOCK monitor)"""
        if channel == 0:
            return 1 if self.dig_in_pin.value() else 0
        return 0
    
    def dig_in_bank(self, bank):
        """Read digital input bank (ADC equivalent)"""
        # Bank 0: V POL Detector Output Monitor (ADC channel 8)
        # Bank 1: H POL Detector Output Monitor (ADC channel 9)
        # Bank 2: Attenuator values
        # Bank -1: Firmware revision
        if bank == -1:
            return FIRMWARE_REV
        elif bank == 0:
            # Return ADC reading for channel 8 (placeholder)
            return 0
        elif bank == 1:
            # Return ADC reading for channel 9 (placeholder)
            return 0
        elif bank == 2:
            # Return attenuator values
            return self.atten_state
        return 0
    
    def dig_out(self, channel, state):
        """Digital output (for optical receiver status reads)"""
        if channel == 0 and self.opt_rx1_uart:
            # Read status from OPT_RX1
            cmd = b"READ\n"
            self.opt_rx1_uart.write(cmd)
            time.sleep_ms(10)
            
            # Clear buffer
            self.opt_rx1_msg[:] = b'\x00' * BUFFER_LEN
            
            # Read response with timeout
            timeout = 600
            count = 0
            while count < timeout:
                if self.opt_rx1_uart.any():
                    data = self.opt_rx1_uart.read(BUFFER_LEN)
                    if data:
                        length = min(len(data), BUFFER_LEN)
                        self.opt_rx1_msg[:length] = data[:length]
                        break
                time.sleep_ms(1)
                count += 1
            
            if count >= timeout:
                self.opt_rx1_msg[:] = b'opt_rx1 read status failed\x00' + b'\x00' * (BUFFER_LEN - 27)
        
        elif channel == 1 and self.opt_rx2_uart:
            # Read status from OPT_RX2
            cmd = b"READ\n"
            self.opt_rx2_uart.write(cmd)
            time.sleep_ms(10)
            
            # Clear buffer
            self.opt_rx2_msg[:] = b'\x00' * BUFFER_LEN
            
            # Read response with timeout
            timeout = 600
            count = 0
            while count < timeout:
                if self.opt_rx2_uart.any():
                    data = self.opt_rx2_uart.read(BUFFER_LEN)
                    if data:
                        length = min(len(data), BUFFER_LEN)
                        self.opt_rx2_msg[:length] = data[:length]
                        break
                time.sleep_ms(1)
                count += 1
            
            if count >= timeout:
                self.opt_rx2_msg[:] = b'opt_rx2 read status failed\x00' + b'\x00' * (BUFFER_LEN - 27)
    
    def dig_out_bank(self, bank, data):
        """Digital output bank (attenuator control)"""
        # Split 16-bit data into two bytes
        byte_low = data & 0xFF
        byte_high = (data >> 8) & 0xFF
        
        if bank == 0:
            # Set attenuator value: combine high and low nibbles
            # Note: data format is high nibble in MSB, low nibble in LSB
            self.atten_set = ((byte_high << 4) | (byte_low & 0x0F)) & 0xFF
        elif bank == 1:
            # Set attenuator mode: 0=normal, 1=table
            self.atten_mode = 0 if data == 0 else 1
        elif bank == 2:
            # Set attenuator table index and offset
            self.atten_index = byte_low & 0xFF
            # Convert unsigned to signed byte
            if byte_high > 127:
                self.atten_offset = byte_high - 256
            else:
                self.atten_offset = byte_high
        elif 3 <= bank < 53:
            # Set attenuator table entry
            self.atten_table[bank - 3] = ((byte_high << 4) | (byte_low & 0x0F)) & 0xFF
    
    def rd_holding_reg(self, address):
        """Read holding register"""
        if address <= (BUFFER_LEN // 2) - 1:
            # OPT_RX1 buffer - each address represents 2 bytes (1 word)
            byte1_idx = address * 2
            byte2_idx = byte1_idx + 1
            if byte2_idx < BUFFER_LEN:
                byte1 = self.opt_rx1_msg[byte1_idx]
                byte2 = self.opt_rx1_msg[byte2_idx]
                # Return as MODBUS word (MSB first)
                return (byte1 << 8) | byte2
        elif address <= BUFFER_LEN - 1:
            # OPT_RX2 buffer
            byte1_idx = (address - (BUFFER_LEN // 2)) * 2
            byte2_idx = byte1_idx + 1
            if byte2_idx < BUFFER_LEN:
                byte1 = self.opt_rx2_msg[byte1_idx]
                byte2 = self.opt_rx2_msg[byte2_idx]
                # Return as MODBUS word (MSB first)
                return (byte1 << 8) | byte2
        elif address == BUFFER_LEN:
            # Attenuator state
            byte_low = self.atten_state & 0x0F
            byte_high = (self.atten_state & 0xF0) >> 4
            # Return as MODBUS word (MSB first)
            return (byte_high << 8) | byte_low
        elif address == BUFFER_LEN + 1:
            # Attenuator index and offset (index in low byte, offset in high byte)
            # Return as MODBUS word (MSB first)
            offset_byte = self.atten_offset & 0xFF
            index_byte = self.atten_index & 0xFF
            return (offset_byte << 8) | index_byte
        elif address == BUFFER_LEN + 2:
            # Attenuator mode
            return self.atten_mode
        elif address <= BUFFER_LEN + 52:
            # Attenuator table
            byte_low = self.atten_table[address - BUFFER_LEN - 3] & 0x0F
            byte_high = (self.atten_table[address - BUFFER_LEN - 3] & 0xF0) >> 4
            # Return as MODBUS word (MSB first)
            return (byte_high << 8) | byte_low
        return 0
    
    def event_trigger(self):
        """Update attenuator state based on mode"""
        if self.atten_mode == 1:
            # Table mode: apply offset
            offset = self.atten_offset - 15
            table_val = self.atten_table[self.atten_index]
            
            byte1 = (table_val & 0x0F) + offset
            byte2 = ((table_val & 0xF0) >> 4) + offset
            
            # Clamp to 0-15
            byte1 = max(0, min(15, byte1))
            byte2 = max(0, min(15, byte2))
            
            self.atten_state = ((byte2 << 4) | byte1) & 0xFF
        else:
            # Normal mode
            self.atten_state = self.atten_set
        
        # Write to hardware
        self._write_attenuator(self.atten_state)


class ModbusSlave:
    """MODBUS RTU Slave implementation"""
    
    def __init__(self, hardware):
        self.hw = hardware
        self.cmd_buffer = bytearray(256)
        self.reply_buffer = bytearray(256)
        self.reply_ptr = 0
    
    def _reply_init(self, cmd):
        """Initialize reply buffer"""
        self.reply_ptr = 0
        self.reply_buffer[self.reply_ptr] = self.hw.mb_address
        self.reply_ptr += 1
        self.reply_buffer[self.reply_ptr] = cmd
        self.reply_ptr += 1
    
    def _reply_byte(self, byte_val):
        """Add byte to reply"""
        if self.reply_ptr < len(self.reply_buffer):
            self.reply_buffer[self.reply_ptr] = byte_val & 0xFF
            self.reply_ptr += 1
            return MB_SUCCESS
        return 1
    
    def _reply_word(self, word_val):
        """Add word (MSB first) to reply"""
        self._reply_byte((word_val >> 8) & 0xFF)
        self._reply_byte(word_val & 0xFF)
        return MB_SUCCESS
    
    def _reply_err(self, err_code):
        """Generate error reply"""
        self._reply_init(self.cmd_buffer[1] | 0x80)
        self._reply_byte(err_code)
        return err_code
    
    def _cmd_word(self, offset):
        """Extract word (MSB first) from command"""
        if offset + 1 < len(self.cmd_buffer):
            return (self.cmd_buffer[offset] << 8) | self.cmd_buffer[offset + 1]
        return 0
    
    def _coil_rd(self, read_func):
        """Read coils (function 0x01 or 0x02)"""
        coil_nbr = self._cmd_word(2)
        coil_cnt = self._cmd_word(4)
        
        byte_cnt = (coil_cnt + 7) // 8
        self._reply_byte(byte_cnt)
        
        coil_idx = coil_nbr
        for byte_idx in range(byte_cnt):
            acc = 0
            mask = 0x01
            for _ in range(8):
                if coil_cnt > 0:
                    state = [0]  # Use list for mutable reference
                    err = read_func(coil_idx, state)
                    if err == MB_SUCCESS:
                        if state[0]:
                            acc |= mask
                    else:
                        return err
                    coil_idx += 1
                    coil_cnt -= 1
                mask <<= 1
            self._reply_byte(acc)
        
        return MB_SUCCESS
    
    def _reg_rd(self, read_func):
        """Read registers (function 0x03 or 0x04)"""
        reg_nbr = self._cmd_word(2)
        reg_cnt = self._cmd_word(4)
        
        self._reply_byte(reg_cnt * 2)
        
        for _ in range(reg_cnt):
            value = [0]  # Use list for mutable reference
            err = read_func(reg_nbr, value)
            if err == MB_SUCCESS:
                self._reply_word(value[0])
            else:
                return err
            reg_nbr += 1
        
        return MB_SUCCESS
    
    def _force_coil(self):
        """Write single coil (function 0x05)"""
        coil_nbr = self._cmd_word(2)
        coil_val = self._cmd_word(4)
        
        self._reply_word(coil_nbr)
        self._reply_word(coil_val)
        
        state = 1 if coil_val != 0 else 0
        self.hw.dig_out(coil_nbr, state)
        
        # Update dig_out_value
        mask = 1 << coil_nbr
        if state:
            self.hw.dig_out_value |= mask
        else:
            self.hw.dig_out_value &= ~mask
        
        return MB_SUCCESS
    
    def _write_reg(self):
        """Write single register (function 0x06)"""
        reg_nbr = self._cmd_word(2)
        reg_val = self._cmd_word(4)
        
        self._reply_word(reg_nbr)
        self._reply_word(reg_val)
        
        self.hw.dig_out_bank(reg_nbr, reg_val)
        return MB_SUCCESS
    
    def _force_coils(self):
        """Write multiple coils (function 0x0F)"""
        coil_nbr = self._cmd_word(2)
        coil_cnt = self._cmd_word(4)
        byte_cnt = self.cmd_buffer[6]
        
        self._reply_word(coil_nbr)
        self._reply_word(coil_cnt)
        
        data_idx = 7
        coil_idx = coil_nbr
        for _ in range(byte_cnt):
            byte_val = self.cmd_buffer[data_idx]
            data_idx += 1
            for bit in range(8):
                if coil_cnt > 0:
                    state = 1 if (byte_val & (1 << bit)) else 0
                    self.hw.dig_out(coil_idx, state)
                    coil_idx += 1
                    coil_cnt -= 1
        
        return MB_SUCCESS
    
    def _write_regs(self):
        """Write multiple registers (function 0x10)"""
        reg_nbr = self._cmd_word(2)
        reg_cnt = self._cmd_word(4)
        
        self._reply_word(reg_nbr)
        self._reply_word(reg_cnt)
        
        data_idx = 7
        for _ in range(reg_cnt):
            if data_idx + 1 < len(self.cmd_buffer):
                reg_val = (self.cmd_buffer[data_idx] << 8) | self.cmd_buffer[data_idx + 1]
                self.hw.dig_out_bank(reg_nbr, reg_val)
                reg_nbr += 1
                data_idx += 2
            else:
                return MB_BADDATA
        
        return MB_SUCCESS
    
    def _reg_mask(self):
        """Mask write register (function 0x16)"""
        reg_nbr = self._cmd_word(2)
        and_mask = self._cmd_word(4)
        or_mask = self._cmd_word(6)
        
        self._reply_word(reg_nbr)
        self._reply_word(and_mask)
        self._reply_word(or_mask)
        
        # Read current value
        curr_val_ref = [0]
        self.mbs_reg_out_rd(reg_nbr, curr_val_ref)
        curr_val = curr_val_ref[0]
        
        new_val = (curr_val & and_mask) | (or_mask & ~and_mask)
        self.hw.dig_out_bank(reg_nbr, new_val)
        return MB_SUCCESS
    
    def _reg_rd_wr(self):
        """Read/write multiple registers (function 0x17)"""
        # Write registers first
        write_reg_nbr = self._cmd_word(6)
        write_reg_cnt = self._cmd_word(8)
        if 10 < len(self.cmd_buffer):
            byte_cnt = self.cmd_buffer[10]
        else:
            return MB_BADDATA
        
        data_idx = 11
        for _ in range(write_reg_cnt):
            if data_idx + 1 < len(self.cmd_buffer):
                reg_val = (self.cmd_buffer[data_idx] << 8) | self.cmd_buffer[data_idx + 1]
                self.hw.dig_out_bank(write_reg_nbr, reg_val)
                write_reg_nbr += 1
                data_idx += 2
            else:
                return MB_BADDATA
        
        # Then read registers
        read_reg_nbr = self._cmd_word(2)
        read_reg_cnt = self._cmd_word(4)
        
        self._reply_byte(read_reg_cnt * 2)
        for _ in range(read_reg_cnt):
            value = self.hw.rd_holding_reg(read_reg_nbr)
            self._reply_word(value)
            read_reg_nbr += 1
        
        return MB_SUCCESS
    
    def mbs_dig_out_rd(self, output_nbr, state_ref):
        """Read digital output state"""
        if output_nbr > 7:
            return MB_BADADDR
        state_ref[0] = (self.hw.dig_out_value >> output_nbr) & 0x01
        return MB_SUCCESS
    
    def mbs_dig_in(self, input_nbr, state_ref):
        """Read digital input"""
        if input_nbr != 0:
            return MB_BADADDR
        state_ref[0] = self.hw.dig_in(input_nbr)
        return MB_SUCCESS
    
    def mbs_reg_out_rd(self, reg_nbr, value_ref):
        """Read holding register"""
        if reg_nbr > BUFFER_LEN + 52:
            return MB_BADADDR
        value_ref[0] = self.hw.rd_holding_reg(reg_nbr)
        return MB_SUCCESS
    
    def mbs_reg_in(self, reg_nbr, value_ref):
        """Read input register"""
        if reg_nbr > 1 and reg_nbr < 65535:
            return MB_BADADDR
        value_ref[0] = self.hw.dig_in_bank(reg_nbr)
        return MB_SUCCESS
    
    def execute(self, cmd_data):
        """Execute MODBUS command"""
        if len(cmd_data) < 2:
            return None
        
        # Copy command to buffer
        self.cmd_buffer[:len(cmd_data)] = cmd_data
        
        cmd = self.cmd_buffer[1]
        self._reply_init(cmd)
        
        err = MB_SUCCESS
        
        if cmd == 0x01:  # Read Coil Status
            err = self._coil_rd(self.mbs_dig_out_rd)
        elif cmd == 0x02:  # Read Input Status
            err = self._coil_rd(self.mbs_dig_in)
        elif cmd == 0x03:  # Read Holding Registers
            err = self._reg_rd(self.mbs_reg_out_rd)
        elif cmd == 0x04:  # Read Input Registers
            err = self._reg_rd(self.mbs_reg_in)
        elif cmd == 0x05:  # Write Single Coil
            err = self._force_coil()
        elif cmd == 0x06:  # Write Single Register
            err = self._write_reg()
        elif cmd == 0x0F:  # Write Multiple Coils
            err = self._force_coils()
        elif cmd == 0x10:  # Write Multiple Registers
            err = self._write_regs()
        elif cmd == 0x16:  # Mask Write Register
            err = self._reg_mask()
        elif cmd == 0x17:  # Read/Write Multiple Registers
            err = self._reg_rd_wr()
        elif cmd == 0x43:  # Load attenuator set value (same as 0x06)
            err = self._write_reg()
        elif cmd == 0x44:  # Load attenuator table (same as 0x10)
            err = self._write_regs()
        else:
            err = MB_BADFUNC
        
        if err != MB_SUCCESS:
            self._reply_err(err)
        
        return self.reply_buffer[:self.reply_ptr] if self.reply_ptr > 0 else None


class DCM:
    """Main DCM controller class"""
    
    def __init__(self, modbus_uart_id=0, modbus_de_pin=2, **kwargs):
        """
        Initialize DCM controller
        
        Args:
            modbus_uart_id: UART ID for MODBUS communication
            modbus_de_pin: Pin for RS-485 direction control
            **kwargs: Additional hardware configuration (see DCMHardware.__init__)
        """
        self.hw = DCMHardware(modbus_uart_id=modbus_uart_id,
                             modbus_de_pin=modbus_de_pin,
                             **kwargs)
        self.modbus = ModbusSlave(self.hw)
        
        # Frame splitting for receiving concatenated frames
        self._frame_splitter = ModbusFrameSplitter()
        self._rx_buf = bytearray()
        self._frame_queue = []  # Queue for multiple frames received in one chunk
        self._collecting = False
        self._last_rx_us = 0
        
        # Frame detection parameters (matching picoMODBUStest.py)
        # At 1843200 baud: 3.5 character times ≈ 19 µs, use 120 µs to be safe
        self.INTER_FRAME_US = 120
    
    def modbus_serial_rx(self):
        """
        Receive MODBUS message from serial port using frame splitting approach.
        Returns payload data (without CRC) on success, 0 if no data, MB_CRC_ERROR if CRC invalid.
        """
        now = time.ticks_us()
        
        # Read available bytes from UART
        n = self.hw.modbus_uart.any()
        if n > 0:
            data = self.hw.modbus_uart.read(min(n, 512))
            if data:
                self._rx_buf.extend(data)
                self._collecting = True
                self._last_rx_us = now
        
        if self._collecting:
            idle = time.ticks_diff(now, self._last_rx_us)
            if idle >= self.INTER_FRAME_US:
                # Frame group complete - split into individual frames
                chunk = bytes(self._rx_buf)
                self._rx_buf = bytearray()
                self._collecting = False
                
                frames, leftover = self._frame_splitter.split(chunk)
                
                # Keep leftover (partial frame) for next round
                if leftover:
                    self._rx_buf.extend(leftover)
                    self._collecting = True
                    self._last_rx_us = time.ticks_us()
                
                # Queue all valid frames for processing
                for raw_msg in frames:
                    payload = raw_msg[:-2]  # Remove CRC
                    if check_crc(raw_msg):
                        self._frame_queue.append(payload)
                
        # Return next queued frame if available
        if self._frame_queue:
            return self._frame_queue.pop(0)
        
        return 0  # No complete frame yet
    
    def modbus_serial_tx(self, data):
        """Transmit MODBUS message to serial port"""
        if not data:
            return
        
        # Calculate and append CRC (LSB first, then MSB on the wire)
        crc = crc16_modbus(data)
        tx_data = bytearray(data)
        tx_data.append(crc & 0xFF)  # LSB first
        tx_data.append((crc >> 8) & 0xFF)  # MSB second
        
        # Enable transmit
        self.hw.de_pin.value(1)
        time.sleep_us(100)  # Small delay
        
        # Send data
        self.hw.modbus_uart.write(tx_data)
        
        # Wait for transmission to complete
        while self.hw.modbus_uart.txdone() == False:
            time.sleep_us(10)
        
        # Disable transmit (return to receive mode)
        self.hw.de_pin.value(0)
    
    def tick(self):
        """Main MODBUS tick function - call periodically"""
        # Receive message
        rx_data = self.modbus_serial_rx()
        
        if rx_data == 0:
            return  # No data
        elif rx_data == MB_CRC_ERROR:
            return  # CRC error, ignore
        
        if len(rx_data) < 2:
            return
        
        addr = rx_data[0]
        func = rx_data[1]
        
        # Check if message is for this device
        if addr == self.hw.mb_address:
            # Execute command
            reply = self.modbus.execute(rx_data)
            if reply:
                self.modbus_serial_tx(reply)
        elif addr == 0 and func == 0x45:  # Event trigger (0x45 = 69 decimal = 'E')
            self.hw.event_trigger()
        elif addr == 0 and func in [0x44, 0x43]:  # Broadcast commands (0x44=68='D', 0x43=67='C')
            # Execute broadcast command but don't send reply
            self.modbus.execute(rx_data)


def main():
    """Main program loop"""
    # Initialize DCM
    dcm = DCM()
    
    # Main loop
    while True:
        dcm.tick()
        time.sleep_ms(1)  # Small delay to prevent tight loop


if __name__ == "__main__":
    main()
