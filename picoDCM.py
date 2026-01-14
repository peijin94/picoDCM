"""
DCM Module for Raspberry Pi Pico (MicroPython)
Migrated from Rabbit Single Board Computer C code

This module implements a MODBUS RTU slave for the EOVSA Downconverter Module (DCM)
with support for:
- MODBUS RTU over RS-485
- Digital I/O
- Analog inputs (ADC)
- Attenuator control (normal and table modes)
"""

from machine import UART, Pin, ADC
import struct
import time

# Constants
BUFFER_LEN = 132
FIRMWARE_REV = 7
MODBUS_BAUD = 1843200

# Pin definitions
MODBUS_TX_PIN = 0    # GP0 - UART TX for MODBUS
MODBUS_RX_PIN = 1    # GP1 - UART RX for MODBUS
MODBUS_DE_PIN = 2    # GP2 - RS-485 DE/RE control pin

# Address pins (AD0-AD4)
AD0_PIN = 3   # GP3 - PB5 (AD0)
AD1_PIN = 4   # GP4 - PB1 (AD1)
AD2_PIN = 5   # GP5 - PB2 (AD2)
AD3_PIN = 6   # GP6 - PB3 (AD3)
AD4_PIN = 7   # GP7 - PB4 (AD4)

# Attenuator output pins (PA0-PA7, Port A equivalent)
PA0_PIN = 10  # GP10
PA1_PIN = 11  # GP11
PA2_PIN = 12  # GP12
PA3_PIN = 13  # GP13
PA4_PIN = 14  # GP14
PA5_PIN = 15  # GP15
PA6_PIN = 16  # GP16
PA7_PIN = 17  # GP17
ATTEN_PINS = list(range(10, 18))  # GP10-GP17

# Digital input pin - DEACTIVATED per user request.
# DIG_IN_PIN = 3  # GP3 - LO2_LOCK monitor (same as AD0, check hardware design)

# ADC input pins
ADC_LN0_PIN = 26  # GP26 - LN0 detector output
ADC_LN1_PIN = 27  # GP27 - LN1 detector output
ADC_A2_PIN = 28   # GP28 - A2 ADC input

# Debug flag - diagnostics print when DEBUG_ENV=False (inverted logic for production diagnostics)
DEBUG_ENV = False  # Set to True to suppress diagnostics, False to print them
PRINT_MY_ADDRESS_ONLY = True  # Set to True to print only my address, False to print all messages
PRINT_TICK = False  # Set to True to print tick messages, False to print only my address

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
    Calculate MODBUS CRC16 using lookup tables (matches Rabbit C MODBUS_CRC function).
    Returns CRC16 value. CRC is appended MSB first, then LSB on the wire.
    """
    c_hi = 0xFF
    c_lo = 0xFF
    for b in data:
        w = c_hi ^ b
        c_hi = c_lo ^ MODBUS_CRC_MSB[w]
        c_lo = MODBUS_CRC_LSB[w]
    return ((c_hi << 8) | c_lo) & 0xFFFF


def modbus_crc(data):
    """Alias for crc16_modbus for backward compatibility"""
    return crc16_modbus(data)


def check_crc(frame: bytes) -> bool:
    """Check if frame has valid CRC"""
    if len(frame) < 4:
        return False
    rx_crc = (frame[-2] << 8) | frame[-1]  # MSB then LSB on the wire
    calc_crc = crc16_modbus(frame[:-2])
    return rx_crc == calc_crc


def is_plausible_addr(a: int) -> bool:
    """Check if address is valid MODBUS address (0-247)"""
    return 0 <= a <= 247


def is_plausible_func(f: int) -> bool:
    """Check if function code is valid (0-127 or 0x80-0xFF for exceptions)"""
    return (0 <= f <= 0x7F) or (0x80 <= f <= 0xFF)


def hex_bytes(b: bytes, sep=" ") -> str:
    """Convert bytes to hex string"""
    return sep.join("{:02X}".format(x) for x in b)


def u16_be(b0: int, b1: int) -> int:
    """Convert two bytes to big-endian uint16"""
    return (b0 << 8) | b1


class DCMDecoder:
    """
    Decode MODBUS frames to human-readable format (matching picoDCMdecoder.py).
    Can decode full frames (with CRC) or payload-only data (without CRC).
    """
    
    def decode(self, frame_or_payload: bytes, has_crc: bool = True) -> str:
        """
        Decode MODBUS frame or payload to formatted string.
        
        Args:
            frame_or_payload: Full frame (with CRC) or payload (without CRC)
            has_crc: True if frame includes CRC (last 2 bytes), False if payload only
        
        Returns:
            Formatted string: addr=XX | func=0xXXXX | funcName=NAME | payload=...
        """
        if has_crc:
            if len(frame_or_payload) < 4:
                return f"INVALID (too short: {len(frame_or_payload)} bytes)"
            frame = frame_or_payload
            payload = frame[2:-2]
        else:
            if len(frame_or_payload) < 2:
                return f"INVALID (too short: {len(frame_or_payload)} bytes)"
            frame = frame_or_payload
            payload = frame[2:] if len(frame) > 2 else b""
        
        addr = frame[0]
        func = frame[1]
        
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
            print(f"[WRM_RESP] payload={hex_bytes(payload)}")
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
                 dig_in_pin=None):
        """
        Initialize DCM hardware
        
        Args:
            modbus_uart_id: UART ID for MODBUS (default 0, typically UART0)
            modbus_de_pin: Pin number for RS-485 DE/RE control (default None, use Pin(2) for PE2 equivalent)
            addr_pins: List of pins for reading MODBUS address (default None, will try to read from hardware)
            atten_pins: List of 8 pins for attenuator output (default None, will use GPIO pins)
            dig_in_pin: Pin for digital input (default None)
        """
        # MODBUS UART and RS-485 control (8E1: 8 bits, EVEN parity, 1 stop bit)
        self.modbus_uart = UART(modbus_uart_id, MODBUS_BAUD, tx=Pin(MODBUS_TX_PIN), 
            rx=Pin(MODBUS_RX_PIN), bits=8, parity=0, stop=1)
        if modbus_de_pin is None:
            self.de_pin = Pin(MODBUS_DE_PIN, Pin.OUT)  # GP2 - RS-485 DE/RE control
        else:
            self.de_pin = Pin(modbus_de_pin, Pin.OUT)
        self.de_pin.value(0)  # Start in receive mode
        
        # Address pins - read MODBUS address from hardware
        # On Rabbit: PB1=AD1, PB2=AD2, PB3=AD3, PB4=AD4, PB5=AD0
        # Address = ((Port_B & 0x1E) | ((Port_B & 0x20)>>5))
        if addr_pins is None:
            # Read from hardware pins
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
            
            # Calculate address: ((Port_B & 0x1E) | ((Port_B & 0x20)>>5))
            self.mb_address = ((port_b & 0x1E) | ((port_b & 0x20) >> 5))
        else:
            # Use provided address pins
            port_b = 0
            for i, pin_num in enumerate([AD0_PIN, AD1_PIN, AD2_PIN, AD3_PIN, AD4_PIN]):
                if i < len(addr_pins):
                    pin = Pin(addr_pins[i], Pin.IN, Pin.PULL_DOWN)
                    if pin.value():
                        if i == 0:  # PB5 (AD0)
                            port_b |= 0x20
                        else:  # PB1-PB4 (AD1-AD4)
                            port_b |= (1 << i)
            # Calculate address: (PB5>>5) | (PB1-PB4 masked and shifted)
            self.mb_address = ((port_b & 0x1E) | ((port_b & 0x20) >> 5))
        
        # Always print the MODBUS address
        print(f"MODBUS Address from pins: {self.mb_address}")
        
        # Attenuator control - 8-bit output (Port A equivalent, GP10-GP17)
        if atten_pins is None:
            # Default: use GPIO pins 10-17 for 8-bit attenuator output
            self.atten_pins = [Pin(i, Pin.OUT) for i in ATTEN_PINS]
        else:
            self.atten_pins = [Pin(p, Pin.OUT) for p in atten_pins]
        
        # Digital input (PC3 equivalent for LO2_LOCK monitor)
        # Note: DIG_IN_PIN is deactivated.
        if dig_in_pin is None:
            self.dig_in_pin = None # Deactivated
        else:
            self.dig_in_pin = Pin(dig_in_pin, Pin.IN, Pin.PULL_DOWN)
        
        # ADC initialization for analog inputs
        self.adc_ln0 = ADC(Pin(ADC_LN0_PIN))  # LN0 detector output (GP26)
        self.adc_ln1 = ADC(Pin(ADC_LN1_PIN))  # LN1 detector output (GP27)
        self.adc_a2 = ADC(Pin(ADC_A2_PIN))    # A2 ADC input (GP28)
        
        # Initialize state variables
        self.atten_set = 0xFF  # Initial attenuation: 15dB
        self.atten_state = 0xFF
        self.atten_mode = 0  # 0=normal, 1=table
        self.atten_index = 0
        self.atten_offset = 0
        self.atten_table = [0xFF] * 50  # 50 entries, all initialized to 15dB
        
        self.dig_out_value = 0
        
        # Initialize attenuator output
        self._write_attenuator(self.atten_state)
    
    def _write_attenuator(self, value):
        """Write 8-bit value to attenuator output pins"""
        for i in range(8):
            self.atten_pins[i].value((value >> i) & 1)
    
    def dig_in(self, channel):
        """Read digital input channel (0 = LO2_LOCK monitor)"""
        if self.dig_in_pin is None:
            return 0  # Pin deactivated
        if channel == 0:
            return 1 if self.dig_in_pin.value() else 0
        return 0
    
    def dig_in_bank(self, bank):
        """Read digital input bank (ADC equivalent)"""
        # Bank 0: V POL Detector Output Monitor (LN0)
        # Bank 1: H POL Detector Output Monitor (LN1)
        # Bank 2: Attenuator values
        # Bank -1: Firmware revision
        if bank == -1:
            return FIRMWARE_REV
        elif bank == 0:
            # Read LN0 (GP26) - 12-bit ADC, read_u16() returns 0-65535
            # Scale to 12-bit (0-4095) to match original behavior
            raw_value = self.adc_ln0.read_u16()
            # Convert 16-bit reading (0-65535) to 12-bit value (0-4095)
            adc_value = (raw_value * 4095) // 65535
            voltage = (raw_value / 65535.0) * 3.3
            if DEBUG_ENV:
                print(f"[ADC] LN0 (GP26): raw={raw_value:5d}, 12-bit={adc_value:4d}, voltage={voltage:.3f}V")
            return adc_value
        elif bank == 1:
            # Read LN1 (GP27) - 12-bit ADC, read_u16() returns 0-65535
            # Scale to 12-bit (0-4095) to match original behavior
            raw_value = self.adc_ln1.read_u16()
            # Convert 16-bit reading (0-65535) to 12-bit value (0-4095)
            adc_value = (raw_value * 4095) // 65535
            voltage = (raw_value / 65535.0) * 3.3
            if DEBUG_ENV:
                print(f"[ADC] LN1 (GP27): raw={raw_value:5d}, 12-bit={adc_value:4d}, voltage={voltage:.3f}V")
            return adc_value
        elif bank == 2:
            # Return attenuator values
            return self.atten_state
        return 0
    
    def dig_out(self, channel, state):
        """Digital output (no-op, reserved for future use)"""
        # Digital output functionality removed - optical receiver support removed
        pass
    
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
            table_index = bank - 3
            table_value = ((byte_high << 4) | (byte_low & 0x0F)) & 0xFF
            self.atten_table[table_index] = table_value
    
    def rd_holding_reg(self, address):
        """Read holding register"""
        if address <= BUFFER_LEN - 1:
            # Optical receiver buffers removed - return 0
            return 0
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
        self.cmd_len = 0  # Actual length of command data
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
        if offset + 1 < self.cmd_len:
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
    
    def _validate_write_bank_and_value(self, bank, value):
        """
        Validate write register and value according to Rabbit C semantics.
        Returns error code if invalid, None if valid.
        Rabbit C: OutRegNbr > 52 => MB_BADADDR
                  For bank 0 and banks 3..52: both bytes must be <= 0x0F else MB_BADDATA
        """
        # Range check (Rabbit: OutRegNbr > 52 => MB_BADADDR)
        if bank < 0 or bank > 52:
            return MB_BADADDR
        
        lo = value & 0xFF
        hi = (value >> 8) & 0xFF
        
        # Nibble-only validation for atten set (bank 0) and table entries (banks 3..52)
        if bank == 0 or (3 <= bank <= 52):
            if lo > 0x0F or hi > 0x0F:
                return MB_BADDATA
        
        # Optional safety: enforce table index bounds (bank 2) to prevent IndexError
        if bank == 2:
            idx = lo & 0xFF
            if idx > 49:
                return MB_BADDATA
        
        return None
    
    def _write_reg(self):
        """Write single register (function 0x06)"""
        reg_nbr = self._cmd_word(2)
        reg_val = self._cmd_word(4)
        
        # Validate before applying (Rabbit validates before digOutBank)
        err = self._validate_write_bank_and_value(reg_nbr, reg_val)
        if err is not None:
            return err
        
        # Reply with register number and value (MODBUS standard)
        self._reply_word(reg_nbr)
        self._reply_word(reg_val)
        
        # Apply the write
        self.hw.dig_out_bank(reg_nbr, reg_val)
        return MB_SUCCESS
    
    def _force_coils(self):
        """Write multiple coils (function 0x0F)"""
        # Validate minimum frame length: 7 bytes (addr+func+start+qty+byteCount)
        if self.cmd_len < 7:
            return MB_BADDATA
        
        coil_nbr = self._cmd_word(2)
        coil_cnt = self._cmd_word(4)
        byte_cnt = self.cmd_buffer[6]
        
        # Validate byteCount: should be ceil(coil_cnt / 8)
        expected_byte_cnt = (coil_cnt + 7) // 8
        if byte_cnt != expected_byte_cnt:
            return MB_BADDATA
        
        # Validate total frame length: 7 + byteCount
        if self.cmd_len < 7 + byte_cnt:
            return MB_BADDATA
        
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
        # Validate minimum frame length: 7 bytes (addr+func+start+qty+byteCount)
        if self.cmd_len < 7:
            return MB_BADDATA
        
        start_reg_nbr = self._cmd_word(2)
        reg_cnt = self._cmd_word(4)
        byte_cnt = self.cmd_buffer[6]
        
        # Validate byteCount: must equal 2 * reg_cnt (each register is 2 bytes)
        expected_byte_cnt = 2 * reg_cnt
        if byte_cnt != expected_byte_cnt:
            return MB_BADDATA
        
        # Validate total frame length: 7 + byteCount
        if self.cmd_len < 7 + byte_cnt:
            return MB_BADDATA
        
        # Validate all registers first (Rabbit validates before applying)
        data_idx = 7
        reg_vals = []
        reg_nbr = start_reg_nbr
        for _ in range(reg_cnt):
            if data_idx + 1 < self.cmd_len:
                reg_val = (self.cmd_buffer[data_idx] << 8) | self.cmd_buffer[data_idx + 1]
                reg_vals.append((reg_nbr, reg_val))
                reg_nbr += 1
                data_idx += 2
            else:
                return MB_BADDATA
        
        # Validate each register
        for bank, val in reg_vals:
            err = self._validate_write_bank_and_value(bank, val)
            if err is not None:
                return err
        
        # Reply with start register and count (MODBUS standard)
        self._reply_word(start_reg_nbr)
        self._reply_word(reg_cnt)
        
        # Apply all writes after validation passes
        for bank, val in reg_vals:
            self.hw.dig_out_bank(bank, val)
        
        return MB_SUCCESS
    
    def _reg_mask(self):
        """Mask write register (function 0x16)"""
        reg_nbr = self._cmd_word(2)
        and_mask = self._cmd_word(4)
        or_mask = self._cmd_word(6)
        
        # Read current value
        curr_val_ref = [0]
        err = self.mbs_reg_out_rd(reg_nbr, curr_val_ref)
        if err != MB_SUCCESS:
            return err
        curr_val = curr_val_ref[0]
        
        new_val = (curr_val & and_mask) | (or_mask & ~and_mask)
        
        # Validate the resulting value before applying
        err = self._validate_write_bank_and_value(reg_nbr, new_val)
        if err is not None:
            return err
        
        self._reply_word(reg_nbr)
        self._reply_word(and_mask)
        self._reply_word(or_mask)
        
        self.hw.dig_out_bank(reg_nbr, new_val)
        return MB_SUCCESS
    
    def _reg_rd_wr(self):
        """Read/write multiple registers (function 0x17)"""
        # Validate minimum frame length: 11 bytes (addr+func+readStart+readQty+writeStart+writeQty+byteCount)
        if self.cmd_len < 11:
            return MB_BADDATA
        
        # Write registers first
        write_reg_nbr = self._cmd_word(6)
        write_reg_cnt = self._cmd_word(8)
        byte_cnt = self.cmd_buffer[10]
        
        # Validate byteCount: must equal 2 * write_reg_cnt (each register is 2 bytes)
        expected_byte_cnt = 2 * write_reg_cnt
        if byte_cnt != expected_byte_cnt:
            return MB_BADDATA
        
        # Validate total frame length: 11 + byteCount
        if self.cmd_len < 11 + byte_cnt:
            return MB_BADDATA
        
        # Collect all write values first for validation
        data_idx = 11
        write_vals = []
        for _ in range(write_reg_cnt):
            if data_idx + 1 < self.cmd_len:
                reg_val = (self.cmd_buffer[data_idx] << 8) | self.cmd_buffer[data_idx + 1]
                write_vals.append((write_reg_nbr, reg_val))
                write_reg_nbr += 1
                data_idx += 2
            else:
                return MB_BADDATA
        
        # Validate all writes before applying
        for bank, val in write_vals:
            err = self._validate_write_bank_and_value(bank, val)
            if err is not None:
                return err
        
        # Apply all writes after validation passes
        for bank, val in write_vals:
            self.hw.dig_out_bank(bank, val)
        
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
        self.cmd_len = len(cmd_data)
        self.cmd_buffer[:self.cmd_len] = cmd_data
        
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
        
        # Frame detection parameters (matching picoMODBUStest.py)
        # At 1843200 baud: 3.5 character times ≈ 19 µs, use 120 µs to be safe
        self.INTER_FRAME_US = 30
        
        # Maximum buffer sizes to prevent memory exhaustion
        self.MAX_RX_BUF_SIZE = 4096  # Maximum RX buffer size (bytes)
        self.MAX_FRAME_QUEUE_SIZE = 100  # Maximum frames in queue
        self.TX_DELAY_US = 15  # Delay after enabling transceiver (us)
        self.MAX_ACCUM_US = 8000  # Maximum accumulated time (us)
        
        # Frame splitting for receiving concatenated frames
        self._frame_splitter = ModbusFrameSplitter()
        self._rx_buf = bytearray(self.MAX_RX_BUF_SIZE)  # Preallocated buffer
        self._rx_buf_len = 0  # Actual length of data in buffer
        self._frame_queue = []  # Queue for multiple frames received in one chunk
        self._collecting = False
        self._last_rx_us = 0
        
        # Decoder for formatted output
        self._decoder = DCMDecoder()
    
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
                data_len = len(data)
                # Prevent RX buffer from growing too large
                if self._rx_buf_len + data_len > self.MAX_RX_BUF_SIZE:
                    # Buffer overflow - reset buffer to prevent memory exhaustion
                    if DEBUG_ENV:
                        print(f"[RX] Buffer overflow: {self._rx_buf_len} bytes, resetting")
                    self._rx_buf_len = 0
                    self._collecting = False
                else:
                    # Copy data into preallocated buffer
                    self._rx_buf[self._rx_buf_len:self._rx_buf_len + data_len] = data
                    self._rx_buf_len += data_len
                    self._collecting = True
                    self._last_rx_us = now
        
        if self._collecting:
            idle = time.ticks_diff(now, self._last_rx_us)
            if idle >= self.INTER_FRAME_US:
                # Frame group complete - split into individual frames
                chunk = bytes(self._rx_buf[:self._rx_buf_len])
                self._rx_buf_len = 0
                self._collecting = False
                
                frames, leftover = self._frame_splitter.split(chunk)
                
                # Keep leftover (partial frame) for next round
                if leftover:
                    leftover_len = len(leftover)
                    self._rx_buf[0:leftover_len] = leftover
                    self._rx_buf_len = leftover_len
                    self._collecting = True
                    self._last_rx_us = time.ticks_us()
                
                # Queue all valid frames for processing
                if DEBUG_ENV:
                    if not PRINT_MY_ADDRESS_ONLY:
                        print(f"[RX] Split: {len(frames)} frames, {len(leftover)} leftover bytes")
                    else:
                        pass
                
                for raw_msg in frames:
                    payload = raw_msg[:-2]  # Remove CRC
                    if check_crc(raw_msg):
                        # Prevent frame queue from growing too large
                        if len(self._frame_queue) >= self.MAX_FRAME_QUEUE_SIZE:
                            # Queue overflow - drop oldest frame
                            dropped = self._frame_queue.pop(0)
                            if DEBUG_ENV:
                                print(f"[RX] Frame queue overflow: dropped frame")
                        self._frame_queue.append(payload)
                        if DEBUG_ENV:
                            addr = payload[0] if len(payload) > 0 else 0
                            if PRINT_MY_ADDRESS_ONLY and addr != self.hw.mb_address:
                                pass
                            else:
                                decoded = self._decoder.decode(payload, has_crc=False)
                                print(f"[RX] {decoded}")
                    else:
                        if DEBUG_ENV:
                            addr = payload[0] if len(payload) > 0 else 0
                            if PRINT_MY_ADDRESS_ONLY and addr != self.hw.mb_address:
                                pass
                            else:
                                decoded = self._decoder.decode(payload, has_crc=False)
                                rx_crc = raw_msg[-2] | (raw_msg[-1] << 8)
                                calc_crc = crc16_modbus(payload)
                                print(f"[RX] {decoded} [CRC_ERROR: calc=0x{calc_crc:04X} rx=0x{rx_crc:04X}]")
                
        # Return next queued frame if available
        if self._frame_queue:
            frame = self._frame_queue.pop(0)
            # Don't print here - will be printed in tick() when processed
            return frame
        
        return 0  # No complete frame yet
    
    def modbus_serial_tx(self, data):
        if not data:
            return

        crc = crc16_modbus(data)
        tx_data = bytearray(data)
        tx_data.append((crc >> 8) & 0xFF)   # MSB (sent first)
        tx_data.append(crc & 0xFF)          # LSB (sent second)

        # Enable driver
        self.hw.de_pin.value(1)
        time.sleep_us(10)  # allow transceiver to enable

        # Write bytes
        self.hw.modbus_uart.write(tx_data)

        # Conservative hold time: assume 11 bits/byte for 8E1
        bits_per_byte = 11
        tx_time_us = int(len(tx_data) * bits_per_byte * 1_000_000 / MODBUS_BAUD)
        time.sleep_us(tx_time_us + 50)  # extra margin

        self.hw.de_pin.value(0)
    
    def tick(self):
        """Main MODBUS tick function - call periodically"""
            # Receive message
        if True:
            rx_data = self.modbus_serial_rx()
            
            if rx_data == 0:
                return  # No data
            elif rx_data == MB_CRC_ERROR:
                if DEBUG_ENV and PRINT_TICK:
                    if not PRINT_MY_ADDRESS_ONLY:
                        print("[TICK] CRC error - ignoring frame")
                    else:
                        pass
                return  # CRC error, ignore
            
            if len(rx_data) < 2:
                if DEBUG_ENV and PRINT_TICK:
                    if not PRINT_MY_ADDRESS_ONLY:
                        print(f"[TICK] Frame too short: {len(rx_data)} bytes")
                    else:
                        pass
                return
            
            addr = rx_data[0]
            func = rx_data[1]
            
            if DEBUG_ENV:
                if PRINT_MY_ADDRESS_ONLY and addr != self.hw.mb_address:
                    pass
                else:
                    decoded = self._decoder.decode(rx_data, has_crc=False)
                    if PRINT_TICK:
                        print(f"[TICK] Processing: {decoded}")
            
            # Check if message is for this device
            if addr == self.hw.mb_address:
                # Execute command
                reply = self.modbus.execute(rx_data)
                if reply:
                    if DEBUG_ENV and PRINT_TICK:
                        print(f"[TICK] Command executed, sending reply")
                    self.modbus_serial_tx(reply)
                else:
                    if DEBUG_ENV and PRINT_TICK:
                        print(f"[TICK] Command executed, no reply")
            elif addr == 0 and func == 0x45:  # Event trigger (0x45 = 69 decimal = 'E')
                if DEBUG_ENV and PRINT_TICK:
                    decoded = self._decoder.decode(rx_data, has_crc=False)
                    print(f"[TICK] Event trigger: {decoded}")
                self.hw.event_trigger()
            elif addr == 0 and func in [0x44, 0x43]:  # Broadcast commands (0x44=68='D', 0x43=67='C')
                if DEBUG_ENV and PRINT_TICK:
                    decoded = self._decoder.decode(rx_data, has_crc=False)
                    print(f"[TICK] Broadcast command: {decoded}")
                # Execute broadcast command but don't send reply
                self.modbus.execute(rx_data)
            else:
                if DEBUG_ENV and PRINT_TICK:
                    decoded = self._decoder.decode(rx_data, has_crc=False)
                    print(f"[TICK] Address mismatch: {decoded}")


def main():
    """Main program loop"""
    # Initialize DCM
    try:
        dcm = DCM()
    except Exception as e:
        print(f"Failed to initialize DCM: {e}")
        import sys
        sys.print_exception(e)
        return
    
    # Main loop with exception handling
    while True:
        dcm.tick()

        #time.sleep_ms(1)  # Small delay to prevent tight loop


if __name__ == "__main__":
    main()