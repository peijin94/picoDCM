from machine import UART, Pin
import time

BAUD = 1843200
uart = UART(0, BAUD, tx=Pin(0), rx=Pin(1), bits=8, parity=None, stop=1)

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF

QUIET_US = 3000      # increase if frames still chopped
MAXLEN   = 256

buf = bytearray()
last_rx = time.ticks_us()

while True:
    n = uart.any()
    if n:
        chunk = uart.read(n)
        if chunk:
            buf.extend(chunk)
            last_rx = time.ticks_us()
            if len(buf) > MAXLEN:
                buf[:] = b''     # reset safely
    else:
        if buf and time.ticks_diff(time.ticks_us(), last_rx) > QUIET_US:
            frame = bytes(buf)
            buf[:] = b''         # reset safely

            if len(frame) >= 4:
                payload = frame[:-2]
                rx_crc = frame[-2] | (frame[-1] << 8)  # LSB-first
                ok = (crc16(payload) == rx_crc)

                print("FRAME len", len(frame), "CRC", "OK" if ok else "BAD")
                print("  raw:", frame.hex())
                if len(payload) >= 2:
                    print("  addr=", payload[0], "func=0x%02X" % payload[1])