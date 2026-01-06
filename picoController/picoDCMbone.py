"""
picoDCMbone.py - Test module for DCM hardware
Tests hardware functionality without MODBUS communication:
- Reads MODBUS address from address pins
- Writes random attenuation values
- Reads ADC values (LN0, LN1)
"""

from machine import Pin, ADC
import time
import random

# Pin definitions based on pin mapping
# Address pins (AD0-AD4)
AD0_PIN = 3   # GP3
AD1_PIN = 4   # GP4
AD2_PIN = 5   # GP5
AD3_PIN = 6   # GP6
AD4_PIN = 7   # GP7

# Attenuator output pins (PA0-PA7, Port A equivalent)
PA_PINS = list(range(10, 18))  # GP10-GP17

# ADC input pins
LN0_PIN = 26  # GP26
LN1_PIN = 27  # GP27
ADC_A2_PIN = 28  # GP28


def read_modbus_address():
    """
    Read MODBUS address from address pins (AD0-AD4)
    
    Address mapping (from original code):
    - AD0 (GP3) = PB5 (bit 5)
    - AD1 (GP4) = PB1 (bit 1)
    - AD2 (GP5) = PB2 (bit 2)
    - AD3 (GP6) = PB3 (bit 3)
    - AD4 (GP7) = PB4 (bit 4)
    
    Formula: MB_address = ((Port_B & 0x1E) | ((Port_B & 0x20)>>5))
    where Port_B bits: PB5=AD0, PB1=AD1, PB2=AD2, PB3=AD3, PB4=AD4
    """
    # Set up address pins as inputs with pull-down
    ad0 = Pin(AD0_PIN, Pin.IN, Pin.PULL_DOWN)
    ad1 = Pin(AD1_PIN, Pin.IN, Pin.PULL_DOWN)
    ad2 = Pin(AD2_PIN, Pin.IN, Pin.PULL_DOWN)
    ad3 = Pin(AD3_PIN, Pin.IN, Pin.PULL_DOWN)
    ad4 = Pin(AD4_PIN, Pin.IN, Pin.PULL_DOWN)
    
    # Read pin values and construct Port_B equivalent
    # Port_B mapping: PB5=AD0, PB1=AD1, PB2=AD2, PB3=AD3, PB4=AD4
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
    
    # Calculate address using original formula
    # MB_address = ((Port_B & 0x1E) | ((Port_B & 0x20)>>5))
    mb_address = ((port_b & 0x1E) | ((port_b & 0x20) >> 5))
    
    return mb_address


def setup_attenuator_pins():
    """Set up attenuator output pins (PA0-PA7) as outputs"""
    atten_pins = []
    for pin_num in PA_PINS:
        pin = Pin(pin_num, Pin.OUT)
        atten_pins.append(pin)
    return atten_pins


def write_attenuator(atten_pins, value):
    """
    Write 8-bit attenuation value to PA pins
    
    Args:
        atten_pins: List of Pin objects for PA0-PA7
        value: 8-bit value (0-255) to write
    """
    for i in range(8):
        if i < len(atten_pins):
            # Write bit i to pin (bit 0 = PA0, bit 7 = PA7)
            atten_pins[i].value((value >> i) & 1)


def setup_adc():
    """Set up ADC inputs for LN0, LN1, and ADC_A2"""
    adc_ln0 = ADC(Pin(LN0_PIN))
    adc_ln1 = ADC(Pin(LN1_PIN))
    adc_a2 = ADC(Pin(ADC_A2_PIN))
    return adc_ln0, adc_ln1, adc_a2


def read_adc_values(adc_ln0, adc_ln1, adc_a2):
    """
    Read ADC values and convert to voltage
    
    Args:
        adc_ln0: ADC object for LN0
        adc_ln1: ADC object for LN1
        adc_a2: ADC object for ADC_A2
    
    Returns:
        Tuple of (ln0_raw, ln0_voltage, ln1_raw, ln1_voltage, a2_raw, a2_voltage)
    """
    # Read raw ADC values (0-65535 for 16-bit ADC on Pico)
    ln0_raw = adc_ln0.read_u16()
    ln1_raw = adc_ln1.read_u16()
    a2_raw = adc_a2.read_u16()
    
    # Convert to voltage (ADC is 12-bit on Pico, reference voltage is 3.3V)
    # Actually, read_u16() returns 0-65535, but ADC is 12-bit (0-4095)
    # So we need to scale properly
    # For 12-bit ADC: voltage = (raw_value / 4095) * 3.3
    # For 16-bit read: voltage = (raw_value / 65535) * 3.3
    ln0_voltage = (ln0_raw / 65535.0) * 3.3
    ln1_voltage = (ln1_raw / 65535.0) * 3.3
    a2_voltage = (a2_raw / 65535.0) * 3.3
    
    return ln0_raw, ln0_voltage, ln1_raw, ln1_voltage, a2_raw, a2_voltage


def main():
    """Main test loop"""
    print("=== DCM Hardware Test Module ===")
    print()
    
    # Initialize and read MODBUS address
    print("Reading MODBUS address from pins...")
    mb_address = read_modbus_address()
    print(f"MODBUS Address: {mb_address}")
    print(f"  AD0 (GP3): {Pin(AD0_PIN, Pin.IN, Pin.PULL_DOWN).value()}")
    print(f"  AD1 (GP4): {Pin(AD1_PIN, Pin.IN, Pin.PULL_DOWN).value()}")
    print(f"  AD2 (GP5): {Pin(AD2_PIN, Pin.IN, Pin.PULL_DOWN).value()}")
    print(f"  AD3 (GP6): {Pin(AD3_PIN, Pin.IN, Pin.PULL_DOWN).value()}")
    print(f"  AD4 (GP7): {Pin(AD4_PIN, Pin.IN, Pin.PULL_DOWN).value()}")
    print()
    
    # Set up attenuator pins
    print("Setting up attenuator output pins (PA0-PA7)...")
    atten_pins = setup_attenuator_pins()
    print(f"Configured {len(atten_pins)} attenuator pins: GP{PA_PINS[0]}-GP{PA_PINS[-1]}")
    print()
    
    # Set up ADC inputs
    print("Setting up ADC inputs...")
    adc_ln0, adc_ln1, adc_a2 = setup_adc()
    print(f"LN0: GP{LN0_PIN}, LN1: GP{LN1_PIN}, ADC_A2: GP{ADC_A2_PIN}")
    print()
    
    print("Starting test loop (1 second intervals)...")
    print("Press Ctrl+C to stop")
    print("-" * 60)
    
    loop_count = 0
    
    try:
        while True:
            loop_count += 1
            
            # Generate random attenuation value (0-255)
            atten_value = random.randint(0, 255)
            
            # Write attenuation value
            write_attenuator(atten_pins, atten_value)
            
            # Read ADC values
            ln0_raw, ln0_voltage, ln1_raw, ln1_voltage, a2_raw, a2_voltage = read_adc_values(
                adc_ln0, adc_ln1, adc_a2
            )
            
            # Print status
            print(f"Loop #{loop_count}:")
            print(f"  Attenuator value: 0x{atten_value:02X} ({atten_value:3d}) = ", end="")
            print(f"PA7-PA0: ", end="")
            for i in range(7, -1, -1):
                bit_val = (atten_value >> i) & 1
                print(f"{bit_val}", end="")
            print()
            
            print(f"  LN0 (GP{LN0_PIN}): raw={ln0_raw:5d}, voltage={ln0_voltage:.3f}V")
            print(f"  LN1 (GP{LN1_PIN}): raw={ln1_raw:5d}, voltage={ln1_voltage:.3f}V")
            print(f"  ADC_A2 (GP{ADC_A2_PIN}): raw={a2_raw:5d}, voltage={a2_voltage:.3f}V")
            print()
            
            # Wait 1 second
            time.sleep(1)
    
    except KeyboardInterrupt:
        print()
        print("Test stopped by user")
        print(f"Total loops: {loop_count}")


if __name__ == "__main__":
    main()
