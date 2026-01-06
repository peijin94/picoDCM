# picoDCM

MicroPython implementation of EOVSA Downconverter Module (DCM) controller for Raspberry Pi Pico, migrated from Rabbit Single Board Computer C code.

## Overview

This module implements MODBUS RTU slave functionality for controlling DCM hardware, including:
- Attenuator control (normal and table modes)
- Digital I/O
- ADC readings
- Serial communication with optical receivers
- RS-485 MODBUS communication

## Files

- `picoDCM.py` - Main DCM module with MODBUS RTU slave
- `picoController/picoDCMbone.py` - Test module for hardware verification (no MODBUS)

## Quick Start

### Hardware Test
```python
from picoController.picoDCMbone import main
main()
```

### MODBUS Controller
```python
from picoDCM import DCM
dcm = DCM()
while True:
    dcm.tick()
```

## Pin Mapping

```
GP0  - TX_E
GP1  - RX_E
GP2  - DRV_EN
GP3  - AD0
GP4  - AD1
GP5  - AD2
GP6  - AD3
GP7  - AD4
GP8  - TX_D
GP9  - RX_D
GP10 - PA0
GP11 - PA1
GP12 - PA2
GP13 - PA3
GP14 - PA4
GP15 - PA5
GP16 - PA6
GP17 - PA7
GP18 - RX_C
GP19 - TX_C
GP20 - GP20
GP21 - GP21
GP22 - GP22
GP26 - LN0
GP27 - LN1
GP28 - ADC_A2
```

## License

GPT-4.1
