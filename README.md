# picoDCM

MicroPython implementation of EOVSA Downconverter Module (DCM) controller for Raspberry Pi Pico, migrated from Rabbit Single Board Computer C code.

## Deployment

To deploy `picoDCM.py` to a Raspberry Pi Pico:

```bash
mpremote connect auto fs cp picoDCM.py :main.py
```

This copies `picoDCM.py` to the Pico as `main.py`, which will run automatically on boot.

## Module Overview

This module implements a MODBUS RTU slave for the EOVSA Downconverter Module (DCM) with support for:
- MODBUS RTU over RS-485 (1843200 baud, 8E1)
- Digital I/O control
- Analog inputs (ADC)
- Attenuator control (normal and table modes)
- Event-driven table selection
- Frame splitting for handling concatenated MODBUS frames

Issue: https://github.com/ovro-eovsa/eovsa15-issues/issues/51

## Architecture

The module consists of four main classes:

### `DCMHardware`
Hardware abstraction layer that manages:
- MODBUS UART communication (RS-485)
- Address pins (AD0-AD4) for MODBUS address configuration
- Attenuator control pins (PA0-PA7)
- Digital I/O pins
- ADC inputs (LN0, LN1, A2)
- Attenuator state machine (normal mode, table mode, event trigger)

### `ModbusSlave`
MODBUS RTU slave implementation that handles:
- Command parsing and validation
- Register/coil read/write operations
- Error handling and exception responses
- Attenuator bank validation (matches Rabbit C semantics)

### `ModbusFrameSplitter`
Utility class for splitting concatenated MODBUS frames received in a single UART read operation.

### `DCM`
Main controller class that:
- Coordinates hardware and MODBUS communication
- Implements the main `tick()` function for periodic processing
- Manages frame reception queue
- Handles broadcast commands and event triggers


## MODBUS Functions Supported

The implementation supports the following MODBUS RTU functions:

| Function Code | Name | Description |
|--------------|------|-------------|
| 0x01 | Read Coil Status | Read digital outputs (coils) |
| 0x02 | Read Input Status | Read digital inputs |
| 0x03 | Read Holding Registers | Read output registers (attenuator, etc.) |
| 0x04 | Read Input Registers | Read input registers (ADC, digital inputs) |
| 0x05 | Write Single Coil | Write a single digital output |
| 0x06 | Write Single Register | Write a single register (e.g., attenuator set value) |
| 0x0F | Write Multiple Coils | Write multiple digital outputs |
| 0x10 | Write Multiple Registers | Write multiple registers (e.g., attenuator table) |
| 0x16 | Mask Write Register | Mask write to a register |
| 0x17 | Read/Write Multiple Registers | Read and write registers in one transaction |
| 0x43 | Load Attenuator Set Value | Alias for 0x06 (write single register) |
| 0x44 | Load Attenuator Table | Alias for 0x10 (write multiple registers) |
| 0x45 | Event Trigger | Broadcast command to trigger table selection (address 0) |

## Register Mapping

### Output Registers (Holding Registers, 0x03)

- **Bank 0**: Attenuator set value (nibble-only: 0x00-0x0F per byte)
- **Bank 1**: Attenuator mode (0 = normal, 1 = table)
- **Bank 2**: Attenuator table index (0-49)
- **Bank 3-52**: Attenuator table entries (nibble-only: 0x00-0x0F per byte)

### Input Registers (0x04)

- **Bank 0-255**: Digital inputs, ADC readings, and status registers

### Validation Rules

The implementation matches Rabbit C code validation semantics:
- Register addresses must be in range 0-52 for writes
- Banks 0 and 3-52 require nibble-only data (each byte ≤ 0x0F)
- Bank 2 (table index) should be in range 0-49

## Configuration

### Debug Settings

Edit these constants at the top of `picoDCM.py`:

```python
DEBUG_ENV = False  # Set to True to suppress diagnostics, False to print them
PRINT_MY_ADDRESS_ONLY = True  # Print only messages for this device's address
PRINT_TICK = False  # Print tick processing messages
```

### Frame Detection Parameters

```python
INTER_FRAME_US = 30  # Inter-frame timeout (microseconds)
MAX_RX_BUF_SIZE = 4096  # Maximum RX buffer size (bytes)
MAX_FRAME_QUEUE_SIZE = 100  # Maximum frames in queue
TX_DELAY_US = 15  # Delay after enabling transceiver (microseconds)
MAX_ACCUM_US = 8000  # Maximum accumulated time (microseconds)
```

## Pin Mapping

```
GP0  - TX_E (MODBUS UART TX)
GP1  - RX_E (MODBUS UART RX)
GP2  - DRV_EN (RS-485 DE/RE control)
GP3  - AD0 (Address bit 0)
GP4  - AD1 (Address bit 1)
GP5  - AD2 (Address bit 2)
GP6  - AD3 (Address bit 3)
GP7  - AD4 (Address bit 4)
GP8  - TX_D
GP9  - RX_D
GP10 - PA0 (Attenuator bit 0)
GP11 - PA1 (Attenuator bit 1)
GP12 - PA2 (Attenuator bit 2)
GP13 - PA3 (Attenuator bit 3)
GP14 - PA4 (Attenuator bit 4)
GP15 - PA5 (Attenuator bit 5)
GP16 - PA6 (Attenuator bit 6)
GP17 - PA7 (Attenuator bit 7)
GP18 - RX_C
GP19 - TX_C
GP20 - GP20
GP21 - GP21
GP22 - GP22
GP26 - LN0 (ADC input)
GP27 - LN1 (ADC input)
GP28 - ADC_A2 (ADC input)
```

## MODBUS Communication

- **Baud Rate**: 1843200
- **Parity**: Even (8E1: 8 bits, Even parity, 1 stop bit)
- **CRC**: MODBUS CRC16 using lookup tables (matches Rabbit C implementation)
- **Address**: Configured via AD0-AD4 pins (0-31)
- **RS-485**: Half-duplex using DE/RE control pin

## Performance Optimizations

- Preallocated RX buffer to avoid repeated allocations
- Lookup table-based CRC calculation
- Frame queue for handling multiple frames in one chunk
- Efficient frame splitting algorithm

## Files

- `picoDCM.py` - Main DCM module with MODBUS RTU slave
- `picoDCMdecoder.py` - MODBUS frame decoder for debugging
- `picoController/picoDCMbone.py` - Test module for hardware verification (no MODBUS)

## License

GPT-4.1
