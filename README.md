# roboreactor-firmata

A unified, dynamic Firmata client library for configuring and controlling **GPIO**, **I2C**, **SPI**, **UART**, and **CAN-bus** peripherals on **STM32** and **ESP32** microcontrollers.

Instead of being limited to hardcoded hardware pins, `roboreactor-firmata` allows you to dynamically assign peripheral pins at runtime from Python using a custom extended SysEx protocol.

---

## Installation

```bash
pip install roboreactor-firmata
```

---

## Firmware Flashing

Before running the Python client, flash the matching custom `.ino` sketch located inside the `firmware/` directory to your microcontroller:

- **STM32**: Flash `StandardFirmataCustom_STM32.ino` (Requires `STM32duino` core and optional `STM32_CAN` library).
- **ESP32**: Flash `StandardFirmataCustom_ESP32.ino` (Uses ESP32's native Arduino core and built-in `TWAI` CAN driver).

---

## Quick Start Examples

### 1. STM32 Control (e.g., STM32F401RCT6)

```python
import time
from roboreactor_firmata import STM32Board

# Connect to the board
board = STM32Board('/dev/ttyUSB0', mcu='STM32F401RCT6')
board.samplingOn(50)  # Start polling thread

# --- 1. Dynamic I2C Config ---
board.configure_custom_i2c(sda_pin="PB9", scl_pin="PB8")
board.i2c_config()

def i2c_callback(data):
    print(f"I2C Data: {data}")

board.i2c_read(0x68, 0x75, 1, callback=i2c_callback)

# --- 2. Dynamic UART Config ---
def uart_callback(data):
    print(f"UART Rx: {data}")

board.configure_custom_uart(rx_pin="PA3", tx_pin="PA2", baudrate=9600, callback=uart_callback)
board.uart_write(b"Hello STM32")

# --- 3. Dynamic SPI Config ---
def spi_callback(data):
    print(f"SPI Rx: {data}")

board.configure_custom_spi(mosi="PA7", miso="PA6", sclk="PA5", cs="PA4")
board.spi_transfer("PA4", [0x9F, 0x00, 0x00, 0x00], callback=spi_callback)

time.sleep(2)
board.exit()
```

### 2. ESP32 Control

```python
import time
from roboreactor_firmata import ESP32Board

# Connect to the board (ESP32 defaults to 115200 baud)
board = ESP32Board('/dev/ttyUSB0', baudrate=115200)
board.samplingOn(50)

# --- 1. Dynamic I2C Config ---
# Route Wire to GPIO 21 (SDA) and GPIO 22 (SCL)
board.configure_custom_i2c(sda_pin=21, scl_pin=22)
board.i2c_config()

# --- 2. Dynamic CAN-bus Config ---
def can_callback(msg_id, payload, is_extended):
    print(f"CAN Rx - ID: {hex(msg_id)}, Payload: {payload}")

# Route TWAI CAN to GPIO 4 (RX) and GPIO 5 (TX) at 500kbps
board.configure_custom_can(rx_pin=4, tx_pin=5, baudrate=500000, callback=can_callback)

# Transmit CAN frame
board.can_write(msg_id=0x123, data_bytes=[0xAA, 0xBB, 0xCC], is_extended=0)

time.sleep(2)
board.exit()
```

---

## License

This project is licensed under the MIT License - see the `LICENSE` file for details.
