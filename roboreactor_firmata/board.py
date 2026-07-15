import time
from pyfirmata2 import Board

# Standard Firmata Commands
I2C_REQUEST = 0x76
I2C_REPLY = 0x77
I2C_CONFIG = 0x78

# Custom Dynamic Peripherals Commands
CUSTOM_I2C_CONFIG   = 0x10
CUSTOM_SPI_CONFIG   = 0x11
CUSTOM_UART_CONFIG  = 0x12
CUSTOM_SPI_TRANSFER = 0x13
CUSTOM_CAN_CONFIG   = 0x14
CUSTOM_CAN_WRITE    = 0x15
CUSTOM_CAN_READ     = 0x16
CUSTOM_UART_WRITE   = 0x17

class RoboReactorBoard(Board):
    """
    Base Board class for RoboReactor microcontrollers.
    Inherits from pyfirmata2.Board and adds support for dynamic I2C, SPI, UART, and CAN-bus.
    """
    def __init__(self, port, layout, baudrate=57600, *args, **kwargs):
        self.i2c_callbacks = {}
        self.spi_callback = None
        self.uart_callback = None
        self.can_callback = None
        
        # Initialize base Board
        super().__init__(port, layout, baudrate, *args, **kwargs)
        
        # Register standard command handlers
        self.add_cmd_handler(I2C_REPLY, self._handle_i2c_reply)
        
        # Register custom command handlers
        self.add_cmd_handler(CUSTOM_SPI_TRANSFER, self._handle_spi_reply)
        self.add_cmd_handler(CUSTOM_UART_CONFIG, self._handle_uart_incoming)
        self.add_cmd_handler(CUSTOM_CAN_READ, self._handle_can_incoming)

    def get_pin_index(self, pin_name):
        """Must be implemented by subclasses to map physical names to pin indices."""
        raise NotImplementedError("get_pin_index() must be implemented by subclasses.")

    # --- Convenience Write/Read Methods ---
    def digital_write(self, pin_name, value):
        """Write HIGH (1) or LOW (0) to a digital pin by name."""
        pin_idx = self.get_pin_index(pin_name)
        self.digital[pin_idx].mode = 1  # OUTPUT
        self.digital[pin_idx].write(value)

    def digital_read(self, pin_name):
        """Read state from a digital pin by name."""
        pin_idx = self.get_pin_index(pin_name)
        self.digital[pin_idx].mode = 0  # INPUT
        return self.digital[pin_idx].read()

    def pwm_write(self, pin_name, duty_cycle):
        """Write PWM value (0.0 to 1.0) to a pin by name."""
        pin_idx = self.get_pin_index(pin_name)
        self.digital[pin_idx].mode = 3  # PWM
        self.digital[pin_idx].write(duty_cycle)

    def servo_write(self, pin_name, angle):
        """Write servo angle (0 to 180 degrees) to a pin by name."""
        pin_idx = self.get_pin_index(pin_name)
        self.digital[pin_idx].mode = 4  # SERVO
        self.digital[pin_idx].write(angle)

    # --- Dynamic I2C Config (SDA / SCL redirection) ---
    def configure_custom_i2c(self, sda_pin, scl_pin):
        """Redirect and configure dynamic SDA and SCL pins for the Wire bus."""
        sda_idx = self.get_pin_index(sda_pin)
        scl_idx = self.get_pin_index(scl_pin)
        self.send_sysex(CUSTOM_I2C_CONFIG, [sda_idx, scl_idx])

    # --- Standard I2C Engine ---
    def i2c_config(self, delay_us=0):
        """Configure I2C bus delay (in microseconds)."""
        data = self._encode_8bit_to_7bit([delay_us & 0xFF, (delay_us >> 8) & 0xFF])
        self.send_sysex(I2C_CONFIG, data)

    def i2c_write(self, address, register, data_bytes):
        """Write data bytes to an I2C slave register."""
        mode = 0  # Write Mode
        payload = [register] + list(data_bytes)
        data = [address & 0x7F, mode & 0x7F]
        data.extend(self._encode_8bit_to_7bit(payload))
        self.send_sysex(I2C_REQUEST, data)

    def i2c_read(self, address, register, num_bytes, callback=None):
        """Read data bytes from an I2C slave register."""
        if callback:
            self.i2c_callbacks[(address, register)] = callback
        mode = 1  # Read Once Mode
        payload = [register, num_bytes]
        data = [address & 0x7F, mode & 0x7F]
        data.extend(self._encode_8bit_to_7bit(payload))
        self.send_sysex(I2C_REQUEST, data)

    def _handle_i2c_reply(self, *data):
        if len(data) < 4:
            return
        address = data[0]
        register = data[1] | (data[2] << 7)
        raw_payload = data[3:]
        decoded_bytes = self._decode_7bit_pairs(raw_payload)
        callback = self.i2c_callbacks.get((address, register))
        if callback:
            callback(decoded_bytes)

    # --- Dynamic SPI Config & Transfer ---
    def configure_custom_spi(self, mosi, miso, sclk, cs):
        """Configure dynamic MOSI, MISO, SCLK, and CS pins for SPI."""
        mosi_idx = self.get_pin_index(mosi)
        miso_idx = self.get_pin_index(miso)
        sclk_idx = self.get_pin_index(sclk)
        cs_idx = self.get_pin_index(cs)
        self.send_sysex(CUSTOM_SPI_CONFIG, [mosi_idx, miso_idx, sclk_idx, cs_idx])

    def spi_transfer(self, cs_pin, data_bytes, callback=None):
        """Perform full-duplex SPI read/write transfer and register callback for reply."""
        if callback:
            self.spi_callback = callback
        cs_idx = self.get_pin_index(cs_pin)
        payload = [cs_idx] + self._encode_8bit_to_7bit(data_bytes)
        self.send_sysex(CUSTOM_SPI_TRANSFER, payload)

    def _handle_spi_reply(self, *data):
        decoded = self._decode_7bit_pairs(data)
        if self.spi_callback:
            self.spi_callback(decoded)

    # --- Dynamic UART Config & Operations ---
    def configure_custom_uart(self, rx_pin, tx_pin, baudrate, callback=None):
        """Configure dynamic RX/TX pins for a UART serial port and register read callback."""
        if callback:
            self.uart_callback = callback
        rx_idx = self.get_pin_index(rx_pin)
        tx_idx = self.get_pin_index(tx_pin)
        baud_bytes = [
            baudrate & 0x7F,
            (baudrate >> 7) & 0x7F,
            (baudrate >> 14) & 0x7F
        ]
        self.send_sysex(CUSTOM_UART_CONFIG, [rx_idx, tx_idx] + baud_bytes)

    def uart_write(self, data_bytes):
        """Write raw bytes to the dynamic UART serial port."""
        payload = self._encode_8bit_to_7bit(data_bytes)
        self.send_sysex(CUSTOM_UART_WRITE, payload)

    def _handle_uart_incoming(self, *data):
        decoded = self._decode_7bit_pairs(data)
        if self.uart_callback:
            self.uart_callback(bytes(decoded))

    # --- Dynamic CAN-bus Config & Operations ---
    def configure_custom_can(self, rx_pin, tx_pin, baudrate, callback=None):
        """Configure dynamic RX/TX pins for CAN-bus and register frame receive callback."""
        if callback:
            self.can_callback = callback
        rx_idx = self.get_pin_index(rx_pin)
        tx_idx = self.get_pin_index(tx_pin)
        baud_bytes = [
            baudrate & 0x7F,
            (baudrate >> 7) & 0x7F,
            (baudrate >> 14) & 0x7F,
            (baudrate >> 21) & 0x7F
        ]
        self.send_sysex(CUSTOM_CAN_CONFIG, [rx_idx, tx_idx] + baud_bytes)

    def can_write(self, msg_id, data_bytes, is_extended=0):
        """Write a CAN frame to the CAN bus."""
        id_bytes = [
            msg_id & 0x7F,
            (msg_id >> 7) & 0x7F,
            (msg_id >> 14) & 0x7F,
            (msg_id >> 21) & 0x7F,
            (msg_id >> 28) & 0x7F
        ]
        length = len(data_bytes)
        payload = id_bytes + [is_extended & 0x7F, length & 0x7F]
        payload.extend(self._encode_8bit_to_7bit(data_bytes))
        self.send_sysex(CUSTOM_CAN_WRITE, payload)

    def _handle_can_incoming(self, *data):
        if len(data) < 7:
            return
        msg_id = 0
        for i in range(5):
            msg_id |= (data[i] << (i * 7))
        is_extended = data[5]
        length = data[6]
        raw_payload = data[7:]
        decoded_payload = self._decode_7bit_pairs(raw_payload)
        
        if self.can_callback:
            self.can_callback(msg_id, bytes(decoded_payload[:length]), bool(is_extended))

    # --- Helper methods for encoding / decoding ---
    def _encode_8bit_to_7bit(self, data):
        """Encodes standard 8-bit bytes into 7-bit pairs for Firmata Sysex."""
        encoded = []
        for val in data:
            encoded.append(val & 0x7F)
            encoded.append((val >> 7) & 0x7F)
        return encoded

    def _decode_7bit_pairs(self, data):
        """Decodes 7-bit pairs from Firmata Sysex back into 8-bit bytes."""
        decoded = []
        for i in range(0, len(data), 2):
            if i + 1 < len(data):
                val = data[i] | (data[i+1] << 7)
                decoded.append(val)
        return decoded
