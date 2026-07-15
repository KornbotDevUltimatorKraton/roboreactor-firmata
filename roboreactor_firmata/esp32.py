from .board import RoboReactorBoard

class ESP32Board(RoboReactorBoard):
    """
    Board implementation specifically for ESP32 microcontrollers.
    Pins are configured directly by their GPIO index (e.g., GPIO12 or 12).
    """
    def __init__(self, port, baudrate=115200, *args, **kwargs):
        # ESP32 has up to 40 GPIO pins (0-39)
        # Note: Baudrate defaults to 115200 for ESP32 standard uploads
        num_digital_pins = 40
        num_analog_pins = 16  # standard 16 analog channels max (ADC1/ADC2)
        pwm_pins = tuple(range(num_digital_pins))
        
        layout = {
            'digital': tuple(range(num_digital_pins)),
            'analog': tuple(range(num_analog_pins)),
            'pwm': pwm_pins,
            'use_ports': True,
            'disabled': (1, 3) # TX0 (1) / RX0 (3) are default disabled
        }
        
        super().__init__(port, layout, baudrate, *args, **kwargs)

    def get_pin_index(self, pin_name):
        """Translate a pin name (e.g., 'GPIO12', 'IO12', or '12') to its numeric index."""
        if isinstance(pin_name, int):
            return pin_name
        pin_name = str(pin_name).upper().strip()
        if pin_name.startswith('GPIO'):
            try:
                return int(pin_name[4:])
            except ValueError:
                pass
        elif pin_name.startswith('IO'):
            try:
                return int(pin_name[2:])
            except ValueError:
                pass
        try:
            return int(pin_name)
        except ValueError:
            raise ValueError(f"Invalid ESP32 pin name: {pin_name}. Use format like 'GPIO12', 'IO12', or integer 12.")

    def get_gpio(self, pin_name, mode):
        """
        Configure and return a pin object using the physical pin name.
        """
        pin_idx = self.get_pin_index(pin_name)
        
        if mode == 'a':
            # Analog channel mapping for ESP32
            return self.get_pin(f'a:{pin_idx}:i')
        else:
            return self.get_pin(f'd:{pin_idx}:{mode}')
