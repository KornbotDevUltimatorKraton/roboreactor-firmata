from .board import RoboReactorBoard

# Pin mappings for different STM32 microcontrollers in the STM32duino Core
PIN_MAPS = {
    # STM32F401RCT6 (64-pin package)
    'STM32F401RCT6': {
        **{f'PA{i}': i for i in range(16)},
        **{f'PB{i}': i + 16 for i in range(16)},
        **{f'PC{i}': i + 32 for i in range(16)},
        'PD2': 48,
        'PH0': 49,
        'PH1': 50,
    },
    # STM32F401CCU6 (48-pin package / Blackpill)
    'STM32F401CCU6': {
        **{f'PA{i}': i for i in range(16)},
        **{f'PB{i}': i + 16 for i in range(16)},
        'PC13': 32,
        'PC14': 33,
        'PC15': 34,
        'PH0': 35,
        'PH1': 36,
    },
    # STM32F103C8T6 (48-pin package / Bluepill)
    'STM32F103C8T6': {
        **{f'PA{i}': i for i in range(16)},
        **{f'PB{i}': i + 16 for i in range(16)},
        'PC13': 32,
        'PC14': 33,
        'PC15': 34,
        'PD0': 35,
        'PD1': 36,
    }
}

class STM32Board(RoboReactorBoard):
    """
    Board implementation specifically for STM32 microcontrollers.
    """
    def __init__(self, port, mcu='STM32F401RCT6', baudrate=57600, *args, **kwargs):
        self.mcu = mcu.upper()
        if self.mcu not in PIN_MAPS:
            raise ValueError(f"Unknown MCU type: {mcu}. Available options: {list(PIN_MAPS.keys())}")
        
        self.pin_map = PIN_MAPS[self.mcu]
        
        # Dynamically build the layout based on the number of pins on this MCU
        num_digital_pins = max(self.pin_map.values()) + 1
        num_analog_pins = 16  # Standard 16 analog channels max
        pwm_pins = tuple(self.pin_map.values())
        
        layout = {
            'digital': tuple(range(num_digital_pins)),
            'analog': tuple(range(num_analog_pins)),
            'pwm': pwm_pins,
            'use_ports': True,
            'disabled': (0, 1) # Default disabled serial pins (e.g. RX/TX)
        }
        
        super().__init__(port, layout, baudrate, *args, **kwargs)

    def get_pin_index(self, pin_name):
        """Translate a physical pin name (e.g., 'PA0') to its numeric index."""
        pin_name = pin_name.upper().strip()
        if pin_name not in self.pin_map:
            raise ValueError(f"Pin {pin_name} is not available or exposed on {self.mcu}")
        return self.pin_map[pin_name]

    def get_gpio(self, pin_name, mode):
        """
        Configure and return a pin object using the physical pin name.
        
        Modes:
            'i' - Input
            'o' - Output
            'a' - Analog Input
            'p' - PWM Output
            's' - Servo Output
        """
        pin_idx = self.get_pin_index(pin_name)
        
        if mode == 'a':
            if pin_name.startswith('PA') and int(pin_name[2:]) < 8:
                analog_channel = int(pin_name[2:])
                return self.get_pin(f'a:{analog_channel}:i')
            else:
                raise ValueError(f"Pin {pin_name} does not support analog input mode on {self.mcu}")
        else:
            return self.get_pin(f'd:{pin_idx}:{mode}')
