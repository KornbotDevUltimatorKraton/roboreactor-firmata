import time
import sys
import os

# Allow running directly from the examples directory without installing the package
#sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from roboreactor_firmata import STM32Board

# Configuration
PORT = '/dev/ttyUSB0'

# Direct physical pin names on STM32F401RCT6
ANALOG_PIN = 'PA0'  # Potentiometer / Sensor input (A0 channel)
SERVO_PIN = 'PB9'   # Servo motor signal pin
LED_PIN = 'PB6'     # Blink LED pin

def main():
    print("Connecting to STM32 Board using roboreactor-firmata...")
    try:
        # Connect to board using our custom class
        board = STM32Board(PORT, mcu='STM32F401RCT6')
        print(f"Connected successfully to STM32 on port: {PORT}")
    except Exception as e:
        print(f"Error connecting: {e}")
        sys.exit(1)

    # 1. Configure the pins dynamically using physical pin names!
    analog_in = board.get_gpio(ANALOG_PIN, mode='a')  # Mode 'a' = Analog Input
    servo_out = board.get_gpio(SERVO_PIN, mode='s')   # Mode 's' = Servo Output
    led_out = board.get_gpio(LED_PIN, mode='o')       # Mode 'o' = Digital Output

    latest_analog_value = 0.0

    # 2. Callback function for background analog reads
    def analog_callback(value):
        nonlocal latest_analog_value
        latest_analog_value = value

    # Register callback and start sampling thread (50ms interval)
    analog_in.register_callback(analog_callback)
    analog_in.enable_reporting()
    board.samplingOn(50)

    print("\n--- Starting Simultaneous Analog Read, Servo Sweep & LED Blink ---")
    print("Press Ctrl+C to stop.\n")

    last_led_toggle = time.time()
    led_state = 0
    servo_angle = 0
    sweep_direction = 5

    try:
        while True:
            # A. Update Servo Sweep position (0 to 180 degrees)
            servo_out.write(servo_angle)
            servo_angle += sweep_direction
            if servo_angle >= 180 or servo_angle <= 0:
                sweep_direction *= -1
            
            # B. Toggle LED state every 500ms
            current_time = time.time()
            if current_time - last_led_toggle >= 0.5:
                led_state = 1 - led_state
                led_out.write(led_state)
                last_led_toggle = current_time
            
            # C. Display status on a single line
            print(f"Analog [{ANALOG_PIN}]: {latest_analog_value:.3f} | Servo [{SERVO_PIN}]: {servo_angle:3d}° | LED [{LED_PIN}]: {'ON' if led_state else 'OFF'}", end="\r")
            
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\nStopping control loop...")
    finally:
        print("Closing board connection...")
        board.exit()
        print("Done.")

if __name__ == "__main__":
    main()
