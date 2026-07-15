import time
import sys
import os

# Allow running directly from the examples directory without installing the package
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from roboreactor_firmata import STM32Board

# Device Addresses
TCA_ADDR = 0x70      # TCA9548A Multiplexer
PCA_ADDR = 0x40      # PCA9685 Servo Driver (connected to TCA Channel 2)
MPU_ADDR = 0x68      # MPU6050 Gyroscope/Accelerometer (connected to TCA Channel 5)

# PCA9685 Registers
MODE1_REG = 0x00
PRESCALE_REG = 0xFE
LED0_ON_L = 0x06

# MPU6050 Registers
MPU_PWR_MGMT_1 = 0x6B
MPU_ACCEL_XOUT_H = 0x3B

def select_tca_channel(board, channel):
    """Selects an active I2C channel (0-7) on the TCA9548A multiplexer."""
    if channel < 0 or channel > 7:
        raise ValueError("Channel must be between 0 and 7")
    control_mask = 1 << channel
    # Write the channel select mask to the TCA9548A
    board.i2c_write(TCA_ADDR, register=control_mask, data_bytes=[])
    time.sleep(0.01) # Short delay for hardware state transition

# --- PCA9685 Helper Functions ---
def init_pca9685(board, frequency=50):
    """Initializes PCA9685 frequency (usually 50Hz for standard analog servos)."""
    # Put PCA9685 into Sleep mode to configure prescaler
    board.i2c_write(PCA_ADDR, MODE1_REG, [0x10])
    
    # Calculate prescaler value
    prescale_val = int(round(25000000.0 / (4096.0 * frequency)) - 1)
    board.i2c_write(PCA_ADDR, PRESCALE_REG, [prescale_val])
    
    # Wake up device and enable register auto-increment
    board.i2c_write(PCA_ADDR, MODE1_REG, [0x20])
    time.sleep(0.01)

def set_pca9685_servo(board, channel, pulse_width_ticks):
    """Controls a servo connected to a specific channel of the PCA9685 (0-15)."""
    reg = LED0_ON_L + (channel * 4)
    on_l = 0
    on_h = 0
    off_l = pulse_width_ticks & 0xFF
    off_h = (pulse_width_ticks >> 8) & 0xFF
    board.i2c_write(PCA_ADDR, reg, [on_l, on_h, off_l, off_h])

# --- MPU6050 Helper Functions ---
def init_mpu6050(board):
    """Wakes up the MPU6050 sensor."""
    # Write 0 to PWR_MGMT_1 to wake up the sensor
    board.i2c_write(MPU_ADDR, MPU_PWR_MGMT_1, [0x00])
    time.sleep(0.01)

def mpu6050_callback(data):
    """Asynchronous callback handling raw accelerometer data from the MPU6050."""
    if len(data) < 6:
        return
    # Decode 6 raw bytes into signed 16-bit integers
    ax = (data[0] << 8) | data[1]
    ay = (data[2] << 8) | data[3]
    az = (data[4] << 8) | data[5]
    
    if ax > 32767: ax -= 65536
    if ay > 32767: ay -= 65536
    if az > 32767: az -= 65536
    
    print(f"[MPU6050 Data] Accel X: {ax:6d} | Y: {ay:6d} | Z: {az:6d}")

def main():
    # Configure board on /dev/ttyUSB0
    print("Connecting to STM32 Board...")
    try:
        board = STM32Board("/dev/ttyUSB0", pin_count=32)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

    # Route Wire to PB9 (SDA) and PB8 (SCL) and start the engine
    board.configure_custom_i2c("PB9", "PB8")
    board.i2c_config()
    board.samplingOn(50)
    
    # 1. Initialize PCA9685 on TCA Channel 2
    print("\n[TCA] Selecting Channel 2 to configure PCA9685 Servo Driver...")
    select_tca_channel(board, channel=2)
    init_pca9685(board, frequency=50)
    print("PCA9685 initialized at 50Hz.")
    
    # 2. Initialize MPU6050 on TCA Channel 5
    print("\n[TCA] Selecting Channel 5 to configure MPU6050 Gyro/Accel...")
    select_tca_channel(board, channel=5)
    init_mpu6050(board)
    print("MPU6050 sensor wakes up.")

    print("\n--- Running Control Loop ---")
    try:
        angle_direction = 1
        pulse_width = 300 # Start at servo neutral
        
        while True:
            # Step A: Select Channel 2 and update Servo Position
            select_tca_channel(board, channel=2)
            set_pca9685_servo(board, channel=0, pulse_width_ticks=pulse_width)
            
            # Sweep pulse width between 150 (0 deg) and 500 (180 deg)
            pulse_width += 15 * angle_direction
            if pulse_width >= 500 or pulse_width <= 150:
                angle_direction *= -1
            
            # Step B: Select Channel 5 and trigger a Gyroscope Reading
            select_tca_channel(board, channel=5)
            # Request 6 bytes starting from MPU_ACCEL_XOUT_H (0x3B)
            board.i2c_read(MPU_ADDR, MPU_ACCEL_XOUT_H, 6, callback=mpu6050_callback)
            
            time.sleep(0.2) # Run at 5Hz rate
            
    except KeyboardInterrupt:
        print("\nExiting and cleaning up...")
        board.exit()

if __name__ == "__main__":
    main()
