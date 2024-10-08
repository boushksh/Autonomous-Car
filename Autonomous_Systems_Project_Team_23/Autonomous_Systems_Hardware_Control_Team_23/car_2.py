import smbus
import math
import time
import RPi.GPIO as GPIO

# GPIO pin for the servo signal
servo_pin = 4  # MPU6050 Registers

PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT = 0x3B
ACCEL_YOUT = 0x3D
ACCEL_ZOUT = 0x3F
GYRO_XOUT = 0x43
GYRO_YOUT = 0x45
GYRO_ZOUT = 0x47
# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)

# Create PWM object
pwm = GPIO.PWM(servo_pin, 40)  # 50 Hz (adjust frequency as needed)


# Function to set the servo angle
def set_angle(angle):
    duty = angle / 18 + 2  # Map angle to duty cycle (adjust as needed)
    pwm.ChangeDutyCycle(duty)
    # time.sleep(0.3)  # Give the servo time to reach the position


# MPU6050 Class
class MPU6050:
    def __init__(self, address=0x68):
        self.bus = smbus.SMBus(1)
        self.address = address
        self.initialize()
        self.gyro_scale_factor = 131.0
        self.dt = 0.1  # time interval between readings (adjust as needed)
        self.angle_z = 0.0

    def initialize(self):
        # Wake up the MPU6050
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)

    def read_raw_data(self, register):
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)
        value = (high << 8) + low
        return value if value < 32768 else value - 65536

    def get_rotation(self):
        x = self.read_raw_data(GYRO_XOUT) / self.gyro_scale_factor
        y = self.read_raw_data(GYRO_YOUT) / self.gyro_scale_factor
        z = self.read_raw_data(GYRO_ZOUT) / self.gyro_scale_factor
        return (x, y, z)

    def calculate_angle(self):
        rotation = self.get_rotation()
        self.angle_z += rotation[2] * self.dt
        return self.angle_z


# Main function
if __name__ == '__main__':
    try:
        mpu6050 = MPU6050()
        # Start PWM
        pwm.start(0)

        while True:
            set_angle(90)  # 0 degrees

            angle_z = mpu6050.calculate_angle()

            print(f"Rotation angle (z-axis): {angle_z} degrees")

            time.sleep(mpu6050.dt)

    except KeyboardInterrupt:
        pass

