#!/usr/bin/env python3

import rospy 
import numpy as np
import pyfirmata
import time
import keyboard

rospy.init_node('ol_p')

class Car:
    def __init__(self, port, servo_pin, sensor1_pin, motor_pin, wheel_diameter):
        self.wheel_diameter = wheel_diameter / 100  # centimetre to metre
        self.board = pyfirmata.Arduino(port)
        self.rpm_sensor = self.board.get_pin(f'a:{sensor1_pin}:i')
        self.motor = self.board.get_pin(f'd:{motor_pin}:p')
        self.servo = self.board.get_pin(f'd:{servo_pin}:o')
        self.board.digital[1].write(1)
        self.board.digital[2].write(0)
        self.it = pyfirmata.util.Iterator(self.board)  # create iterator object
        self.it.start()  # start iterator

    def accelerate(self, value):
        # Limit the value to range [0, 100]
        value = max(min(value, 100), 0)
        # Calculate the PWM duty cycle based on the value
        duty_cycle = (value / 100) * 255
        # Apply the PWM duty cycle to the motor pin
        self.motor.write(duty_cycle)

    def steer(self, value):
        # Limit the value to range [-90, 90]
        value = max(min(value, 90), -90)
        # Calculate the angle in degrees based on the value
        angle = (value / 90) * 45 + 90
        # Set the position of the servo motor
        self.servo.write(angle)

    def brake(self):
        self.motor.write(0)

    def calc_rpm(self):
        # Define the variables for measuring the time between pulses and calculating RPM
        time1 = 0
        time2 = 0
        deltaTime = 0
        rpm_ = 0
        threshold = 480

        # Main loop
        while True:
            time1 = time.monotonic_ns()  # record the time when the first pulse is detected
            while self.rpm_sensor.read() > threshold:  # wait for the pulse to end
                pass
            time2 = time.monotonic_ns()  # record the time when the second pulse is detected
            deltaTime = time2 - time1  # calculate the time between pulses
            rpm_ = 60000000000 / deltaTime  # convert the time between pulses to RPM
            yield rpm_  # return the RPM value using a generator

    def get_velocity(self):
        while True:
            rpm_ = next(self.calc_rpm())  # retrieve the next yielded value from the RPM generator
            yield (rpm_ * self.wheel_diameter * np.pi) / 60  #

    def __del__(self):
        self.board.exit()


car = Car('COM9', 4, 0, 9, 6)

steering_angle = 0
acceleration = 0

while True:
    if keyboard.is_pressed('esc'):
        break
    if keyboard.is_pressed('w'):
        acceleration += 1
    if keyboard.is_pressed('s'):
        acceleration -= 1

    # Limit the acceleration to the range [0, 20]
    acceleration = max(min(acceleration, 20), 0)

    # Apply the acceleration
    car.accelerate(acceleration)
    if keyboard.is_pressed('b'):
        acceleration = 0
        car.brake()

    if keyboard.is_pressed('a'):
        steering_angle = max(steering_angle - 5, -45)
    if keyboard.is_pressed('d'):
        steering_angle = min(steering_angle + 5, 45)

    # Set the steering angle
    car.steer(steering_angle)

    print(next(car.calc_rpm()))
    print(next(car.get_velocity()))
    time.sleep(0.02)

del car
