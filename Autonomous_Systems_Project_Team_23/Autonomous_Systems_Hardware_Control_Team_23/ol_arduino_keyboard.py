#!/usr/bin/env python3


import serial

import time

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ser.reset_input_buffer()
pwm = 0

while True:

    value = input("telescope input : ")

    if value == "w":
        ser.write(b"FORWARD\n")

    if value == "d":
        ser.write(b"RIGHT\n")

    if value == "a":
        ser.write(b"LEFT\n")

    if value == "s":
        ser.write(b"BACKWARD\n")

    if value == " ":
        ser.write(b"stop\n")

    if value == "m":
        ser.write(b"up\n")

    if value == "n":
        ser.write(b"down\n")

    time.sleep(1)

