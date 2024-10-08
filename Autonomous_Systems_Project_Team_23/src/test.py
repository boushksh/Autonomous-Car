import serial
import struct

# Open the serial port
ser = serial.Serial('/dev/ttyACM0', 9600)

while True:
    # Read the binary data of the integer variable from the serial port
    data = ser.read(struct.calcsize('i'))

    # Unpack the binary data into an integer variable
    integerValue = struct.unpack('i', data)[0]

    # Process the integer value as needed
    print(integerValue)