import struct
import serial
import time
import string

ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.5)

time.sleep(0.5)

commands = (200, 240, 190, 180)

string = ""
for i in commands:
    string = string + struct.pack('!B',i)


while True:
    ser.write(string)
    time.sleep(1)
    #print(string)
