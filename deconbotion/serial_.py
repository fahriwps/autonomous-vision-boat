import serial
import time

arduino = serial.Serial(port='/dev/ttyUSB0',baudrate=9600,timeout=0.01)
time.sleep(1)
for i in range(1,100):
    arduino.write(str(i).encode())
    print(i)
    time.sleep(0.02)
