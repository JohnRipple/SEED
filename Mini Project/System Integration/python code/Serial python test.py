import serial
import time

ser = serial.Serial('/dev/ttyACM0',115200)
time.sleep(3)

def ReadfromArduino():
    while(ser.in_waiting > 0):
        try:
            line = ser.readline().decode('utf-8').rstrip()
            print("serial output : ",line)
        except:
            print("Communication Error")

value = "hello" + "\n"
ser.write(value.encode())
time.sleep(2)
ReadfromArduino()
print("Done")
