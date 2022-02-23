#Joshua Lee
#Exercise 8
# this program sends a value to the arduino and the arduino adds 5 to it and sends it back
# this is done through serial communication
# importing all necessary libraries
import serial
import time
# sets the serial address to the serial monitor at a 115200 Baud rate
ser = serial.Serial('/dev/ttyACM0',115200)
# delay of 3 seconds
time.sleep(3)
# function that reads from the arduino
def ReadfromArduino():
    # while the serial address/monitor is waiting for things to be sent
    while(ser.in_waiting > 0):
        # reads the line from the serial address/monitor and prints out the line that was read
        try:
            line = ser.readline().decode('utf-8').rstrip()
            print("serial output : ",line)
        # if there is an exception it prints out "Communication Error"
        except:
            print("Communication Error")
# asks the user to input a value between 1 and 9
value = input("please enter an integer between 1-9: ");
# encodes the values and sends it to the serial address/monitor for the arduino to read
ser.write(value.encode())
# sets a 2 second delay
time.sleep(2)
# sends a signal to arduino to read from it
ReadfromArduino()
print("Done")
