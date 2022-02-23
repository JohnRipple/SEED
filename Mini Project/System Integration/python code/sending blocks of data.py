#Joshua Lee
#Exercise 5 and 7
# this program takes in a user defined string and converts each letter into integer ASCII values
# and is then sent to the arduino where the arduino reverses the order of the array and sends it back
# to the RPI and the RPI converts the integers back to characters and outputs it
# this also produces a string to the LCD("I2C Error") if the SDA wire/pin is removed and there produces an IOError
# importing all necessary libraries
import smbus
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
# setting the address and bus value so that the RPI and arduino can properly talk and send/retrieve data from the right address/place
bus = smbus.SMBus(1)
address = 0x04
# Modify this if you have a different sized Character LCD
lcd_columns = 16
lcd_rows = 2

# Initialise I2C bus.
i2c = board.I2C()  # uses board.SCL and board.SDA

# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
#clears the LCD screen
lcd.clear()
# initializing 3 different arrays data which is the data that we are sending to the arduino
# numberarray which is the array that is getting send back from the arduino
# returndata which is the data that we are outputting back out through the pi and through python 3
data = [];
numberarray = [];
returndata = [];
# function that sends a block/array of values to the address/to the arduino
def writeNumber(value):
    bus.write_i2c_block_data(address,0,value)
    return -1
# function that receives a block/array of Bytes from the address/from the arduino
def readNumber():
    numberarray = bus.read_i2c_block_data(address,0,32)
    return numberarray
# while loop to infinitly keep running
while True:
    # try catch statement to catch an IOError and keep running the program if that error occurs
    try:
        #clears both the data and returndata arrays so that previous strings don't leak into future strings
        data.clear();
        returndata.clear();
        #asking user for a string and putting into the variable var
        var = input("Enter a string: ")
        # for loop to iterate through string and convert each character into an integer value and putting it into the array data
        for char in var:
            temp = ord(char);
            data.append(temp);
        # sending the array to the arduino
        writeNumber(data);
        # printing out the array that was sent to the arduino
        print("RPI: Hi Arduino, I sent you ",data)
        # setting a 1 second timer/delay to not ask the arduino for information/values that haven't been processed yet
        time.sleep(1);
        # getting an array back from the arduino and setting it to numberarray
        numberarray = readNumber();
        # for loop to convert the array of integer values back to character
        for char in numberarray:
            # if the integer is a 0 or an empty placeholder because the arduino sends back an array of size 32 then it goes to the next
            if (char == 0):
                continue;
            # if the integer is not a 0 then convert it to a character and add it the returndata array
            else:
                temp = chr(char);
                returndata.append(temp);
        # print out the string but in reverse
        print("Arduino: Hey RPI, I sent u back ",returndata)
        print
    # exception block to catch the IOError exception and print out string "I2C Error" to the LCD
    except IOError:
        lcd.message = "I2C Error";
    
