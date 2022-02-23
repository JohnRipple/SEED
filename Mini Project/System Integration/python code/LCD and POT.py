#Joshua Lee
#Exercise 6
# this program takes in the voltage values from the arduino/potentiometer and sends it to the RPI which then prints the voltage values
# onto the LCD screen
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
# a function that reads in a value from the arduino or address
def readNumber():
    number = bus.read_byte(address)
    return number
# while loop to infinitly keep running
while True:
    # reads in a number from the arduino
    number = readNumber()
    # prints out the number that was read from the arduino
    print("Hey Arduino just read: ",number)
    print
    # sets color of LCD background to red to see if the LCD is receiving the signal/code
    lcd.color = [100, 0, 0]
    # prints out the number on the LCD
    lcd.message = str(number);
    # 2 second delay so that the LCD has time to register the next number and print out the value
    # this is also to check if the value received is the right one that we can compare from the serial monitor
    time.sleep(2);
