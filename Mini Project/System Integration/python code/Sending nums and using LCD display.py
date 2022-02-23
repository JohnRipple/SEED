import smbus
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
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

def writeNumber(value,offset):
    #bus.write_byte(address,value)
    bus.write_byte_data(address,offset,value)
    return -1

def readNumber():
    number = bus.read_byte(address)
    #number = bus.read_byte_data(address,0)
    return number

while True:
    var = input("Enter 1 - 9:")
    var = int(var)
    if not var:
        continue
    off = input("Enter either 1 or 0 for offset:");
    off = int(off);
    writeNumber(var,off)
    print("RPI: Hi Arduino, I sent you ",var)
    number = readNumber()
    print("Arduino: Hey RPI, I sent u back ",number)
    print
    time.sleep(1)
    lcd.color = [100, 0, 0]
    lcd.message = "sent: " + str(var) + "\ngot: " + str(number);
    time.sleep(1);
    
