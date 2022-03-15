import smbus
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import cv2 as cv
import numpy as np
import picamera
from picamera import PiCamera
from time import sleep
from picamera.array import PiRGBArray

'''
#setting address/bus
bus = smbus.SMBus(1)
address = 0x04

#initialize I2C bus
i2c = board.I2C()

#lcd settings
lcd_columns = 16
lcd_rows = 2
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.clear()

# function that sends number to arduino
def writeNumber(value):
    bus.write_byte(address,value)
    return -1
'''

def findangle(x, center):
    #Pi camera FOV is 53.5 deg Horizontal 41.41 deg Vertical, 67 deg diagonal
    fov = 53.5
    if x != -1:
        #Find the angle relative to the center of the image to rotate the camera
        phi = round(fov/2*((center-x)/center), 4)
        global phiold
        if phi != phiold:
            phiold = phi
            #writeNumber(phi)
            #lcd.message = "Angle: " + str(phi)
            print(phi)

cv.setUseOptimized(True)
phiold = 100.00

#Camera Calibration
size = (320, 240)
camera = picamera.PiCamera()
camera.resolution = size
camera.framerate = 24
# Wait for the automatic gain control to settle
sleep(2)
# Now fix the values
camera.shutter_speed = camera.exposure_speed
camera.exposure_mode = 'off'
camera.awb_mode = "off"
camera.awb_gains = (343/256, 101/64)
rawCapture = PiRGBArray(camera, size)

#Load distorition matrix and set variables
data = np.load('camera_distort_matrices.npz')
found = True
kernel = np.ones((5,5), np.uint8)
for framein in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    frame = framein.array
    
    #Uses previously calculated values to undistort image
    frame = cv.undistort(frame, data['mtx'], data['dist'], None, data['newcameramtx'])
    
    #Filters a specific range of color using HSV colorspace
    #H: 108  S: 255  V: 126 using displayColors.py
    #Multiple colors can be added to boundaries, only one is used
    bound = 10
    boundaries = [([108-bound, 0, 0], [108+bound, 255, 255])]

    #Convert img to hsv and resize it by half
    frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = np.zeros((frame.shape[0], frame.shape[1]), dtype="uint8")
    #Iterate through boundaries
    for (lower,upper) in boundaries:
        #create NumPy arrays from boundaries
        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")
        #find colors within the specified boundaries and apply mask
        mask = mask | cv.inRange(frame, lower, upper)
        
    frame = cv.bitwise_and(frame, frame, mask = mask)

    #Reduces noise by eroding and then expanding
    frame = cv.morphologyEx(frame, cv.MORPH_OPEN, kernel)
    
    #Finds the center of the largest object left in the image
    #Convert image to grayscale and then threshold it
    gray = cv.cvtColor(frame, cv.COLOR_HSV2BGR)
    gray = cv.cvtColor(gray, cv.COLOR_BGR2GRAY)
    gray = cv.GaussianBlur(gray, (5,5),0)
    ret, th = cv.threshold(gray, 0, 255, cv.THRESH_BINARY+cv.THRESH_OTSU)
    frame = cv.Canny(th, 100, 200)
    #Find the positions of all non zero values in the image
    x = -1
    y = -1
    _, contours, _ = cv.findContours(frame.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    
    if len(contours) < 1:
        if found == True:
            print("No Markers Found")
    else: 
        largest_item = max(contours, key=cv.contourArea)
        #largest item
        M = cv.moments(largest_item)
        if M["m00"] >  0: 
            x = int(M['m10']/M['m00'])
            y = int(M['m01']/M['m00'])
            
    if x == -1:
        found = False
    else:
        found = True
    findangle(x, frame.shape[1]/2)
    cv.imshow("Frame", frame)
    if cv.waitKey(1) & 0xFF == 27:
        break
    rawCapture.truncate(0)
camera.close()
cv.destroyAllWindows()

