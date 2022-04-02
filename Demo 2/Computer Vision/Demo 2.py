'''
Team 6
EENG 350 - Seed Lab (Spring 2022)
Demo 2 Computer Vision 
'''
# -- Inclusions --
import smbus
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import cv2 as cv
import numpy as np
import math
from picamera import PiCamera
from picamera.array import PiRGBArray
from time import sleep

#setting address/bus
bus = smbus.SMBus(1)
address = 0x04

# function that sends number to arduino
def writeNumber(value):
    bus.write_byte(address,value)
    return -1

# initialize I2C bus
i2c = board.I2C()

# lcd settings
lcd_columns = 16
lcd_rows = 2
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.clear()



# -- Function Declarations --
'''
Name:      sendSecondary()
Function:  Send data from the camera to the arduino and lcd
'''
def sendSecondary(angleH, inFrame, angle):
    signS = 0
    signH = 0
    if angle < 0:   
        signS = 1
    if angleH < 0:
        signH = 1
    array = [round(abs(angleH)*100), signH, round(abs(angle)*100), signS]
    print("Horizontal Angle: %d   Shift Angle: %d" % ((pow(-1, array[1]))*array[0], (pow(-1, array[3]))*array[2]))
    
    try:
        bus.write_i2c_block_data(address, 0, array)
    except:
        print("I2C connection failed, please check connection")
    '''
    lcd.clear()
    if inFrame:
        lcd.message = "Angle:\n" + str(angle)
    else:
        lcd.message = "Searching..."
    '''

# 2d distance
def dist2D(one, two):
    dx = one[0] - two[0]
    dy = one[1] - two[1]
    return math.sqrt(dx*dx + dy*dy)

# angle between three points (the last point is the middle)
def angle3P(p1, p2, p3):
    # get distances
    a = dist2D(p3, p1)
    b = dist2D(p3, p2)
    c = dist2D(p1, p2)

    # calculate angle // assume a and b are nonzero
    # (law of cosines)
    numer = c**2 - a**2 - b**2
    denom = -2 * a * b
    if denom == 0:
        denom = 0.000001
    rads = math.acos(numer / denom)
    degs = math.degrees(rads)

    # check if past 90 degrees
    return degs

'''
Name:      findangle()
Function:  Find the angle offset from the center of the screen
Inputs: x position of countour, center of image
Output: phi angle offset from center line
'''
def findangle(x, center):
    #Pi camera FOV is 53.5 deg Horizontal 41.41 deg Vertical, 67 deg diagonal
    fov = 53.5
    if x != -1:
        #Find the angle relative to the center of the image to rotate the camera
        phi = round(fov/2*((center-x)/center), 2)
        global phiOld
        global newValues
        if not (phi <= phiOld + 1 and phi >= phiOld - 1):
            phiOld = phi
            newValues = True
            #writeNumber(phi)
            #lcd.message = "Angle: " + str(phi)
            #print(phi)
    return phiOld

'''
Name:      calibrate()
Function:  Initalize the PyCamera and fix the white balance
Input:     None
Output:    None
'''
def calibrate(camera):
    camera.resolution = (320, 240) # Set resolution
    #camera.iso = 50               # Fix iso (100/200 for light, 300/400 for dark)
    camera.framerate = 24          # Fix framerate
    sleep(2)                  # Allow camera to adjust
    #camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'   # Auto exposure off
    #g = camera.awb_gains
    camera.awb_mode = 'off'        # Auto white balance off
    #camera.awb_gains = g
    camera.awb_gains = (343/256, 101/64)

# -- Main --
cv.setUseOptimized(True)
phiOld = 0
angleOld = 0
data = np.load('camera_distort_matrices.npz') # Load the Previously found distorition matrix
found = True
newValues = False
kernel = np.ones((2,2), np.uint8)

camera = PiCamera() # Initialize PyCamera and calibrate
rawCapture = PiRGBArray(camera)
calibrate(camera)
'''
cap = cv.VideoCapture(0)
cap.set(3, 320)                               # Set Camera width
cap.set(4, 240)                               # Set Camera height
while True:
'''
for framein in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    frame = framein.array
    #ret,frame= cap.read()
    
    # Uses previously calculated values to undistort image
    frame = cv.undistort(frame, data['mtx'], data['dist'], None, data['newcameramtx'])
    org = frame
    
    # Sends the frame through the filter process to get only the yellow hexagon
    # H: 108  S: 255  V: 126 using displayColors.py
    # Multiple colors can be added to boundaries, only one is used
    bound = 15
    boundaries = [([101-bound, 50, 50], [101+bound, 255, 255])]
    #boundaries = [([95, 100, 20], [125,255,255])]
    frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)  # Convert to HSV
    mask = np.zeros((frame.shape[0], frame.shape[1]), dtype="uint8")
    # Iterate through boundaries
    for (lower,upper) in boundaries:
        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")
        # find colors within the specified boundaries and apply mask
        mask = mask | cv.inRange(frame, lower, upper) 
    frame = cv.bitwise_and(frame, frame, mask = mask)
    #frame = cv.morphologyEx(frame, cv.MORPH_OPEN, kernel)

    # Finds the center of the largest object left in the image
    gray = cv.cvtColor(frame, cv.COLOR_HSV2BGR)  # Convert image to grayscale
    gray = cv.cvtColor(gray, cv.COLOR_BGR2GRAY)  # Convert 
    gray = cv.GaussianBlur(gray, (5,5),0)
    ret, th = cv.threshold(gray, 0, 255, cv.THRESH_BINARY+cv.THRESH_OTSU)
    (_,contours, _ )= cv.findContours(th.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    x = -1
    y = -1
    
    if len(contours) < 1:
        if found == True:
            print("No Markers Found")
            sendSecondary(100, False, 100)
    else:
        # Create bounding box for the largest contour found 
        largest_item = max(contours, key=cv.contourArea)
        M = cv.moments(largest_item)
        if M["m00"] >  10: 
            rect = cv.minAreaRect(largest_item)
            box = cv.boxPoints(rect)
            box = np.int0(box)
            cv.drawContours(frame,[box],0,(0,0,255),2)
            cv.drawContours(org,[box],0,(0,0,255),2)
            root = box[0]

            # find the longer side
            end = None
            one = box[-1]
            two = box[1]
            if dist2D(one, root) > dist2D(two, root):
                end = one
            else:
                end = two

            # take the left-most point
            left_point = None
            right_point = None
            if end[0] < root[0]:
                left_point = end
                right_point = root
            else:
                left_point = root
                right_point = end

            # calculate the angle [-90, 90]
            offshoot = [left_point[0] + 100, left_point[1]]
            angle = angle3P(right_point, offshoot, left_point)
            if left_point[1] > right_point[0]:
                angle = -angle
            if not (angle <= angleOld + 1 and angle >= angleOld - 1):
                angleOld = round(angle, 2)
                newValues = True
            
            M = cv.moments(largest_item)
            #if M["m00"] >  0: 
            x = int(M['m10']/M['m00'])
            y = int(M['m01']/M['m00'])
            
    if x == -1:
        found = False
    else:
        found = True
    phi = findangle(x, frame.shape[1]/2)
    #if newValues is True:
        #sendSecondary(angleOld, found, phiOld)
    sendSecondary(angle, found, phi)
    newValues = False
    cv.imshow("Frame", frame)
    cv.imshow("Threashold", th)
    cv.imshow("Original", org)
    rawCapture.truncate(0)
    if cv.waitKey(1) & 0xFF == 27:
        break
    
#cap.release()
cv.destroyAllWindows()

