#import smbus
#import board
#import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import cv2 as cv
import numpy as np
import math
#import picamera
#from picamera import PiCamera
from time import sleep
#from picamera.array import PiRGBArray

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

data = np.load('camera_distort_matrices.npz')
cap = cv.VideoCapture(0)
#Set width and height
cap.set(3, 320)
cap.set(4, 240)
found = True
kernel = np.ones((5,5), np.uint8)
while True:
    ret,frame= cap.read()
    
    #Uses previously calculated values to undistort image
    frame = cv.undistort(frame, data['mtx'], data['dist'], None, data['newcameramtx'])
    org = frame
    
    #Sends the frame through the filter process to get only the yellow hexagon
    #H: 108  S: 255  V: 126 using displayColors.py
    #Multiple colors can be added to boundaries, only one is used
    bound = 7
    boundaries = [([101-bound, 0, 0], [101+bound, 255, 255])]

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
    _,contours, _ = cv.findContours(frame.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
    if len(contours) < 1:
        if found == True:
            print("No Markers Found")
    else: 
        largest_item = max(contours, key=cv.contourArea)
        #largest item
        frame = cv.cvtColor(frame, cv.COLOR_GRAY2BGR)
        rect = cv.minAreaRect(largest_item)
        box = cv.boxPoints(rect)
        box = np.int0(box)
        rows,cols = frame.shape[:2]
        [vx,vy,x,y] = cv.fitLine(largest_item, cv.DIST_L2,0,0.01,0.01)
        lefty = int((-x*vy/vx) + y)
        righty = int(((cols-x)*vy/vx)+y)
        cv.line(frame,(cols-1,righty),(0,lefty),(0,255,0),2)
        #hull = cv.convexHull(largest_item)
        cv.drawContours(frame,[box],0,(0,0,255),2)
        #box = geom.order_box(box)
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
        print(angle)
        

        M = cv.moments(largest_item)
        if M["m00"] >  0: 
            x = int(M['m10']/M['m00'])
            y = int(M['m01']/M['m00'])
            
    if x == -1:
        found = False
    else:
        found = True
    #findangle(x, frame.shape[1]/2)
    cv.imshow("Frame", frame)
    if cv.waitKey(1) & 0xFF == 27:
        break
    
cap.release()
cv.destroyAllWindows()

