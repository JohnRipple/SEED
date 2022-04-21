'''
Team 6 - John Ripple, Michael Klima, Andrew Samson, Josh Lee
EENG 350 - Seed Lab (Spring 2022)
Final Demo Computer Vision 
'''
# -- Inclusions --
import cv2 as cv
import numpy as np
import math
from time import sleep
import serial

# Set up serial communication
ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
ser.reset_input_buffer()


# -- Function Declarations --
'''
Name:      sendSecondary()
Function:  Send data from the camera to the arduino and lcd
'''
def sendSecondary(angleH, inFrame, angle, stopSig):
    signS = 0
    signH = 0
    if angle < 0:   
        signS = 1
    if angleH < 0:
        signH = 1
    array = [round(abs(angleH)), signH, round(abs(angle)), signS, stopSig]
    s = " ".join(map(str, array)) + " \n"
    ser.write(s.encode('utf-8')) 
    line = ser.readline().decode('utf-8').rstrip()
    print(line)
    print("Horizontal Angle: %d   Shift Angle: %d   Stop Signal: %d" % ((pow(-1, array[1]))*array[0], (pow(-1, array[3]))*array[2], stopSig))
    

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
        phi = round(fov/2*((center-x)/center), 4)
        global phiOld
        global newValues
        if not (phi <= phiOld + 0.5 and phi >= phiOld - 0.5):
            phiOld = phi
            newValues = True
            #writeNumber(phi)
            #lcd.message = "Angle: " + str(phi)
            #print(phi)
        phiOld = phi
    return phiOld


# -- Main --
cross = False
lastCross = False
stop = False
cv.setUseOptimized(True)
phiOld = 0
angleOld = 0
data = np.load('camera_distort_matrices.npz') # Load the Previously found distorition matrix
found = True
newValues = False
kernel = np.ones((2,2), np.uint8)

#Set Camera values
cap = cv.VideoCapture(0)
cap.set(3, 320)                               # Set Camera width
cap.set(4, 240)                               # Set Camera height

while True:
    '''
for framein in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    frame = framein.array
    '''
    ret,frame= cap.read()
    
    # Uses previously calculated values to undistort image
    frame = cv.undistort(frame, data['mtx'], data['dist'], None, data['newcameramtx'])
    org = frame
    
    # Sends the frame through the filter process to get only the blue tape
    # H: 108  S: 255  V: 126 using displayColors.py
    # Multiple colors can be added to boundaries, only one is used
    bound = 15
    boundaries = [([90, 35, 90], [101+bound, 255, 130])] # For light blue tape
    #boundaries = [([90, 35, 80], [115, 200, 150])] # For dark blue tape
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
    (contours, _ )= cv.findContours(th.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    x = -1
    y = -1
    
    cross = False
    if len(contours) < 1:
        if found == True:
            print("No Markers Found")
            sendSecondary(0, False, 0, 2)
    else:
        # Create bounding box for the largest contour found 
        largest_item = max(contours, key=cv.contourArea)
        
        bot = tuple(largest_item[largest_item[:, :, 1].argmax()][0])
        #if bot[1] > 180: #reset flag section
            #stop = True
        M = cv.moments(largest_item)
        if M["m00"] >  10: 
            rect = cv.minAreaRect(largest_item)
            # Checking for the cross using bounding box ratio, area, and numer of countour points
            sizeRatio = rect[1][0]/rect[1][1]
            area = rect[1][0]*rect[1][1]
            #print("Ratio %f \t Area: %f" % (sizeRatio, area))
            if (sizeRatio < 2) and sizeRatio > 0.5 and area > 17000:
                epsilon = 0.01*cv.arcLength(largest_item,True)
                approx = cv.approxPolyDP(largest_item,epsilon,True)
                cv.drawContours(org, [approx], 0, (0,255,0),2)
                if len(approx) >= 12 and len(approx) < 17:
                    cross = True
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
                angleOld = round(angle, 4)
                #newValues = True
            angleOld = angle
            
            M = cv.moments(largest_item)
            #if M["m00"] >  0: 
            x = int(M['m10']/M['m00'])
            y = int(M['m01']/M['m00'])
        else:
            sendSecondary(0, False, 0, 2)
            
    if x == -1:
        found = False
    else:
        found = True
    phi = findangle(x, frame.shape[1]/2)
    #print("Last cross: " + str(lastCross) + "\t Cross: " + str(cross))
    if lastCross is True and cross is False:
        sendSecondary(0, found, 0, 1)
        print("Cross")
    elif newValues is True:
        sendSecondary(angleOld, found, phiOld, 0)
    lastCross = cross
    newValues = False
    #cv.imshow("Frame", frame)
    #cv.imshow("Threashold", th)
    cv.imshow("Original", org)
    
    if cv.waitKey(1) & 0xFF == 27:
        break
    elif cv.waitKey(1) & 0xFF == ord('r'):
        stop = False
    
cap.release()
cv.destroyAllWindows()

