'''
Team 6
EENG 350 - Seed Lab (Spring 2022)
Demo 2 Computer Vision 
'''
# -- Inclusions --
import cv2 as cv
import numpy as np
import math
import imutils
from time import sleep


# -- Main --
pi = False
stop = False
cv.setUseOptimized(True)
phiOld = 0
angleOld = 0
data = np.load('camera_distort_matrices.npz') # Load the Previously found distorition matrix
found = True
newValues = False
kernel = np.ones((2,2), np.uint8)

cap = cv.VideoCapture(0)
cap.set(3, 320)                               # Set Camera width
cap.set(4, 240)                               # Set Camera height

while True:
    frame = cv.imread("cross.png")

    # Uses previously calculated values to undistort image
    frame = cv.undistort(frame, data['mtx'], data['dist'], None, data['newcameramtx'])
    org = frame

    # Sends the frame through the filter process to get only the yellow hexagon
    # H: 108  S: 255  V: 126 using displayColors.py
    # Multiple colors can be added to boundaries, only one is used
    bound = 15
    boundaries = [([50, 0, 0], [130, 255, 255])]
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
    contours, _ = cv.findContours(th.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    x = -1
    y = -1

    if len(contours) < 1:
        if found == True:
            print("No Markers Found")

    else:
        # Create bounding box for the largest contour found 
        largest_item = max(contours, key=cv.contourArea)
        epsilon = 0.01*cv.arcLength(largest_item,True)
        approx = cv.approxPolyDP(largest_item,epsilon,True)
        print(len(approx))
        cv.drawContours(org, [approx], 0, (0,255,0),2)
        bot = tuple(largest_item[largest_item[:, :, 1].argmax()][0])
        #if bot[1] > 180: #reset flag section
            #stop = True
        M = cv.moments(largest_item)
        if M["m00"] >  10: 
            rect = cv.minAreaRect(largest_item)
            box = cv.boxPoints(rect)
            box = np.int0(box)
            cv.drawContours(frame,[box],0,(0,0,255),2)
            cv.drawContours(org,[box],0,(0,0,255),2)
            root = box[0]
            
            M = cv.moments(largest_item)
            #if M["m00"] >  0: 
            x = int(M['m10']/M['m00'])
            y = int(M['m01']/M['m00'])

            
    if x == -1:
        found = False

    #if found == True:
        #sendSecondary(angleOld, found, phiOld)
    newValues = False
    cv.imshow("Frame", frame)
    cv.imshow("Threashold", th)
    cv.imshow("Original", org)

    #rawCapture.truncate(0)
    if cv.waitKey(1) & 0xFF == 27:
        break
    elif cv.waitKey(1) & 0xFF == ord('r'):
        stop = False

cap.release()
cv.destroyAllWindows()

