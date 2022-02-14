import cv2 as cv
import numpy as np
import picamera
from picamera import PiCamera
from time import sleep
from statistics import mean
from picamera.array import PiRGBArray

#Turns an image hsv to bgr
def bgr(image):
    img = cv.cvtColor(image, cv.COLOR_HSV2BGR)
    return img

#Turns an image HSV
def hsv(image):
    img = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    return img

#Turns an image HSV to Grayscale
def gry(image):
    image = bgr(image)
    img = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    return img

#Resizes the image to half its size
def resize(img):
    res = cv.resize(img, None, fx=0.5, fy=0.5, interpolation = cv.INTER_AREA)
    return res

#Takes picture and filters out everything except yellow
def filtercolor(img):
    #H: 18   S: 255  V: 162 using displayColors.py
    #Multiple colors can be added to boundaries, only yellow is used
    boundaries = [([18-10, 100, 100], [18+10, 255, 255])]
    #Convert img to hsv and resize it by half
    img = hsv(img)
    img = resize(img)
    #Iterate through boundaries
    for (lower,upper) in boundaries:
        #create NumPy arrays from boundaries
        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")
        
        #find colors within the specified boundaries and apply mask
        mask = cv.inRange(img, lower, upper)
        output = cv.bitwise_and(img, img, mask = mask)
        return output

#Cleans up an image using filtering and transformations
def cleanimg(img):
    #Creates a 5x5 kernel of ones and uses it to Morph open the image
    kernel = np.ones((10,10), np.uint8)
    clean = cv.morphologyEx(img, cv.MORPH_OPEN, kernel)
    #clean = cv.erode(clean,kernel,iterations = 3)
    return clean

#Determines location of object in image
def findpos(img, found):
    #Convert image to grayscale and then threshold it
    gray = gry(img)
    ret, th = cv.threshold(gray, 0, 255, cv.THRESH_BINARY+cv.THRESH_OTSU)
    th = cv.Canny(th, 50, 100)
    #Find the positions of all non zero values in the image
    x = -1
    y = -1
    _, contours, _ = cv.findContours(th.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    if len(contours) < 1:
        if found == True:
            print("No Markers Found")
        return -1, -1, th
    largest_item = max(contours, key=cv.contourArea)
    
    #largest item
    M = cv.moments(largest_item)
    if M["m00"] >  0: 
        x = int(M['m10']/M['m00'])
        y = int(M['m01']/M['m00'])
    return x,y, th
    
#Sets the awb to off
def calibration(size):
    camera = PiCamera()
    # Set ISO to the desired value
    camera.iso = 800
    camera.framerate = 24;
    camera.resolution = size
    
    # Wait for the automatic gain control to settle
    sleep(0.1)
    
    # Fix the exposure, can't see bright things if not originally pointing at them
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    
    # Sets awb to a specific value
    g = camera.awb_gains
    print(g)
    g=((399/256), (155/128))
    camera.awb_mode = 'off'
    camera.awb_gains = g
    return camera

#Captures a video of only the yellow hexagon
def videoproc():
    size = (640, 480)
    camera = calibration(size)
    rawCapture = PiRGBArray(camera, size)
    found = True
    for framein in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        frame = framein.array
        
        #Sends the frame through the filter process to get only the yellow hexagon
        frame = filtercolor(frame)
        frame = cleanimg(frame)
        
        #Finds the center of the hexagon in the image
        x,y, frame = findpos(frame, found)
        if x == -1:
            found = False
        else:
            found = True
        #findangle(x, frame.shape[1]/2)
        findquad(x, y, frame.shape[1]/2, frame.shape[0]/2)
        
        cv.imshow("Frame", frame)
        if cv.waitKey(1) & 0xFF == 27:
            break
        rawCapture.truncate(0)
    camera.close()
    cv.destroyAllWindows()

def findangle(x, center):
    #Pi camera FOV is 53.5 deg Horizontal 41.41 deg Vertical, 67 deg diagonal
    fov = 53.5
    if x is not -1:
        #Find the angle relative to the center of the image to rotate the camera
        phi = fov/2*((x-center)/center)
        print(phi)

def findquad(x, y, xcent, ycent):
    msg = ""
    if x > xcent and y > ycent:
        msg = "SE"
    elif x > xcent and y < ycent:
        msg = "NE"
    elif x < xcent and y > ycent:
        msg = "SW"
    elif x < xcent and y < ycent:
        msg = "NW"
    if x != -1 and y != -1:
        print(msg)

videoproc()
