'''
This program takes a user input filename and saves a picture to it.
The picture can be resized to half its size and turned grayscale.
'''

import cv2 as cv
import numpy as np
import picamera
import time
from picamera.array import PiRGBArray

clr = True

def getColor(event, x, y, flags, param):
    if event == cv.EVENT_LBUTTONDOWN:
        if clr == False:
            #Prints the BGR pixel values of img
            print("B: {}\tG: {}\tR: {}".format(image[y,x,0], image[y,x,1], image[y,x,2]))
        else:
            #Gets the BGR value of a pixel and converts it to hsv value that is then printed
            bgr = np.uint8([[[image[y,x,0], image[y,x,1], image[y,x,2]]]])
            hsv = cv.cvtColor(bgr, cv.COLOR_BGR2HSV)
            print("H: {}\tS: {}\tV: {}".format(hsv[0][0][0], hsv[0][0][1], hsv[0][0][2]))
            #print("H: {}\tS: {}\tV: {}".format(hsv[0][0][0]*2, hsv[0][0][1]/255*100, hsv[0][0][2]/255*100))

#Turns an image grayscale
def bgr(image):
    img = cv.cvtColor(image, cv.COLOR_HSV2BGR)
    return img

#Turns an image HSV
def hsv(image):
    img = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    return img

#Resizes the image to half its size
def resize(img):
    res = cv.resize(img, None, fx=0.5, fy=0.5, interpolation = cv.INTER_AREA)
    return res


#Sets up camera
camera = picamera.PiCamera()
size = (640, 480)
camera.resolution = size
camera.framerate = 24
# Wait for the automatic gain control to settle
time.sleep(2)
# Now fix the values

camera.shutter_speed = camera.exposure_speed
camera.exposure_mode = 'off'

camera.awb_mode = "off"
camera.awb_gains = (343/256, 101/64)

rawCapture = PiRGBArray(camera, size)
cv.namedWindow('image')
cv.setMouseCallback('image', getColor)
print("t takes a picture, s saves, m changes colorspace, r retakes picture")

for framein in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = framein.array
    #Creates a window that can be used with a mouse
    cv.imshow('image', image)
    #Checks what the keyboard presses are while in the window
    k = cv.waitKey(1) & 0xFF
    if k == ord('q'):
        cv.destroyAllWindows()
        exit(0)
    elif k == ord('s'):
        userInput = input("Enter a filename: ")
        userInput = userInput + ".jpg"
        cv.imwrite(userInput, image)
    elif k == ord('m'):
        clr = not clr
    elif k == ord('t'):
        while(1):
            k = cv.waitKey(1) & 0xFF
            if k == ord('r'):
                break
            elif k == ord('s'):
                userInput = input("Enter a filename: ")
                userInput = userInput + ".jpg"
                cv.imwrite(userInput, image)
                break
            elif k == ord('q'):
                cv.destroyAllWindows()
                exit(0)
            
    rawCapture.truncate(0)
camera.close()
