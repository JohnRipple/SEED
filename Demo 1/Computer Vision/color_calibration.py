'''
This program takes a user input filename and saves a picture to it.
The picture can be resized to half its size and turned grayscale.
'''

import cv2 as cv
import numpy as np
import picamera
import time

clr = False

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
    res = cv.resize(img, None, fx=0.55, fy=0.5, interpolation = cv.INTER_AREA)
    return res


#Sets up camera
camera = picamera.PiCamera()
camera.resolution = (1920, 1088)

#camera.start_preview()
time.sleep(0.1) #Let camera lens adjust to the light
g=((380/256), (155/128))
camera.awb_mode = 'off'
camera.awb_gains = g



while(1):
    #Creates a window that can be used with a mouse
    camera.start_preview(alpha=200)
    input("Ready to take picture? (enter): ")
    print("q quits, r retakes picture, m changes colorspace")
    #Set up cv object and capture image
    image = np.empty((1088 * 1920 * 3,), dtype=np.uint8)
    camera.capture(image, 'bgr')
    #camera.stop_preview()
    image = image.reshape((1088, 1920, 3))
    image = resize(image)
    cv.namedWindow('image')
    cv.setMouseCallback('image', getColor)
    camera.stop_preview()
    while(1):
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
            break
        elif k == ord('m'):
            clr = not clr
        elif k == ord('r'):
            break