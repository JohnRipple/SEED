import cv2 as cv
import numpy as np

img = cv.imread('calib_test3.jpg', 1)
cv.imshow('image', img)
cv.waitKey(0)
cv.destroyAllWindows()

