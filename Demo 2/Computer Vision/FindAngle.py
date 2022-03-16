import numpy as np
import cv2
import math

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

# get the rotated box
x = cv2.imread('vertical.png') 
cv_image = cv2.cvtColor(x, cv2.COLOR_RGB2GRAY)
ret, thresh = cv2.threshold(cv_image,70,255,cv2.THRESH_BINARY)
contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
recta = cv2.minAreaRect(contours[0])
center_x, center_y, angle = recta

# get the corners
box = cv2.boxPoints(recta)
box = np.int0(box)

# choose the first point
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