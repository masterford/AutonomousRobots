# -*- coding: utf-8 -*-
"""
Created on Tue Oct 29 19:53:01 2019

@author: Gabe
"""

import cv2
import sys

import numpy as np

def auto_canny(img, sigma=0.33):
	# compute the median of the single channel pixel intensities
	v = np.median(img)
 
	# apply automatic Canny edge detection using the computed median
	lower = int(max(0, (1.0 - sigma) * v))
	upper = int(min(255, (1.0 + sigma) * v))
	edged = cv2.Canny(img, lower, upper)
 
	# return the edged image
	return edged

#video_capture = cv2.VideoCapture(0)

#while (1):
    #_, frame = video_capture.read()

img = cv2.imread("beer.jpg", 1)

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

lower_orange_1 = np.array([1,150,150])
upper_orange_1 = np.array([15,255,255])

mask_1 = cv2.inRange(hsv, lower_orange_1, upper_orange_1)

edged = auto_canny(mask_1)
cv2.imshow('edged', img)
cv2.waitKey(0)

#edgeImg = auto_canny(mask_1)
#cv2.imshow('raw', img)
#cv2.imshow('masked', mask_1)
#cv2.imshow('edged', edgeImg)

    
#    k = cv2.waitKey(5) & 0xFF
#    if k == 27:
#        break

#cv2.destroyAllWindows()