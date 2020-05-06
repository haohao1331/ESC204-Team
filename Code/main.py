from 'motor.py' import all

import numpy as np
import cv2
import glob
import os
import math


cap=cv2.VideoCapture(0)
mtx=[[1.16887114*10**3, 0.00000000, 6.08676211*10**2],
 [0.00000000, 1.17063923*10**3, 3.59277197*10**2],
 [0.00000000, 0.00000000, 1.00000000]]
dist=[[2.74639413*10**(-1),  -2.39794893, -3.43932065*10**(-3), -3.75716784*10**(-3), 9.83950931]]

mtx=np.float32(mtx)
dist=np.float32(dist)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((7*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:7].T.reshape(-1,2)


while(True):
    if (cap.isOpened()):
        ret, img=cap.read()
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (7,7),None)
        dimensions = img.shape
        if ret==True:
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        else:
            continue
        coordinate=corners2[24]
        centre=[dimensions[0]/2.0, dimensions[1]/2.0]
        difference = coordinate-centre
        if abs(difference[0])<5: #Check for horizaontal difference
            #Get area of the chessboard in pixel squared
            #Calculate distance based on area of chessboard
            #if (area fills the screen):
                #auto moves to complete target
            #else:
                #move forward such that the chessboard fills reaches the boundary of the picture
        else:
            #adjust left or right (to be filled in later)
            

        
    else:
        print("Error!")
        break
        
