#from 'imu.py' import all
#from 'motor.py' import all

import numpy as np
import cv2
import glob
import os
import math


cap=cv2.VideoCapture(0,cv2.CAP_DSHOW)
mtx=[[1.16887114*10**3, 0.00000000, 6.08676211*10**2],
 [0.00000000, 1.17063923*10**3, 3.59277197*10**2],
 [0.00000000, 0.00000000, 1.00000000]]
dist=[[2.74639413*10**(-1),  -2.39794893, -3.43932065*10**(-3), -3.75716784*10**(-3), 9.83950931]]

mtx=np.float32(mtx)
dist=np.float32(dist)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((7*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:7].T.reshape(-1,2)

def calcDist(width): #in cm
    return 60/width*13.8
def calcDistpx(corners2): #in px
    gridwidth = ((corners2[24][0][0]-corners2[23][0][0])**2+(corners2[24][0][1]-corners2[23][0][1])**2)**0.5
    gridwidth += ((corners2[24][0][0]-corners2[25][0][0])**2+(corners2[24][0][1]-corners2[25][0][1])**2)**0.5
    gridwidth += ((corners2[24][0][0]-corners2[17][0][0])**2+(corners2[24][0][1]-corners2[17][0][1])**2)**0.5
    gridwidth += ((corners2[24][0][0]-corners2[31][0][0])**2+(corners2[24][0][1]-corners2[31][0][1])**2)**0.5
    gridwidth/=4
    return gridwidth
def calcHorDist(difference):
    return difference[0][0]/60*1.25
while(True):
    if (cap.isOpened()):
        ret, img=cap.read()
        img=cv2.undistort(img, mtx, dist, None, mtx)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (7,7),None)
        dimensions = img.shape
        if ret==True:
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        else:
            cv2.imshow('img',cv2.resize(img,None,fx=1, fy=1, interpolation = cv2.INTER_CUBIC))
            k = cv2.waitKey(1)
            if k==ord('q'):
                break
            continue
        coordinate=corners2[24]
        gridwidth = ((corners2[24][0][0]-corners2[23][0][0])**2+(corners2[24][0][1]-corners2[23][0][1])**2)**0.5
        gridwidth=calcDistpx(corners2)
        distance=calcDist(gridwidth)
        centre=[dimensions[1]//2, dimensions[0]//2]
        difference = coordinate-centre
        hordist=calcHorDist(difference)

        print(distance,gridwidth,hordist)

        img=cv2.circle(img,(centre[0],centre[1]), 5, (0,0,255), -1)
        cv2.imshow('img',cv2.resize(img,None,fx=1, fy=1, interpolation = cv2.INTER_CUBIC))
        k = cv2.waitKey(1)
        if k==ord('q'):
            break
        if abs(difference[0][0])<5: #Check for horizaontal difference
            gridwidth=calcDistpx(corners2)
            distance=calcDist(gridwidth)            
            if (distance>14.5):
                print("move forward by "+str(distance-14.5))
            elif (distance <14):
                print("move backward by "+str(14-distance))
            else:
                print("move right/left by preset distance and move forward by 14.25")
        else:
            if (difference[0][0]<0):
                print("move left")
            else:
                print("move right")
            

        
    else:
        print("Error!")
        break
        
