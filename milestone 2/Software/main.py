#from 'imu.py' import all #commented out for testing purpose
#from 'motor.py' import all

import numpy as np #import required libraries
import cv2
import glob
import os
import math


cap=cv2.VideoCapture(0,cv2.CAP_DSHOW)
#below are matrices associated with the intrinsic parameters of the camera. One is the camera materix
# and the other is the distortion matrix. Both are obtained from prior callibration
mtx=[[1.16887114*10**3, 0.00000000, 6.08676211*10**2],
 [0.00000000, 1.17063923*10**3, 3.59277197*10**2],
 [0.00000000, 0.00000000, 1.00000000]]
dist=[[2.74639413*10**(-1),  -2.39794893, -3.43932065*10**(-3), -3.75716784*10**(-3), 9.83950931]]

mtx=np.float32(mtx)
dist=np.float32(dist)

#Below sets up chess corner detection
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((7*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:7].T.reshape(-1,2)

#After callibration, we can calulate distance based on 
def calcDist(width): #in cm
    return 60/width*13.8
def calcDistpx(corners2): #in px
    gridwidth = ((corners2[24][0][0]-corners2[23][0][0])**2+(corners2[24][0][1]-corners2[23][0][1])**2)**0.5
    gridwidth += ((corners2[24][0][0]-corners2[25][0][0])**2+(corners2[24][0][1]-corners2[25][0][1])**2)**0.5
    gridwidth += ((corners2[24][0][0]-corners2[17][0][0])**2+(corners2[24][0][1]-corners2[17][0][1])**2)**0.5
    gridwidth += ((corners2[24][0][0]-corners2[31][0][0])**2+(corners2[24][0][1]-corners2[31][0][1])**2)**0.5
    gridwidth/=4
    return gridwidth
def calcHorDist(difference):#in cm
    return difference[0][0]/60*1.25
    
#while loop for target detection, namely detecting the centre of the chessbaord.
while(True):
    if (cap.isOpened()):#check if camera is connected
        ret, img=cap.read()#take picture
        img=cv2.undistort(img, mtx, dist, None, mtx)#undistort the image given the distortion matrix
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)#convert to gray pictures for processing
        ret, corners = cv2.findChessboardCorners(gray, (7,7),None)#Find chessboard corner
        dimensions = img.shape #Gives the dimension of the image
        if ret==True: #if chessboard is found
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria) #genarating corner matrix
        else:
            cv2.imshow('img',cv2.resize(img,None,fx=1, fy=1, interpolation = cv2.INTER_CUBIC)) #Display image
            k = cv2.waitKey(1)
            if k==ord('q'):
                break
            continue
        coordinate=corners2[24] #24 is the centre grid
        gridwidth = ((corners2[24][0][0]-corners2[23][0][0])**2+(corners2[24][0][1]-corners2[23][0][1])**2)**0.5
        gridwidth=calcDistpx(corners2)
        #below finds distance in actual scale.
        distance=calcDist(gridwidth)
        centre=[dimensions[1]//2, dimensions[0]//2]
        difference = coordinate-centre
        hordist=calcHorDist(difference)
    
        print(distance,gridwidth,hordist)#print distances for debugging

        img=cv2.circle(img,(centre[0],centre[1]), 5, (0,0,255), -1)
        cv2.imshow('img',cv2.resize(img,None,fx=1, fy=1, interpolation = cv2.INTER_CUBIC))
        k = cv2.waitKey(1)
        if k==ord('q'):
            break
        if abs(difference[0][0])<5: #Check for horizaontal difference, 5 pixels is the tolerance
            gridwidth=calcDistpx(corners2)
            distance=calcDist(gridwidth)            
            if (distance>14.5):#First try to fill up the screen with the chessboard for accuracy
                print("move forward by "+str(distance-14.5))
            elif (distance <14):
                print("move backward by "+str(14-distance))
            else:
                print("move right/left by preset distance and move forward by 14.25") #taking an average value between 14 and 14.5 cm
        else:
            if (difference[0][0]<0): #if not centred, move left or right to adjust
                print("move left by "+str(hordist))
            else:
                print("move right by "+str(hordist))
            
        #afterward, the robot should move by a preset distance, the distance between the centre of the charging port and the 
        #centre of the chessboard. This should be done after some callibration with physical model. Then break
        
    else:
        print("Error!")
        break
        
