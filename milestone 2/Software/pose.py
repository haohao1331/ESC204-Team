import numpy as np
import cv2
import glob
import os
import math
##modify line 7 for source, enter 0 if directly taken from camera
filename='VID_20200214_152950.mp4'
filename =0
##modily camera 0 for my Huawei phone, 1 for PiCam, 2 for Microsoft Webcam
camera=2
##modify key 0 for keyboard updated frame, key 1 for non interrupted video output
key=1
## press any key to advance a frame, press q to quit
savefile=[]
cap = cv2.VideoCapture(cv2.CAP_DSHOW)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((7*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:7].T.reshape(-1,2)
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output1.avi',fourcc, 20.0, (640,480))
mtx=[1,2,3]
dist=[1,2,3]
mtx[0]=[[3.99357119*10**3, 0, 1.46751797*10**3],[0, 4.10744370*10**3, 1.37886128*10**3],[0, 0, 1.00000000*10**0]]

dist[0]=[[-0.35385041,  1.93301963,  0.01392529, -0.0221485,  -3.27667525]]

mtx[1]=[[2.33231417*10**3, 0.00000000, 1.25557633*10**3],
 [0.00000000, 2.32852142*10**3, 9.16960057*10**2],
 [0.00000000, 0.00000000, 1.00000000]]
dist[1]=[[-4.11133440*10**(-2),  1.62518044, -3.67444770*10**(-3), -5.16801853*10**(-3), -7.97102971]]

mtx[2]=[[1.16887114*10**3, 0.00000000, 6.08676211*10**2],
 [0.00000000, 1.17063923*10**3, 3.59277197*10**2],
 [0.00000000, 0.00000000, 1.00000000]]
dist[2]=[[2.74639413*10**(-1),  -2.39794893, -3.43932065*10**(-3), -3.75716784*10**(-3), 9.83950931]]

mtx=np.float32(mtx[camera])
dist=np.float32(dist[camera])

#for blue filter
lower_blue = np.array([100,150,0])
upper_blue = np.array([140,255,255])

#draw axis for chessbaord
def draw(img, corners, imgpts,d,l):
    n=24
    corner = tuple(corners[n].ravel())
    # img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 4)
    # img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 4)
    # #print("hehe",imgpts[2].ravel())
    # img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 4)
    # img = cv2.line(img, corner, tuple(imgpts[3].ravel()), (255,0,0), 4)
    # img = cv2.line(img, corner, tuple(imgpts[4].ravel()), (0,255,0), 4)

    xvec=[imgpts[0].ravel()-corners[n].ravel(),imgpts[3].ravel()-corners[n].ravel()]
    yvec=[imgpts[1].ravel()-corners[n].ravel(),imgpts[4].ravel()-corners[n].ravel()]
    zvec=imgpts[2].ravel()-corners[n].ravel()
    x=[]
    y=[]
    # print(xvec)
    # print(yvec)
    if (xvec[0][0]>=0):
        x+=[xvec[0]]
    if (xvec[1][0]>=0):
        x+=[xvec[1]]  
    if (yvec[0][0]>=0):
        x+=[yvec[0]]
    if (yvec[1][0]>=0):
        x+=[yvec[1]]

    if (xvec[0][1]>=0):
        y+=[xvec[0]]
    if (xvec[1][1]>=0):
        y+=[xvec[1]]  
    if (yvec[0][1]>=0):
        y+=[yvec[0]]
    if (yvec[1][1]>=0):
        y+=[yvec[1]]
    if len(x)<2 or len(y)<2:
        return img
    axis=[]
    if x[0][0]>x[1][0]:
        xaxis=x[0]
    else:
        xaxis=x[1]
    
    if y[0][1]>y[1][1]:
        yaxis=y[0]
    else:
        yaxis=y[1]
    print(xaxis,yaxis,corners[n][0])
    xaxis+=corners[n][0]
    yaxis+=corners[n][0]
    
    img = cv2.line(img, corner, tuple(xaxis.ravel()), (0,255,255), 4)
    img = cv2.line(img, corner, tuple(yaxis.ravel()), (255,255,0), 4)
    #print("xvec",xvec,"yvec",yvec,"zvec",zvec)

    #below is not functional
    if not l is None:      
        for rho,theta in l[0]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
        
            img=cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
            a1=[x1,y1]-corners[n].ravel()
            a2=[x2,y2]-corners[n].ravel()
                
    return img
    
#taken from the internet
def rotationMatrixToEulerAngles(R) :
 
     
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])*180/math.pi
   
    
axis = np.float32([[3,0,0], [0,3,0], [3,3,-3],[3,6,0],[6,3,0]]).reshape(-1,3)
while (cap.isOpened()):
    ret, img = cap.read()
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    dimensions = img.shape
    
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)#converts to HSV space
    
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    mask2=cv2.GaussianBlur(mask,(5,5),0)
    ret, maskk=cv2.threshold(mask2,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    maskk=cv2.GaussianBlur(maskk,(5,5),0)
    ret, maskk=cv2.threshold(maskk,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    ret, maskk=cv2.threshold(maskk,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

    ret, maskk=cv2.threshold(maskk,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    #apply some filtering techniques
    #mask_gauss=cv2.adaptiveThreshold(mask,255,cv2.ADAPTIVE_THRESH_MEAN,cv2.THRESH_BINARY,11,2)
    img_blue=cv2.bitwise_and(img,img, mask= maskk)

    
    ret, corners = cv2.findChessboardCorners(gray, (7,7),None)
    k = cv2.waitKey(key)    
    #cv2.imshow('img',cv2.resize(img_blue,None,fx=1, fy=1, interpolation = cv2.INTER_CUBIC))

    if ret == True:
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        # Find the rotation and translation vectors.
        hehe, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, dist)

        # project 3D points to image plane
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
        rotmat=np.zeros(3*3)
        rotmat=cv2.Rodrigues(rvecs)
        #print ("\n" * 100)
        #print(rvecs,rotmat)
        #print(rotationMatrixToEulerAngles(rotmat[0])) #this line is commented out because I was testing out the blue filter at the time
        edges = cv2.Canny(maskk,50,150,apertureSize = 3)
        

        lines = cv2.HoughLines(edges,1,np.pi/180,200)
        frame = draw(img,corners2,imgpts,dimensions,lines)
        #out.write(frame)
        cv2.imshow('img',cv2.resize(frame,None,fx=0.8, fy=0.8, interpolation = cv2.INTER_CUBIC))
        #k = cv2.waitKey(key)
        if k==ord('b'):
            cv2.imshow('img_blue',cv2.resize(edges,None,fx=0.8, fy=0.8, interpolation = cv2.INTER_CUBIC))
        
        
    else: 
        cv2.imshow('img',cv2.resize(img,None,fx=0.8, fy=0.8, interpolation = cv2.INTER_CUBIC))
        #k=cv2.waitKey(key)
        #out.write(img)
        if k==ord('b'):
            cv2.imshow('img_blue',cv2.resize(img_blue,None,fx=0.8, fy=0.8, interpolation = cv2.INTER_CUBIC))
        

cap.release()
out.release()
cv2.destroyAllWindows()