import numpy as np
import cv2
import glob
import os 
import math
##modify line 7 for source, enter 0 if directly taken from camera
filename='VID_20200214_152950.mp4'
##modily camera 0 for my Huawei phone, 1 for ArduCam, 2 for Microsoft Webcam
camera=2
##modify key 0 for keyboard updated frame, key 1 for non interrupted video output
key=0
## press any key to advance a frame, press q to quit
savefile=[]
cap = cv2.VideoCapture(filename)
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


def angle(a,b):
    if a[0]==a[1]==0 or b[0]==b[1]==0:
        return False
        
    angle=math.acos(np.inner(a,b)/math.sqrt((np.inner(a,a)*np.inner(b,b))))*180/math.pi
    sign=np.cross(a,b)
    
    if angle>90:
        angle= 180-angle
        angle = -sign/abs(sign)*angle
    elif angle<90:
        angle = sign/abs(sign)*angle
    return angle
    

    
def xy(x,y):
    if (abs(x[1])>abs(x[0])):
        return [y,x]
    else:
        return [x,y]
    
 
def draw(img, corners, imgpts,d):
    n=24
    corner = tuple(corners[n].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 4)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 4)
    #print("hehe",imgpts[2].ravel())
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 4)
    img = cv2.line(img, corner, tuple([round(d[1]/2),round(d[0]/2)]), (0,255,255), 4)

    xvec=imgpts[0].ravel()-corners[n].ravel()
    yvec=imgpts[1].ravel()-corners[n].ravel()
    xvec,yvec=xy(xvec,yvec)
    

    zvec=imgpts[2].ravel()-corners[n].ravel()
    #anglee=anglenew(zvec)
    x=np.float32([1,0])
    y=np.float32([0,1])
    angle1=angle(xvec,zvec)
    angle2=angle(yvec,zvec)
    #print(angle1,angle2)
    return img
    


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
   
    
axis = np.float32([[3,0,0], [0,3,0], [3,3,-3],[3,1,0]]).reshape(-1,3)
while (cap.isOpened()):
    ret, img = cap.read()
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (7,7),None)
    dimensions = img.shape
    if ret == True:
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        # Find the rotation and translation vectors.
        hehe, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, dist)

        # project 3D points to image plane
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
        rotmat=np.zeros(3*3)
        rotmat=cv2.Rodrigues(rvecs)
        print ("\n" * 100)
        #print(rvecs,rotmat)
        print(rotationMatrixToEulerAngles(rotmat[0]))
    
        frame = draw(img,corners2,imgpts,dimensions)
        #out.write(frame)
        cv2.imshow('img',cv2.resize(frame,None,fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC))
        k = cv2.waitKey(key)
        if k==ord('q'):
            break
        
    else: 
        cv2.imshow('img',cv2.resize(img,None,fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC))
        k=cv2.waitKey(key)
        #out.write(img)
        if k==ord('q'):
            break
        

cap.release()
out.release()
cv2.destroyAllWindows()