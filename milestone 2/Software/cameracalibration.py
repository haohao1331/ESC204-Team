import numpy as np
import cv2
import glob
import os
# this file is partially copied from https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_tutorials.html
n=7
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((7*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:7].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

#images = glob.glob('*.jpg')
i=0
savefile=[]
cap = cv2.VideoCapture(0)

fourcc = cv2.VideoWriter_fourcc(*'MP42')
out = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))

while(cap.isOpened()):
    ret1, img = cap.read()
    print('ret1',ret1)
    if ret1==True:
        if len(savefile)<=i:
            savefile=savefile+[('hehe'+str(i)+'.jpg')]
        
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (7,7),None)
        print('findchessboard',ret)
        # If found, add object points, image points (after refining them)
        if ret == True:
    
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
    
            # Draw and display the corners
            frame = cv2.drawChessboardCorners(img, (7,7), corners2,ret)
            cv2.imshow('img',frame)
            out.write(frame)
            k=cv2.waitKey(1)
            if k==ord('s'):
                cv2.imwrite(savefile[i],img)
                objpoints.append(objp)
                imgpoints.append(corners2)

                i+=1
            elif k==ord('q'):
                break
        else:
            print('here')
            cv2.imshow('img',img)
            k=cv2.waitKey(1)
            out.write(img)
            if k==ord('q'):
                break

    else:
        break
    print('hehehehehehehe')

if objpoints!=[]:
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
    print(mtx,dist)#print out to save for later
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    
    #######################分割线##################################################
    #this section is on axis drawing, demonstrating that the previous callibration was correct.
    i=0
    savefile=[]
    cap = cv2.VideoCapture(0)
    
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('output1.avi',fourcc, 20.0, (640,480))
    
    
    def draw(img, corners, imgpts):
        corner = tuple(corners[0].ravel())
        img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
        img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
        img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
        return img
        
    axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
    while (cap.isOpened()):
        ret1, img = cap.read()
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (7,7),None)
    
        if ret == True:
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
    
            # Find the rotation and translation vectors.
            hehe, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, dist)
    
            # project 3D points to image plane
            imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
    
            frame = draw(img,corners2,imgpts)
            cv2.imshow('img',frame)
            k = cv2.waitKey(1)
            out.write(frame)
            if k==ord('q'):
                break
            
        else: 
            print('here')
            cv2.imshow('img',img)
            k=cv2.waitKey(1)
            out.write(img)
            if k==ord('q'):
                break
            

cap.release()
out.release()
cv2.destroyAllWindows()
# for fname in images:
#     savefile=savefile+[('hehe'+str(i)+'.jpg')]
#     
#     img = cv2.imread(fname)
#     gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
# 
#     # Find the chess board corners
#     ret, corners = cv2.findChessboardCorners(gray, (7,7),None)
#     print(ret)
#     # If found, add object points, image points (after refining them)
#     if ret == True:
#         objpoints.append(objp)
# 
#         corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
#         imgpoints.append(corners2)
# 
#         # Draw and display the corners
#         img = cv2.drawChessboardCorners(img, (7,7), corners2,ret)
#         cv2.imshow('img',img)
#         cv2.waitKey(0)
#         cv2.imwrite(savefile[i],img)
#     i+=1
#     print(objpoints)


