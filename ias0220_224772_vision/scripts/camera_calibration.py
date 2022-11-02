#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque

def run(data):
    global state
    global counter
    global image_list
    global d
    global dist
    global mtx
    #COLLECT
    if state == 'Collect':
        counter += 1
        br =  CvBridge()
        msg = br.imgmsg_to_cv2(data)
        (rows,cols,channels) = msg.shape
        d.append(msg)
        rospy.loginfo('Collect /image_raw and save image %s', counter)
        if counter == 20:
            counter = 0
            state = 'Calibrate'

    #CALIBRATE
    if state == 'Calibrate':
        squareSize = 0.108
        objp = np.zeros((6*7,3), np.float32)
        objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2) * squareSize
        
        objpoints = [] # 3d point in real world space Arrays to store object points and image points from all the images.
        imgpoints = [] # 2d points in image plane.
       
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)  # termination criteria
        for img in d:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # convert image from color to gray scale
            ret, corners = cv2.findChessboardCorners(gray, (7,6), None) # Find the chess board corners using corner detection algorithm

            if ret == True: # If found, add object points, image points (after refining them)
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
                imgpoints.append(corners2)
                img = cv2.drawChessboardCorners(img, (7,6), corners2, ret)
                pubImg = br.cv2_to_imgmsg(img, 'bgr8')
                pubCorners.publish(pubImg)
                rospy.loginfo('Calibrate cornersimage to /image_processed')
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        tot_error=0
        total_points=0
        mean_error = 0
        for i in range(len(objpoints)):
            reprojected_points, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            tot_error+=np.sum(np.abs(imgpoints[i]-reprojected_points)**2)
            total_points+=len(objpoints[i])
        mean_error=np.sqrt(tot_error/total_points)
        rospy.loginfo('Mean error  %s', mean_error)
        state = 'Publish'
        d.clear()

    #PUBLISH CAMERA INFO
    if state == 'Publish':
        global ci
        ci.header.stamp = rospy.Time.now()
        ci.header.frame_id = 'camera'
        ci.width = 1920
        ci.height = 1080
        ci.distortion_model = 'plumb_bob'
        ci.D = dist.flatten()
        ci.K = mtx.flatten()
        ci.R = np.eye(3).flatten()
    
        zeros = np.zeros((3,1), dtype=float)
        P = np.append(mtx, zeros, axis=1)
        ci.P = P.flatten()

        pubCamera.publish(ci)
        rospy.loginfo('Publish camera info to /camera_info')
        state = 'Loop'
        rospy.loginfo('')

    #MAIN LOOP PUBLISHING CAMERA INFO
    if state == 'Loop':
        ci.header = data.header
        pubCamera.publish(ci)
        rospy.loginfo('Publish camera info to /camera_info LOOP')
        global rate
        rate.sleep()

def listen():
    rospy.Subscriber("/image_raw", Image, run, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    counter = 0
    image_list = []
    d = deque()
    state = 'Collect'
    rospy.init_node('camera_calibration')
    pubCorners = rospy.Publisher("/image_processed", Image, queue_size=100) 
    pubCamera = rospy.Publisher("/camera_info", CameraInfo, queue_size=100)
    global dist
    global mtx
    global ci
    ci = CameraInfo()
    rate = rospy.Rate(2)
    listen()



    