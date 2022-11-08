#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

class ObjectRecognizer():
    def __init__(self):
        rospy.Subscriber("/observerbot/camera1/image_raw", Image, self.img_raw_callback, queue_size=10)
        self.publishAllContours = rospy.Publisher("/image_all_contours", Image, queue_size=100)  
        self.publishBiggestContour = rospy.Publisher("/image_biggest_contour", Image, queue_size=100)
        self.publishRobotTracker = rospy.Publisher("/image_track_robot", Image, queue_size=100)    
        self.rate = rospy.Rate(2)
        self.br = CvBridge()

    def img_raw_callback(self, data):
        img_bgr = self.br.imgmsg_to_cv2(data)

        allContoursImage = img_bgr.copy()
        biggestContourImage = img_bgr.copy()

        hsv_img = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

        edgedImage = cv2.Canny(gray, 30, 200) #around 60 contours
        contoursAll, _ = cv2.findContours(edgedImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
        
        #all contours
        cv2.drawContours(allContoursImage, contoursAll, -1, (0, 255, 0), 3)
        allContoursImg = self.br.cv2_to_imgmsg(allContoursImage, "rgb8")
        self.publishAllContours.publish(allContoursImg) 

        #biggest contour
        c = max(contoursAll, key = cv2.contourArea)
        cv2.drawContours(biggestContourImage, c, -1, (255,0,0),10)
        biggestContourImg = self.br.cv2_to_imgmsg(biggestContourImage, "rgb8")
        self.publishBiggestContour.publish(biggestContourImg)

        #detect robot
        lowest = np.array([120,50,50])
        upper = np.array([124,255,255])
        mask = cv2.inRange(hsv_img, lowest, upper)
        res = cv2.bitwise_and(img_bgr,img_bgr, mask= mask)
        contoursRobot, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 
        cMaxRobot = max(contoursRobot, key = cv2.contourArea)
       
        #draw contour around robot - was not sure if that counted, so I added rectangle draw 
        #cv2.drawContours(img_bgr, cMaxRobot, -1, (255,0,0), 10)
   
        #draw rectangle around robot
        (x,y,w,h) = cv2.boundingRect(cMaxRobot)
        cv2.rectangle(img_bgr,(x,y),(x+w,y+h),(255,55,66), 3)
        trackingRobotImg = self.br.cv2_to_imgmsg(img_bgr, "rgb8")
        self.publishRobotTracker.publish(trackingRobotImg)

        rospy.loginfo('Listening to /image_raw + detecting moving robot')  


if __name__ == '__main__':
    rospy.init_node('object_recognition')
    ob = ObjectRecognizer()
    rospy.spin()
