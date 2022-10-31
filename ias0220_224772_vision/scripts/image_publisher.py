#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImagePublisher():
    def __init__(self):
        self.fileDir = os.path.dirname(os.path.abspath(__file__))
        self.dir = os.path.abspath(os.path.join(self.fileDir, os.pardir)) + '/data/images/'
        self.imageDir = self.dir
        self.image = Image()
        self.bridge = CvBridge()
        self.imgMsg = None
        self.rate = rospy.Rate(2)
        self.publisher = rospy.Publisher("/image_raw", Image, queue_size=100)   

    def publish_image(self, counter):
        self.imageDir = self.dir + 'img_' + f'{counter:04d}' + '.png'
        self.image = cv2.imread(self.imageDir)
        self.imgMsg = self.bridge.cv2_to_imgmsg(self.image, "bgr8")
        self.imgMsg.header.frame_id = 'camera'
        self.publisher.publish(self.imgMsg)
        rospy.loginfo(self.imageDir + ' was sent to /image_raw')   
        pass

    def run(self):
        while not rospy.is_shutdown():
            for x in range (1, 38):
                self.publish_image(x)
                self.rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('image_publisher')
        publisher = ImagePublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
