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
        self.bridge = CvBridge()
        self.rate = rospy.Rate(2)
        self.publisher = rospy.Publisher("/image_raw", Image, queue_size=100)   

    def publish_image(self, counter):
        imageDir = self.dir + 'img_' + f'{counter:04d}' + '.png'
        image = cv2.imread(imageDir)
        imgMsg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        imgMsg.header.frame_id = 'camera'
        imgMsg.header.stamp = rospy.Time.now()
        self.publisher.publish(imgMsg)
        rospy.loginfo(imageDir + ' was sent to /image_raw')  
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
