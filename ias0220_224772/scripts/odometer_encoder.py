#!/usr/bin/env python
# Software License Agreement (BSD License)

# Simple talker demo that listens to std_msgs/Strings published
# rosrun ias0220_224772 odometer_encoder.py 


import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from ias0220_224772.msg import counter_message

def callback_encoder_listener(data):
    left_wheel_count = data.count_left
    right_wheel_count = data.count_right
    time = rospy.get_time()

    global previous_time
    rate = time - previous_time
    rospy.loginfo('At %s - Listened that data is - %s -- %s. Rate %s', time, left_wheel_count, right_wheel_count, rate)

    global previous_left_counter 
    previous_left_counter = left_wheel_count
    global previous_right_counter 
    previous_right_counter = previous_right_counter
    previous_time = time



def mainloop():
    #rospy.init_node('position_calculator', anonymous=True)
    rospy.Subscriber('encoders_output', counter_message, callback_encoder_listener)  # listen encoder
    rospy.spin()

if __name__ == '__main__':    
    rospy.init_node('position_calculator', anonymous=True)

    while rospy.get_time() == 0.0: #wait init
        time.sleep(1.0)
    previous_time = rospy.get_time() #init time
    previous_left_counter = 0
    previous_right_counter = 0

    mainloop()
