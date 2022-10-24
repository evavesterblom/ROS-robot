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

def calculate_ticks_since(previous, current):
    treshold = 300

    if abs(current - previous) < treshold:
        ticks_since = current - previous
    else:
        ticks_since = 2048 - previous + current

    return ticks_since

def callback_encoder_listener(data):
    #read encoder
    current_left_counter = data.count_left
    current_right_counter = data.count_right
    time = rospy.get_time()
    global previous_time
    time_since = time - previous_time

    global previous_left_counter
    global previous_right_counter 
    left_since = calculate_ticks_since(previous_left_counter, current_left_counter)
    right_since = calculate_ticks_since(previous_right_counter, current_right_counter) * -1 #reversed movement

    #rospy.loginfo('%s', current_left_counter)
    #rospy.loginfo('%s', current_right_counter)
    #rospy.loginfo('Left data %s   prev %s     current %s   ticks %s', time_since, previous_left_counter, current_left_counter, left_since)
    rospy.loginfo('R data %s   prev %s     current %s   ticks %s', time_since, previous_right_counter, current_right_counter, right_since)


    #current is next interation previous
    previous_left_counter = current_left_counter
    previous_right_counter = current_right_counter
    previous_time = time



def mainloop():
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
