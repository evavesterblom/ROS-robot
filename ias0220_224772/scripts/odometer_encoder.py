#!/usr/bin/env python
# Software License Agreement (BSD License)

# Simple talker demo that listens to std_msgs/Strings published
# rosrun ias0220_224772 odometer_encoder.py 


import rospy
import time
import math
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from ias0220_224772.msg import counter_message

def calculate_angular_velocity(ticks, time):
    return ((ticks / 2048) * 2 * math.pi) / time

def calculate_linear_velocity(omega, radius):
    return omega * radius


def calculate_ticks_since(previous, current):
    treshold = 500
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

    #ticks
    global previous_left_counter
    global previous_right_counter 
    left_since = calculate_ticks_since(previous_left_counter, current_left_counter)
    right_since = calculate_ticks_since(current_right_counter, previous_right_counter) #reversed movement

    #angular velocity
    left_angual_velocity = calculate_angular_velocity(left_since, time_since)
    right_angual_velocity = calculate_angular_velocity(right_since, time_since)

    #linear velocity
    left_linear_velocity = calculate_linear_velocity(left_angual_velocity, 0.04)
    right_linear_velocity = calculate_linear_velocity(right_angual_velocity, 0.04)

    #linear velocity robot Vp
    robot_linear_velocity = (left_linear_velocity + right_linear_velocity) / 2

    #rotational velocity robot wz
    robot_anglar_velocity = (right_linear_velocity - left_linear_velocity) / 0.08

    #fill odom msg
    global odom_msg
    odom_msg.header.stamp = time
    odom_msg.header.seq +=1
    #TODO odom_msg.twist ... x - robot_linear_velocity and ... z - robot_anglar_velocity
    #TODO odom.msg.pose ... integrate velocities to calculate pose in odometry message. You need to convert between quaternions and euler angles.





    #rospy.loginfo('%s - %s', left_angual_velocity, right_angual_velocity)
    #rospy.loginfo('%s', current_right_counter)
    #rospy.loginfo('Left data %s   prev %s     current %s   ticks %s', time_since, previous_left_counter, current_left_counter, left_since)
    #rospy.loginfo('L data %s   prev %s     current %s   ticks %s', time_since, previous_left_counter, current_left_counter, left_since)
    rospy.loginfo('%s : %s', left_linear_velocity, right_linear_velocity)


    #iteration history
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

    #init odom msg
    odom_msg = Odometry()
    odom_msg.header.seq = 1
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_link"
    odom_msg.pose = 0
    odom_msg.twist = 0


    mainloop()
