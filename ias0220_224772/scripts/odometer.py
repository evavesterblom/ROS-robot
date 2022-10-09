#!/usr/bin/env python
# Software License Agreement (BSD License)

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose

def callback_nametime(data):
    str_data = data.data
    arr = str_data.split(",")
    rospy.loginfo('Student %s contacted me, and told me that current time is: %s', arr[0], arr[1])
    
def callback_velocity(data):
    rospy.loginfo('I heard velocity %s %s %s', data.x, data.y, data.z)
    
    position_x = (data.x * 0.5) + out.x
    position_y = (data.y * 0.5) + out.y
    
    out.x = position_x
    out.y = position_y
    out.z = 0
    #rospy.loginfo('The new position of the walker is: %s', out) 
    
    point = Pose()
    point.position.x = out.x
    point.position.y = out.y
    point.position.z = 0
    rospy.loginfo('The new position of the walker is: %s', point.position) 
    

def listener():
    rospy.init_node('position_calculator', anonymous=True)

    rospy.Subscriber('name_and_time', String, callback_nametime)
    rospy.Subscriber('velocity', Vector3, callback_velocity)
    rospy.spin()

if __name__ == '__main__':
    out = Vector3()
    listener()
