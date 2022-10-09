#!/usr/bin/env python
# Software License Agreement (BSD License)

# Simple talker demo that listens to std_msgs/Strings published
# to the 'chatter' topic


import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker

counter = 0

def callback_nametime(data):
    str_data = data.data
    arr = str_data.split(",")
    rospy.loginfo(
        'Student %s contacted me, and told me that current time is: %s', arr[0], arr[1])


def callback_velocity(data):
    rospy.loginfo('I heard velocity %s %s %s', data.x, data.y, data.z)

    # calc the new position (out) and print as Pose.position
    position_x = (data.x * 0.5) + out.x
    position_y = (data.y * 0.5) + out.y

    out.x = position_x
    out.y = position_y
    out.z = 0

    p = Pose()
    p.position.x = out.x
    p.position.y = out.y
    p.position.z = 0
    rospy.loginfo('The new position of the walker is: %s', p.position)
    publish_marker(p)


def publish_marker(pse):
    m.header.frame_id = "map"
    m.type = m.SPHERE
    m.action = m.ADD
    m.scale.x = 0.5
    m.scale.y = 0.5
    m.scale.z = 0.5
    m.color.a = 0.2
    m.color.r = 1.0
    m.color.g = 0.0
    m.color.b = 0.0
    m.lifetime = rospy.Duration()
    m.pose = pse
    m.pose.orientation.x = 0.0
    m.pose.orientation.y = 0.0
    m.pose.orientation.z = 0.0
    m.pose.orientation.w = 1.0
    global counter
    m.id = counter
    counter += 1 

    #pub_marker.publish(m) #publish marker to topic /walker_path
    #rospy.loginfo('Publishing to walker_path %s', counter)
    publish_path()


def listener():
    rospy.init_node('position_calculator', anonymous=True)

    rospy.Subscriber('name_and_time', String, callback_nametime)  # listen name
    rospy.Subscriber('velocity', Vector3, callback_velocity)  # listen velocity

    rospy.spin()

def publish_path():
    pub_marker.publish(m)
    rospy.loginfo('Publishing to walker_path %s', counter)

if __name__ == '__main__':
    out = Vector3() #vector init
    
    pub_marker = rospy.Publisher('walker_path', Marker, queue_size=10)
    m = Marker() #marker init
    
    listener() #loop
