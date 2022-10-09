#!/usr/bin/env python
# Software License Agreement (BSD License)

import rospy
import random
from geometry_msgs.msg import Vector3
from std_msgs.msg import String

def walker():
    pub_velocity = rospy.Publisher('velocity', Vector3, queue_size=10)
    pub_name = rospy.Publisher('name_and_time', String, queue_size=10)
    rospy.init_node('walker', anonymous=True)
    rate = rospy.Rate(1)  ##2
    while not rospy.is_shutdown():
        test_list = [1, 0, -1]
        out = Vector3()
        out.x = random.choice(test_list)
        out.y = random.choice(test_list)
        out.z = 0
        
        name_str = "224772,%s" % rospy.get_time()
        
        rospy.loginfo(out)
        pub_velocity.publish(out)
        
        rospy.loginfo(name_str)   
        pub_name.publish(name_str)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        walker()
    except rospy.ROSInterruptException:
        pass
