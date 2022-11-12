#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import tf_conversions

class SteeringNode():
    def __init__(self):
        rospy.Subscriber("/imu", Imu, self.imu_callback, queue_size = 200)

    def imu_callback(self, data):
        #convert imu angles from quat to rpy
        quaternion = [data.orientation.x,
                        data.orientation.y,
                        data.orientation.z,
                        data.orientation.w]
        roll, pitch, yaw = tf_conversions.transformations.euler_from_quaternion(quaternion)

        #Pitch linear velocity along x axis

        #Roll angular velocity around z axis

        #Write linear adn angular velocities into Twist and publish to .../cmd_vel
        
        rospy.loginfo('Pitch %s', pitch)  

if __name__ == '__main__':
    rospy.init_node('steering_node')
    ob = SteeringNode()
    rospy.spin()
