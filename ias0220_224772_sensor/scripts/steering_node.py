#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int16
import numpy as np
import tf_conversions
import math

class SteeringNode():
    def __init__(self):
        rospy.Subscriber("/imu", Imu, self.calculate_vel_callback, queue_size = 200)
        rospy.Subscriber("/distance", Int16, self.set_distance_callback, queue_size = 200)
        self.pub_twist = rospy.Publisher('/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        self.rate = rospy.Rate(30) 
        self.distance = 0

    def set_distance_callback(self, data):
        self.distance = data.data

    def calculate_vel_callback(self, data):
        quaternion = [data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w]
        roll, pitch, _ = tf_conversions.transformations.euler_from_quaternion(quaternion)
        pitch_deg = math.degrees(pitch)
        roll_deg = math.degrees(roll)

        velocity_lin = self.calculate_speed(pitch_deg)
        linear_velocity = Vector3(velocity_lin,0,0)
        velocity_ang = self.calculate_speed(roll_deg)
        angular_velocity = Vector3(0,0,-velocity_ang)

        self.twist.linear = linear_velocity
        self.twist.angular = angular_velocity

        self.pub_twist.publish(self.twist)
        self.rate.sleep()

    # If distance 0-20          - max speed is 1
    # If distance 20-50         - max speed is 2
    # If distance -1 or 50+     - robot shoud not move
    def calculate_speed(self, deg):
        if (self.distance >= 0 & self.distance <= 20): 
            return deg / 90
        elif (self.distance > 20 & self.distance <= 50):
            return deg / 45
        else:
            return 0

if __name__ == '__main__':
    rospy.init_node('steering_node')
    ob = SteeringNode()
    rospy.spin()

