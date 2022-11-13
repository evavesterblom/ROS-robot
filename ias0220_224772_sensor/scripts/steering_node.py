#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Int16
import tf_conversions
import math

class SteeringNode():
    def __init__(self):
        rospy.Subscriber("/imu", Imu, self.calculate_vel_callback, queue_size = 200)
        rospy.Subscriber("/distance", Int16, self.set_distance_callback, queue_size = 200)
        self.pub_twist = rospy.Publisher('/diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        self.rate = rospy.Rate(30) 
        self.distance = Int16()

    def set_distance_callback(self, data):
        self.distance = data

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

        rospy.loginfo('Info %s %s', pitch_deg, self.distance)

        self.pub_twist.publish(self.twist)
        self.rate.sleep()
    
    def calculate_speed(self, deg):
        #[0>0 - 90>2]
        return deg / 45

if __name__ == '__main__':
    rospy.init_node('steering_node')
    ob = SteeringNode()
    rospy.spin()


#Add here the distance checking. 
# If distance 0-20 - max speed is 1
# If distance 20 - X - max speed is 2