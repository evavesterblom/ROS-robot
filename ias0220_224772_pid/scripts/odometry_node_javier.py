#!/usr/bin/env python3
"""
Node calculates odometry based on absolute encoder values for a defined differential drive robot

@authors: Andreas Nagel, Simon Godon
@creation_date: 28-08-2020
@updated_on: 08-08-2022
"""

import roslib
import rospy
import math
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_multiply
from ias0220_224772_pid.msg import counter_message
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

def quat_msg_to_tf(q):
    """
    translate from quaternion message to tf quaternion.
    """
    return [q.x, q.y, q.z, q.w]

class OdometryNode():
    """
    Calculates odometry based on absolute encoder values for a defined differential drive robot
    """

    def __init__(self):
        self.cpr = 2048.0 	# clicks per revolution
        self.odom_msg = Odometry()
        self.count_sub = rospy.Subscriber("/encoders_output", counter_message, callback=self.counter_callback, queue_size=1)
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1000)
        self.prev_time = rospy.Time(0)
        self.time = rospy.Time(0)
        self.wheel_radius = 0.04
        self.wheel_base_size = 0.2
        self.left_enc = 0
        self.right_enc = 0
        self.odom_msg.child_frame_id = "base_link"
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.header.stamp = rospy.Time(0)
        self.odom_msg.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))
        
        # Radians per one tick
        self.one_tick_rad = 2 * math.pi / self.cpr
        # Rate of spinning: 25Hz
        self.rate = rospy.Rate(25)

    def normalize_enc_delta(self, enc_delta):
        """
        Normalizes encoder difference values, so that going from max_value -> 0 still gives correct rotation
        @param enc_delta : int
        @return : int
        """
        if abs(enc_delta) < self.cpr / 2:
            return enc_delta
        elif enc_delta > 0:
            return enc_delta - self.cpr
        else:
            return enc_delta + self.cpr
            

    def counter_callback(self, count):
        """
        Calculates odometry values on encoder update for a differential drive robot.
        @param count : [count_left, count_right]
        @return : int
        """
        if self.odom_msg.header.stamp.to_sec() > 0:
            delta_time = rospy.Time.now() - self.odom_msg.header.stamp
            
            left_delta = self.normalize_enc_delta(count.count_left - self.left_enc)
            right_delta = self.normalize_enc_delta(count.count_right - self.right_enc)

            # calculate distance travelled by each wheel in metres
            left_dist = left_delta * self.one_tick_rad * self.wheel_radius
            right_dist = -right_delta * self.one_tick_rad * self.wheel_radius # we put a "-" here because the encoder for right wheel turns in the opposite direction

            #calculate rotation angle and distance travelled (in robot frame)
            x_distance_travelled = (right_dist + left_dist) / 2
            rot_angle = (right_dist - left_dist) / self.wheel_base_size
            
            # update linear and angular velocity values
            try:
                self.odom_msg.twist.twist.linear.x = x_distance_travelled / delta_time.to_sec()
                self.odom_msg.twist.twist.angular.z = rot_angle / delta_time.to_sec()
            except:
                rospy.loginfo("Received only one message, cannot compute delta_time")
            # update orientation value of robot
            q_rot = quaternion_from_euler(0, 0, rot_angle)
            self.odom_msg.pose.pose.orientation = Quaternion(*quaternion_multiply(q_rot, quat_msg_to_tf(self.odom_msg.pose.pose.orientation)))

            # update position value of the robot
            _, _, new_yaw = euler_from_quaternion(quat_msg_to_tf(self.odom_msg.pose.pose.orientation))
            mean_heading_last_two = new_yaw - (rot_angle / 2) # We take the mean heading between the last orientation and the current one to reduce approximation errors
            self.odom_msg.pose.pose.position.x += x_distance_travelled * math.cos(mean_heading_last_two) # We convert to displacements in the odom frame and add to the previously computed position.
            self.odom_msg.pose.pose.position.y += x_distance_travelled * math.sin(mean_heading_last_two) 

        # save encoder values and update odometry message timestamp
        self.left_enc = count.count_left
        self.right_enc = count.count_right
        self.odom_msg.header.stamp = rospy.Time.now()

    def step(self):
        """
        Publishes odometry information if we received at least one message
        """
        try:
            if self.odom_msg.header.stamp.to_sec() > 0:
                self.odom_pub.publish(self.odom_msg)
        except Exception:
            rospy.logerr("Could not publish odometry")

    def run(self):
        """ Main loop of class. Runs at a set rate."""
        while not rospy.is_shutdown():
            self.step()
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('odometry_node')
    odometry = OdometryNode()
    odometry.run()
