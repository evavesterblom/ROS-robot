#!/usr/bin/env python3
"""
Publishes encoder output data based on a fake absolute encoder for the two wheels of a differentially steered robot

@author: Simon Godon
@contact: simon.godon@taltech.ee
@creation_date: 25-08-2020
@updated_on: 08-08-2022
"""

import roslib
import rospy
import math
import tf
from numpy import random
from ias0220_224772.msg import counter_message
import sys

class EncodersNode:
    """
    uses /tf to publish the count of the encoder
    publishes topic "encoders_output"
    """

    def __init__(self, noisy):
        self.cpr = 2048.0  # clicks per revolution
        self.listener = tf.TransformListener()
        self.count_publisher = rospy.Publisher(
            "/encoders_output",
            counter_message,
            queue_size=1000)
        self.rate = rospy.Rate(25)  # rate of spinning: 25Hz
        self.noisy = noisy

    def corrector(self, x, y):
        """ Corrects the instability of the transformation from quaternion to Euler angles."""
        if x < 1:
            if y > 0:
                return y
            else:
                return y+360
        if x > 1:
            return 180-y

    def get_count(self, rot):
        """ Takes a quaternion as an input and outputs the absolute encoder count of the rotation between the two frames. """
        quaternion = (rot[0], rot[1], rot[2], rot[3])
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]*180.0/math.pi  # converting rad to degree
        pitch = euler[1]*180.0/math.pi
        interval = 360.0/self.cpr
        # correcting the instability of the conversion quaternion to euler
        count = self.corrector(roll, pitch)/interval
        if self.noisy == True:
            # Add a random gaussian noise
            count = random.normal(count, 25)
        return int(math.modf(count)[1])

    def step(self):
        """
        Performs one iteration of class loop.
        Publishes click count for each wheel based on the transformations between base_link and the wheels.
        """
        try:
            rot_left = self.listener.lookupTransform(
                'base_link', 'left_wheel', rospy.Time(0))[1]
            rot_right = self.listener.lookupTransform(
                'base_link', 'right_wheel', rospy.Time(0))[1]
        except (tf.LookupException, tf.ConnectivityException):
            return
        self.count_publisher.publish(self.get_count(
            rot_left), self.get_count(rot_right))

    def run(self):
        """ Main loop of class. steps node at set intervals, if data from sensors is available."""
        while not rospy.is_shutdown():
            self.step()
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('encoders_node')
    try:
        assert sys.argv[1] == "True" or sys.argv[1] == "False"
        encoders = EncodersNode(sys.argv[1])
        rospy.loginfo("Started encoder node, with noise set to %s", sys.argv[1])
    except:
        rospy.loginfo("Usage: \"encoders_node.py arg\" - No arg was given for encoders noise, defaulted to True")
        encoders = EncodersNode(True)
    encoders.run()
