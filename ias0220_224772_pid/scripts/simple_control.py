#!/usr/bin/env python3

"""
Template for home assignment 7 (Robot Control).
Node to take a set of waypoints and to drive a differential
drive robot through those waypoints using a simple PD controller
and provided odometry data.

Students should complete the code. Note the places marked with "# TODO".
The skeleton and the TODO here are just helpers, not guidelines.
You are also allowed to create more methods, or delete some. 

@author: ias0220 teachers, Evgenia Vesterblom student
@date: August 2022/November 2022
@input: Odometry as nav_msgs Odometry message
@output: body velocity commands as geometry_msgs Twist message;
         list of waypoints as MarkerArray message
"""

import math
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Point, Vector3
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import tf_conversions


class PDController:
    def __init__(self):

        """Get the static parameters from the parameter server (which have
         been loaded onto the server from the yaml file using the launch file).
         The namespace defined in the launch file must match the namespace
          used here (i.e., "controller_waypoints")"""
        self.Kd = rospy.get_param("/controller_waypoints/controller/Kd")
        self.Kp = rospy.get_param("/controller_waypoints/controller/Kp")
        self.distance_margin = rospy.get_param("/controller_waypoints/mission/distance_margin")
        self.waypoints = rospy.get_param("/controller_waypoints/mission/waypoints")

        # Print the parameters
        rospy.loginfo("Got static data from parameter server:")
        rospy.loginfo(" Mission parameters:")
        rospy.loginfo("  Distance margin: %.2f", self.distance_margin)
        rospy.loginfo("  Waypoints (#: x|y):")
        wp = 1
        for waypoint in self.waypoints:
            rospy.loginfo("   %d: %.1f|%.1f", wp, waypoint[0], waypoint[1])
            wp += 1
        rospy.loginfo(" Controller parameters:")
        rospy.loginfo("  Proportional gains: %.2f, %.2f",
                      self.Kp[0],
                      self.Kp[1])
        rospy.loginfo("  Derivative gains  : %.2f, %.2f",
                      self.Kd[0],
                      self.Kd[1])
        rospy.loginfo("-----------------------------------------------------")

        # Initialization of class variables
        self.term = 0

        self.wpIndex = 0          # counts the visited waypoints
        self.position = Point()   # current position (3D vector: .x, .y, .z)
        self.heading = 0.0        # current orientation of robot (yaw [rad])
        self.time = rospy.Time.now().to_sec()

        self.delta_distance = 0.0
        self.delta_angle = 0.0
        self.delta_lin_velocity = 0.0
        self.delta_ang_velocity = 0.0

        self.done = False
        self.init = True
        self.vel_cmd = [0.0, 0.0]  # calculated velocities (linear, angular)

        # Publishers and subscribers
        self.publisher_cmd_vel = rospy.Publisher("/controller_diffdrive/cmd_vel",Twist, queue_size=10)
        self.publisher_waypoints = rospy.Publisher("/mission_control/waypoints",MarkerArray,queue_size=10)
        rospy.Subscriber('odom', Odometry, self.onOdom)
        
        # Messages
        self.marker_array = None
        self.twist = Twist()
        
        # Registering start time of this node
        self.startTime = 0
        while self.startTime == 0:
            self.startTime = rospy.Time.now().to_sec()

    def wrapAngle(self, angle):
        """
        Helper function that returns angle wrapped between +- Pi.
        Hint: Pass your error in heading [rad] into this function, and it
        returns the shorter angle. This prevents your robot from turning
        along the wider angle and makes it turn along the smaller angle (but
        in opposite direction) instead.
        @param: self
        @param: angle - angle to be wrapped in [rad]
        @result: returns wrapped angle -Pi <= angle <= Pi
        """
        #TODO: return the angle between -PI and PI
        if (abs(angle) > math.pi):
            a = 2*math.pi - abs(angle)
            return -a
        else:
            return angle
        
    def run(self):
        """
        Main loop of class.
        Since we are using a callback function (onOdom) to trigger our
        computations and outputs, we don't need a main loop here. But then we
        must ensure the node does not terminate.
        @param: self
        @result: runs the step function for controller update
        """
        # spin() simply keeps python from exiting until this node is stopped
        rospy.loginfo("Waiting for odom message...")
        rospy.spin()

    def setNextWaypoint(self):
        """
        Removes current waypoint from list and sets next one as current target.
        @param: self
        @result: returns True if the next waypoint exists and has been set,
                 otherwise False
        """
        if not self.waypoints:
            return False
        self.waypoints.pop(0)
        if not self.waypoints:
            return False
        self.wpIndex += 1
        rospy.loginfo("----------------------------------------------")
        rospy.loginfo("                Next waypoint                 ")
        rospy.loginfo("----------------------------------------------")
        return True

    def isWaypointReached(self):
        """
        Checks if waypoint is reached based on pre-defined threshold.
        @param: self
        @result: returns True if waypoint is reached, otherwise False
        """
        # TODO: Check if within distance_margin from waypoint
        return False

    def controller(self):
        """
        Takes the errors and calculates velocities from it, according to
        control algorithm specs.
        command = Kp e + Kd edot
        e = Vector2(distance err, angle err)
        ed = Vector2(vel change, angle change)
        @param: self
        @result: sets the values in self.vel_cmd
        """
        # Output 0 (skip all calculations) if the last waypoint was reached
        # TODO: Your code here
        e =     [self.Kp[0] * self.delta_distance,      self.Kp[1] * self.delta_angle]
        e_dot = [self.Kd[0] * self.delta_lin_velocity,  self.Kd[1] * self.delta_ang_velocity]
        self.vel_cmd = [e[0] + e_dot[0], e[1] + e_dot[1]] 
        

    def publish_vel_cmd(self):
        """
        Publishes command velocities computed by the control algorithm.
        @param: self
        @result: publish message
        """
        # TODO: Your code here
        self.term += 1
       

        #if ((self.term%100) == 1):
        #    self.twist.linear = Vector3(self.vel_cmd[0], 0, 0)
        #    self.twist.angular =  Vector3(0, 0, self.vel_cmd[1])
        #else:  
        #    self.vel_cmd[0] = 0
        #    self.vel_cmd[1] = 0
        #   self.twist.linear = Vector3(self.vel_cmd[0], 0, 0)
        #    self.twist.angular =  Vector3(0, 0, self.vel_cmd[1])

        self.twist.linear = Vector3(self.vel_cmd[0], 0, 0)
        self.twist.angular =  Vector3(0, 0, self.vel_cmd[1])
        self.publisher_cmd_vel.publish(self.twist)
        self.time = rospy.Time.now().to_sec()
        
 


    def publish_waypoints(self):
        """
        Publishes the list of waypoints, so RViz can see them.
        @param: self
        @result: publish message
        """
        self.marker_array = MarkerArray()
        id = 0
        for waypoint in self.waypoints:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = waypoint[0]
            marker.pose.position.y = waypoint[1]
            marker.pose.position.z = 0.05
            marker.id = id
            id += 1
            self.marker_array.markers.append(marker)
        self.publisher_waypoints.publish(self.marker_array)

    def onOdom(self, odom_msg):
        """
        Callback function, handling incoming odometry messages.
        @param: self
        @param odom_msg - odometry geometry message
        @result: update of relevant vehicle state variables
        """
        # Store odometry data
        # TODO: Your code here
        # rospy.loginfo("This is odom msg %s", odom_msg)
        prev_time = self.time
        current_time = rospy.Time.now().to_sec()
        delta_time = current_time - prev_time
        

        goal = self.waypoints[0]
        rospy.loginfo("goal is: %s",  goal)
        prev_prosition = self.position
        prev_heading = self.heading
        
        self.position = odom_msg.pose.pose.position
        q = odom_msg.pose.pose.orientation
        quaternion = [q.x,q.y,q.z,q.w]
        _, _, self.heading = tf_conversions.transformations.euler_from_quaternion(quaternion)
        curr_position = self.position
        rospy.loginfo("curr is: %s,%s",  curr_position.x, curr_position.y)
        curr_heading = self.heading

        #1
        delta_x = (goal[0] - curr_position.x)
        delta_y = (goal[1] - curr_position.y)
        self.delta_distance = math.sqrt( pow(delta_x, 2) + pow(delta_y, 2) )

        #2
        atan =  math.atan2(delta_y, delta_x)
        angle = atan - curr_heading
        #self.delta_angle = self.wrapAngle(angle)
        self.delta_angle = angle

        #3
        delta_x_prev = (curr_position.x - prev_prosition.x)
        delta_y_prev = (curr_position.y - prev_prosition.y)
        delta_distance_prev = math.sqrt( pow(delta_x_prev, 2) + pow(delta_y_prev, 2) )
        self.delta_lin_velocity = (delta_distance_prev) / 0.028

        #4
        self.delta_ang_velocity = (curr_heading - prev_heading) / 0.028
        
        #yaw_deg = math.degrees(self.heading)
        rospy.loginfo("Heading is: %s",  math.degrees(curr_heading))
        rospy.loginfo("Goal is: %s",  math.degrees(atan))
        rospy.loginfo("delta goan is: %s",   math.degrees(self.delta_angle))
        rospy.loginfo("----")

        # Check if current target reached;
        #  set next one if necessary and possible
        if self.isWaypointReached():
            if not self.setNextWaypoint():
                if not self.done:
                    rospy.loginfo("This was the last waypoint in the list.")
                    endTime = rospy.Time.now().to_sec()
                    rospy.loginfo("Started node  [s]: %.2f", self.startTime)
                    rospy.loginfo("Finished node [s]: %.2f", endTime)
                    totalTime = endTime - self.startTime
                    rospy.loginfo("Elapsed time  [s]: %.2f", totalTime)
                    self.done = True

        # Apply PD algorithm
        self.controller()

        # Publish velocity commands
        self.publish_vel_cmd()

        # Publish waypoint list
        self.publish_waypoints()


if __name__ == '__main__':
    rospy.init_node("Planner")
    controller = PDController()
    controller.run()