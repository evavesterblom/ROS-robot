#!/usr/bin/env python3

"""
Template for home assignment 7 (Robot Control).
Node to take a set of waypoints and to drive a differential
drive robot through those waypoints using a simple PD controller
and provided odometry data.

@author: ias0220 teachers, Evgenia Vesterblom student
@date: August 2022/November 2022
@input: Odometry as nav_msgs Odometry message
@output: body velocity commands as geometry_msgs Twist message;
         list of waypoints as MarkerArray message
"""

import math
import rospy, time
import numpy as np
from geometry_msgs.msg import Twist, Point, Vector3, PoseStamped, PointStamped
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import tf_conversions
import tf
class PDController:
    def __init__(self):
        self.Kd = rospy.get_param("/controller_waypoints/controller/Kd")
        self.Kp = rospy.get_param("/controller_waypoints/controller/Kp")
        self.distance_margin = rospy.get_param("/controller_waypoints/mission/distance_margin")
        self.waypoints = rospy.get_param("/controller_waypoints/mission/waypoints")
        self.mandatory_wp_count = len(self.waypoints)

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
        self.term = 0

        self.wpIndex = 0          # counts the visited waypoints
        self.position = Point()   # current position (3D vector: .x, .y, .z)
        self.heading = 0.0        # current orientation of robot (yaw [rad])
        self.time = rospy.Time.now().to_sec()
        self.align = False        # do the alignment flag before going to next wp

        self.delta_distance = 0.0
        self.delta_angle = 0.0
        self.delta_lin_velocity = 0.0
        self.delta_ang_velocity = 0.0

        self.done = False          # no more wp
        self.init = True
        self.vel_cmd = [0.0, 0.0]  # calculated velocities (linear, angular)

        self.publisher_cmd_vel = rospy.Publisher("/controller_diffdrive/cmd_vel",Twist, queue_size=10)
        self.publisher_waypoints = rospy.Publisher("/mission_control/waypoints",MarkerArray,queue_size=10)
        rospy.Subscriber('odom', Odometry, self.onOdom)
        #rospy.Subscriber('/controller_diffdrive/odom', Odometry, self.onOdom)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.appendExtraWaypoint)
        self.listener = tf.TransformListener()

        self.marker_array = None
        self.twist = Twist()
        self.startTime = 0
        while self.startTime == 0:
            self.startTime = rospy.Time.now().to_sec()

    def appendExtraWaypoint(self, msg):
        self.waypoints.append([msg.pose.position.x, msg.pose.position.y]) 
        rospy.loginfo("Appended: [%.1f, %.1f]",   msg.pose.position.x, msg.pose.position.y)
        self.done = False

    def wrapAngle(self, angle):
        if (abs(angle) > math.pi):
            a = 2*math.pi - abs(angle)
            return -a
        else:
            return angle
        
    def run(self):
        rospy.loginfo("Waiting for odom message...")
        rospy.spin()

    def setNextWaypoint(self):
        if not self.waypoints:
            return False
        self.waypoints.pop(0)
        if not self.waypoints:
            return False
        rospy.loginfo("----------------------------------------------")
        rospy.loginfo("                Next waypoint                 ")
        rospy.loginfo("%s",self.waypoints[0])
        return True

    def isWaypointReached(self):
        if (self.delta_distance<=self.distance_margin):
            self.wpIndex += 1
            self.align = True
            return True
        return False

    def controller(self):
        """
        command = Kp e + Kd e_dot
        e       = Vector2(distance err          , angle err)
        e_dot   = Vector2(velocity change in dt , angle change in dt)
        """
        e =     [self.Kp[0] * self.delta_distance,      self.Kp[1] * self.delta_angle]
        e_dot = [self.Kd[0] * self.delta_lin_velocity,  self.Kd[1] * self.delta_ang_velocity]
        self.vel_cmd = [e[0] + e_dot[0], e[1] + e_dot[1]] 

        if (self.align == True): #while aligning the heading, linear speed is 0
            # rospy.loginfo("Aligning towards the goal")
            self.vel_cmd = [0, e[1] + e_dot[1]] 
        

    def publish_vel_cmd(self):
        self.term += 1
        self.twist.linear = Vector3(self.vel_cmd[0], 0, 0)
        self.twist.angular =  Vector3(0, 0, self.vel_cmd[1])
        self.publisher_cmd_vel.publish(self.twist)
        self.time = rospy.Time.now().to_sec()

    def publish_waypoints(self):
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
        #corrected values of pose in the odom frame / latest transform from map to odom
        #(trans,rot) = self.listener.lookupTransform('/odom', '/map', rospy.Time(0))

        p = PoseStamped()
        p.header.frame_id = 'odom'
        p.pose = odom_msg.pose.pose
        transformed_pose = self.listener.transformPose('odom', p)



# The gmapping package does not directly publish any pose. 
# It will publish a topic /map which is an occupancy grid. 
# It will also publish a transform to the map frame from odom; 
# this is essentially a roundabout way of getting a pose. 
# If you want a pose in this frame you need to create another node that takes in the current pose, 
# most recent transform produced from gmapping,
#  and apply it to the pose. This can be done with the tf package, for example:

        prev_time = self.time
        current_time = rospy.Time.now().to_sec()
        delta_time = current_time - prev_time
        
        prev_prosition = self.position
        prev_heading = self.heading
        
        # self.position = odom_msg.pose.pose.position
        # q = odom_msg.pose.pose.orientation
        self.position = transformed_pose.pose.position
        q = transformed_pose.pose.orientation
        _, _, self.heading = tf_conversions.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
    
        curr_position = self.position
        curr_heading = self.heading

        if self.waypoints:
            #1
            goal = self.waypoints[0]
            delta_x = (goal[0] - curr_position.x)
            delta_y = (goal[1] - curr_position.y)
            self.delta_distance = math.sqrt( pow(delta_x, 2) + pow(delta_y, 2) )

            #2
            atan =  math.atan2(delta_y, delta_x)
            angle = atan - curr_heading
            self.delta_angle = self.wrapAngle(angle)

            #3
            delta_x_prev = (curr_position.x - prev_prosition.x)
            delta_y_prev = (curr_position.y - prev_prosition.y)
            delta_distance_prev = math.sqrt( pow(delta_x_prev, 2) + pow(delta_y_prev, 2) )
            self.delta_lin_velocity = (delta_distance_prev) /  (delta_time + 0.001)

            #4
            self.delta_ang_velocity = (curr_heading - prev_heading) / (delta_time + 0.001)

            if ( abs(math.degrees(self.delta_angle)) < 10):
                self.align = False

            # if ((self.term%10) == 1):
            #     rospy.loginfo("Heading is: %.2f degrees",  math.degrees(curr_heading))
            #     rospy.loginfo("Goal is: %.2f degrees",  math.degrees(atan))
            #     rospy.loginfo("Delta goal is: %.2f",   math.degrees(self.delta_angle))
            #     rospy.loginfo("Odom wp is: [%.1f, %.1f]",   curr_position.x, self.position.y)
            #     rospy.loginfo("Goal wp is: %s",   goal)
            #     rospy.loginfo("----")

        if self.isWaypointReached():
            if (self.wpIndex == self.mandatory_wp_count):
                rospy.loginfo("This was the last waypoint in the list.")
                endTime = rospy.Time.now().to_sec()
                rospy.loginfo("Started node  [s]: %.2f", self.startTime)
                rospy.loginfo("Finished node [s]: %.2f", endTime)
                totalTime = endTime - self.startTime
                rospy.loginfo("Elapsed time  [s]: %.2f", totalTime)

            if not self.setNextWaypoint():
                if not self.done:
                    rospy.loginfo("No more waypoints. Add WP")
                    self.done = True

        if not self.done:
            self.controller()
            self.publish_vel_cmd()
            self.publish_waypoints()

if __name__ == '__main__':
    rospy.init_node("Planner")
    controller = PDController()
    controller.run()