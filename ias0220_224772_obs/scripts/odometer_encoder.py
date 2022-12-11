#!/usr/bin/env python
# Software License Agreement (BSD License)

# Simple talker demo that listens to std_msgs/Strings published
# rosrun ias0220_224772 odometer_encoder.py 


import rospy
import time
import math
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
import tf
from ias0220_224772_obs.msg import counter_message

def calculate_angular_velocity(ticks, time):
    return ((ticks / 2048) * 2 * math.pi) / time

def calculate_linear_velocity(omega, radius):
    return omega * radius

def calculate_ticks_since(previous, current):
    treshold = 1200
    if abs(current - previous) < treshold:
        ticks_since = current - previous
    else:
        if (current < previous):
            ticks_since = 2048 - previous + current
        else:
            ticks_since = -(2048 - current + previous)
    return ticks_since

def calculate_publish_odom_msg(robot_linear_velocity, robot_anglar_velocity, time_since, time_header):     #https://answers.ros.org/question/231942/computing-odometry-from-two-velocities/
    #velocities robot (r) frame
    dt = time_since
    v_rx = robot_linear_velocity
    v_ry = 0
    omega_r = robot_anglar_velocity

    #velocities odom (o) frame
    global theta_k
    v_ox = v_rx * math.cos(theta_k) - v_ry * math.sin(theta_k)
    v_oy = v_rx * math.sin(theta_k) + v_ry * math.cos(theta_k)
    thetadot = omega_r

    #compute current robot pose with respect to odom - integration
    global x
    global y
    x = x + (v_ox * dt)
    y = y + (v_oy * dt)
    theta_k = theta_k + (thetadot * dt)

    global odom_msg #fill odom msg
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_link"
    odom_msg.header.stamp = time_header
    odom_msg.header.seq +=1
    odom_msg.twist.twist.linear = Vector3(v_rx, 0, 0) #twist linear velocity in base_link (r) frame
    odom_msg.twist.twist.angular = Vector3(0, 0, omega_r) #twist angular velocity in base_link (r) frame
    odom_msg.pose.pose.position.x = x #Pose
    odom_msg.pose.pose.position.y = y
    odom_msg.pose.pose.position.z = 0
    quaternion = Quaternion()
    quaternion = tf.transformations.quaternion_from_euler(0, 0, theta_k) #https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf
    odom_msg.pose.pose.orientation.x = quaternion[0]
    odom_msg.pose.pose.orientation.y = quaternion[1]
    odom_msg.pose.pose.orientation.z = quaternion[2]
    odom_msg.pose.pose.orientation.w = quaternion[3]
    pub_odom.publish(odom_msg)

def callback_encoder_listener(data):
    global counter
    counter += 1
    
    current_left_counter = data.count_left #read encoder
    current_right_counter = data.count_right
    time = rospy.get_time()
    time_header = rospy.Time.now()
    global previous_time
    time_since = time - previous_time

    #ticks
    global previous_left_counter
    global previous_right_counter 
    if (counter == 1):
        previous_left_counter = current_left_counter
        previous_right_counter = current_right_counter

    left_since = calculate_ticks_since(previous_left_counter, current_left_counter)
    right_since = calculate_ticks_since(previous_right_counter, current_right_counter) * -1 #reversed movement

    #angular velocity
    left_angual_velocity = calculate_angular_velocity(left_since, time_since)
    right_angual_velocity = calculate_angular_velocity(right_since, time_since)

    #linear velocity
    left_linear_velocity = left_angual_velocity * 0.04 #omega*R
    right_linear_velocity = right_angual_velocity * 0.04

    #linear velocity robot Vp
    robot_linear_velocity = (left_linear_velocity + right_linear_velocity) / 2

    #rotational velocity robot wz
    robot_anglar_velocity = (right_linear_velocity - left_linear_velocity) / 0.2

    calculate_publish_odom_msg(robot_linear_velocity, robot_anglar_velocity, time_since, time_header)
    

    #rospy.loginfo('%s - %s', left_angual_velocity, right_angual_velocity)
    #rospy.loginfo('%s', current_right_counter)
    #rospy.loginfo('Left data %s   prev %s     current %s   ticks %s', time_since, previous_left_counter, current_left_counter, left_since)
    #rospy.loginfo('Left data %s   prev %s     current %s   ticks %s', time_since, previous_right_counter, current_right_counter, right_since)
    #rospy.loginfo('L data %s   prev %s     current %s   ticks %s', time_since, previous_left_counter, current_left_counter, left_since)
    #rospy.loginfo('%s : %s', left_linear_velocity, right_linear_velocity)


    #iteration history
    previous_left_counter = current_left_counter
    previous_right_counter = current_right_counter
    previous_time = time
    if (counter%100 == 1):
        rospy.loginfo('I am alive')

def mainloop():
    rospy.Subscriber('encoders_output', counter_message, callback_encoder_listener)
    rospy.spin()

if __name__ == '__main__':    
    rospy.init_node('position_calculator', anonymous=True)
    pub_odom = rospy.Publisher('odom', Odometry, queue_size=50)

    while rospy.get_time() == 0.0: #wait init
        time.sleep(1.0)
    previous_time = rospy.get_time() #init time
    previous_left_counter = 0
    previous_right_counter = 0
    
    odom_msg = Odometry() #init odom msg
    odom_msg.header.seq = 1
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_link"
    linear_v = Vector3(0, 0, 0)
    angular_v = Vector3(0, 0, 0)
    odom_msg.twist.twist.linear = linear_v
    odom_msg.twist.twist.angular = angular_v
    odom_msg.pose.pose.position.x = 0
    odom_msg.pose.pose.position.y = 0
    odom_msg.pose.pose.position.z = 0
    quaternion = Quaternion()
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
    odom_msg.pose.pose.orientation.x = quaternion[0]
    odom_msg.pose.pose.orientation.y = quaternion[1]
    odom_msg.pose.pose.orientation.z = quaternion[2]
    odom_msg.pose.pose.orientation.w = quaternion[3]
    pub_odom.publish(odom_msg)

    x = 0 #init pose
    y = 0
    z = 0
    theta_k = 0
    counter = 0

    mainloop()
