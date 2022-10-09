# Helpers

## ROS and catkin
Default cmd 
```console
source ./devel/setup.bash - in catkin workspace, source
rospack profile -- before rospack find transform_frame

roscore - start core
rosrun - start nodes
rosnode list - list all active nodes
rostopic list - list all active topics
rostopic echo /topic - echoes the msgs in topic
rostopic pub -1 /topic msg - publishes msg to topic
roslaunch - starts nodes as defines in .launch file
catkin_make - compiles into the catkin workspace. Need to resource

rosrun teleop_twist_keyboard teleop_twist_keyboard.py

```

## Tasks
Task 2. Launch URDF model 
```console
roslaunch ias0220_224772 differential_robot_task2.launch 

```


Task 3. Launch walker and position calcer 
```console
rosrun ias0220_224772 random_walker.py
rosrun ias0220_224772 odometer.py

```

Task 3, part 2 


In part 2, you learned how to make a launch file and practiced a little more communication between nodes: you had to start the keyboard control node sending velocity commands, and the move node receiving it, process it and output the position of the robot. Since the robot has actuated joints (the two base_to_****_wheel), the robot_joint_publisher has the role to keep these two joints' state up to date at all time. Then, the robot_state_publisher takes this, plus all the static joints (base_to_head, base_like_to_base_link,...) , and outputs the whole robot (static + non-static joints). Then, since the move node provides the transform from map to base_link, and the robot_state_publisher provides all the transforms from the base link to all other links of the robot, Rviz is able to display the entire robot relative to the map frame.

You probably noticed that here, you could move the robot by keyboard teleoperation, but the wheels were not moving, or you moved the wheels using the sliders in the joint_state_publisher_gui, but this did not make the robot move forward. This is because Rviz it just a visualization tool, it cannot know how the rotation of a wheel would impact the motion of the robot, or vice-versa (instead, we "faked" the motion of the robot by using the move node, that moves the base_link frame relative to the map frame based on keyboard input). To simulate this, we need a simulator. In the next assignment, you will import your robot in a simulator: Gazebo. Then, you will see how the motions in gazebo really depend on the wheels motion or on the objects interacting with the robot. Then, you will write an odometry node, whose role is to compute the pose of the robot, based on the velocity information (does that ring a bell? oh yes, you just made an odometry node of the random walker). Be prepared, you will need mathematics and mechanics.


```console
catkin_make - makes the transform_frame
source /devel/setup.bash
source /opt/ros/noetic/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py - vajab topicut cmd_vel
roslaunch ias0220_224772 differential_robot_task3_part2.launch 

```


## Additional helpers
1. Warnings/debugs\
http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch

2. .launch file structure\
http://www.clearpathrobotics.com/assets/guides/kinetic/ros/Launch%20Files.html\
http://wiki.ros.org/roslaunch/XML/node

3. helpers\
 "rqt_graph", "rostopic info", "rostopic list" or "rosnode info", "rosnode list"

4. Cheat sheet\
https://moodle.taltech.ee/pluginfile.php/470346/mod_assign/intro/ROScheatsheet_catkin.pdf
