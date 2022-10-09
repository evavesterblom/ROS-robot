# Helpers

Default cmd 
```console
roscore - start core
rosrun - start nodes
rosnode list - list all active nodes
rostopic list - list all active topics
rostopic echo /topic - echoes the msgs in topic
rostopic pub -1 /topic msg - publishes msg to topic
roslaunch - starts nodes as defines in .launch file

```

Task 2. Launch URDF model 
```console
roslaunch ias0220_224772 differential_robot_task2.launch 

```


Task 3. Launch walker and position calcer 
```console
rosrun ias0220_224772 random_walker.py
rosrun ias0220_224772 odometer.py

```


## Additional helpers
1. Warnings/debugs\
http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch

2. .launch file structure\
http://www.clearpathrobotics.com/assets/guides/kinetic/ros/Launch%20Files.html \
http://wiki.ros.org/roslaunch/XML/node
