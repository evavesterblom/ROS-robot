# Robot guidance and software

![Diff Drive Robot](ias0220_224772/data/robot_rviz_with_frames_task2_2.png)

The aim of the course is to provide basic knowledge in robot programming.
To do this, we will first introduce the cross-hardware robot operating system ROS, which is the most widely used robot operating system. 
The structure of such an operating system is different from the operating system of computers, 
representing rather a software intermediate layer that connects lower-level processes (such as reading sensor data and controlling motors) 
and robot planning and control software into a common whole.

In the theoretical part of the course, the basics of the operation of the basic algorithms of robots are explained, 
and in the practical part, they are realized in the ROS environment using the robot simulator Gazebo. 
As an example, we consider the most important robot algorithms:
- modeling the robot's surroundings, 
- determining its position, 
- planning the path and 
- avoiding obstacles on the way

**Tasks**

Weekly tasks are divided into different packages:
- Package ias0220_224772 - kinematics, ROS tools, URDF model, odometry, RVIZ config, subscribers-publishers, topics setup, kinematics with ROS tools
- Package ias0220_224772_vision - machine vision, openCV implementation, camera calibration
- Package ias0220_224772_sensor - sensors data in ROS (IMU), control based on pitch (drive forward backwards) roll (turn and drive), odometry
- Package ias0220_224772_pid - position and Orientation control of Differential Drive Robot with a PID controller, implementation
- Package ias0220_224772_obs - path planning and obstacle avoidance using odometry, control
- Package ias0220_224772_slam -  mapping and mapping algorithms

**Videos**
* AMCL testing in simulation https://youtu.be/-50NygNJ52s
* AMCL kidnapped in simulation - https://youtu.be/faS053IOvQc
* Path planning testing in simulation - https://youtu.be/WpgUh_Anijo
* PID and robot control implementation (odometry implementation) - https://youtu.be/sxULUTWcvZQ
* Odometry implementation - https://youtu.be/0LQANRFGBno



