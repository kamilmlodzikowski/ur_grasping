# README #

### What is this repository for? ###

* ROS node for UR control
* Version 0.01

### Requirements ###

* UR driver: https://github.com/ros-industrial/universal_robot
* Improvements: https://github.com/ThomasTimm/ur_modern_driver
* Patch: https://github.com/ThomasTimm/ur_modern_driver/issues/58
* Gazebo: sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
* Optoforce node: https://github.com/shadow-robot/optoforce/tree/indigo-devel/optoforce

### Run with simulator ###

* Roscore: source devel/setup.bash 
           roscore
* Gazebo: roslaunch ur_gazebo ur3.launch
* MoveIt: roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch sim:=true limited:=true
* Rviz: roslaunch ur3_moveit_config moveit_rviz.launch config:=true
* Optoforce node: roslaunch optoforce optoforce.launch
* controller: roslaunch controller_ur controller_SimUR.launch

### Run with real robot ###

* Roscore: source devel/setup.bash 
           roscore
* URdriver: roslaunch ur_bringup ur3_bringup.launch robot_ip:=192.168.0.103 [reverse_port:=REVERSE_PORT]
* MoveIt: roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch limited:=true
* Rviz: roslaunch ur3_moveit_config moveit_rviz.launch config:=true
* Optoforce node: roslaunch optoforce optoforce.launch
* controller: roslaunch controller_ur controller_ur.launch