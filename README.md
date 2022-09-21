# fe_ur5e
Example for using UR5e@ULFE with Pilz motion planner.

## install UR ros driver:

https://github.com/UniversalRobots/Universal_Robots_ROS_Driver

## install moveit

sudo apt install ros-melodic-moveit

## instal pilz motion planner

http://wiki.ros.org/pilz_robots/Tutorials/MoveRobotWithPilzCommand_planner

### Pilz robots:
sudo apt install ros-melodic-pilz-robots

## UR robot calibration

Get calibration data from real robot.

roslaunch ur_calibration calibration_correction.launch robot_ip:=192.168.65.244 target_filename:="${HOME}/catkin_ws_ur5e/src/ur/ur5e_fe/calib/ur5e_1_calibration.yaml"

## UR driver
Launch robot ROS driver.

roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.65.244 kinematics_config:=$(rospack find ur5e_fe)/calib/ur5e_1_calibration.yaml

## UR script
Load and run script on UR5e robot

feros/ros_connect.urp

## Moveit with Pilz 
Load and run Moveit with Pilz motion controller. At the same time Rviz is opened for moving the robot via GUI.

roslaunch ur5e_fe pilz_real_robot_ur5e.launch




