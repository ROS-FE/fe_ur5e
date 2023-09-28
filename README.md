# fe_ur5e
Example for using UR5e@ULFE with Pilz motion planner.

## install UR ros driver:

https://github.com/UniversalRobots/Universal_Robots_ROS_Driver

## install moveit

```bash
$ sudo apt install ros-melodic-moveit
```

## instal pilz motion planner

http://wiki.ros.org/pilz_robots/Tutorials/MoveRobotWithPilzCommand_planner

### Pilz robots:

```bash
$ sudo apt install ros-melodic-pilz-robots
```

## UR robot calibration
Get calibration data from real robot.

```bash
$ roslaunch ur_calibration calibration_correction.launch robot_ip:=192.168.65.244 target_filename:="${HOME}/catkin_ws_ur5e/src/ur/ur5e_fe/calib/ur5e_1_calibration.yaml"
```

## UR driver
Launch robot ROS driver.

```bash
$ roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.65.244 kinematics_config:=$(rospack find ur5e_fe)/calib/ur5e_1_calibration.yaml
```

## UR script
Load and run script on UR5e robot

```bash
feros/ros_connect.urp
```
## Moveit with Pilz 
Load and run Moveit with Pilz motion controller. At the same time Rviz is opened for moving the robot via GUI.

```bash
$ roslaunch ur5e_fe pilz_real_robot_ur5e.launch
```

## Application
Use python moveit interface to plan robot application

MoveIt help:

https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html

class MoveGroupCommander:

http://docs.ros.org/en/indigo/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html

Check example:
```bash
$ roscd ur5e_fe
$ code ./scripts/main_ur5e.py
```
## Use I/O
States of inputs are published on topic `/ur_hardware_interface/io_states`

```bash
$ rostopic echo /ur_hardware_interface/io_states
```
Tool inputs are numbered DI16 and DI17.

Outputs can be set via service `/ur_hardware_interface/set_io`

```bash
$ rosservice call /ur_hardware_interface/set_io "fun: 0 pin: 0 state: 0.0"
```
Service use `ur_msgs/SetIO Service` (check documentation https://docs.ros.org/en/noetic/api/ur_msgs/html/srv/SetIO.html). Tool outputs are numbered DO16 and DO17.

To use in python a correct srv type needs to be imported:

```python
from ur_msgs.srv import SetIO, SetIORequest
...
io_service = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
...
msg = SetIORequest()
msg.fun = 1
msg.pin = 0
msg.state = 1.0
...
resp = io_service(msg)
```



