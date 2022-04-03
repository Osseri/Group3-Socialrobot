# Hardware Interface

<!-- Variables -->
[SRP_main]: https://gitlab.com/social-robot/socialrobot

- Version 1.0.0
- [[Go to the Social Robot Project Main]][SRP_main]

---

<div style="display:flex;">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

**Package summary**

socialrobot hardware interface package.

- Maintainer status: maintained
- Maintainers
  - Jeongmin Jeon (nicky707@daum.net)
  - Hong-ryul Jung (jung.hr.1206@gmail.com)
  - Hyungpil Moon (hyungpil@skku.edu)
- Author
  - Jeongmin Jeon (nicky707@daum.net)
- License: {License Name}
- Source: git https://gitlab.com/social-robot/socialrobot_hardware.git

</div>
<div style="flex:40%; padding-left:10px;">

**Table of Contents**
- [Hardware Interface](#hardware-interface)
  - [Overview](#overview)
  - [Installation methods](#installation-methods)
  - [Dependencies](#dependencies)
    - [Frameworks](#frameworks)
    - [Hardware requirements](#hardware-requirements)
      - [Parameters](#parameters)
  - [Quick start](#quick-start)
  - [Nodes](#nodes)
    - [hw_interface](#hw_interface)
      - [Published Topics](#published-topics)
      - [Messages](#messages)
      - [Services](#services)
    - [path_follower](#path_follower)
      - [Published Topics](#published-topics-1)
      - [Subscribed Topics](#subscribed-topics)
      - [Services](#services-1)
    - [Example](#example)

</div>
</div>

---

## Overview

ROS package for interface management to connect robot hardware or vrep simulator with a social robot system.

## Installation methods
```
catkin_make
```

## Dependencies
- V-REP simulator
  
### Frameworks

- ROS Kinetic/Melodic

### Hardware requirements

This package requires a hardware device.

#### Parameters

- ~robot_name (string, default: social_robot)
  - set robot name to load robot description package.
- ~sim_env (bool, default: True)
  - set robot hardware mode
  - True : simulation
  - False : robot hardware
- ~sim_env (string, default: default)
  - If `sim_env` is set to True, define the V-rep scene file name

## Quick start 
```
roslaunch socialrobot_hardware hardware_interface.launch
```


## Nodes

### hw_interface

<div style="padding-left:40px;">

#### Published Topics

- ~joint_states ([sensor_msgs/JointState]()
  - robot joint states
  
#### Messages

- VrepState.msg
  - vrep_state (`int32`)
    - SIM_STOPPED = 0
    - SIM_RUNNING = 1
  - action_state (`int32`)
    - READY_FOR_ACTION = 0
    - ACTION_RUNNING = 1
- ObjectInfo.msg
  - If `sim_env` is True, publish object info on behalf of the `socialrobot_perception` modules.
  - names (`string[]`)
    - object name list
  - obstacles (`vision_msgs/BoundingBox3D[]`)
    - object bounding box information

#### Services

- ~set_motion ([SetJointTrajectory.srv](srv/SetJointTrajectory.srv))
  - request to the robot controller to execute motion trajectory 

<div style="display:flex; padding-left:50px">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

Request

- trajectory (`trajectory_msgs/JointTrajectory`)
  - motion trajectory calculated from motion planner
- duration (`float64`)
  - executing time

</div>
<div style="flex:50%; padding-left:10px;">

Response

- result (`int32`)
  - OK = 0
  - ERROR = 1

</div>
</div>

- ~get_state ([GetRobotState.srv](srv/GetRobotState.srv))
  - request to the robot controller to execute motion trajectory 

<div style="display:flex; padding-left:50px">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

Request

- query (`std_msgs/Empty`)
  - no request

</div>
<div style="flex:50%; padding-left:10px;">

Response

- state (`int32`)
  - robot's moving state
  - READY_STATE = 0
  - RUNNING_STATE = 1

</div>
</div>


</div>

### path_follower

<div style="padding-left:40px;">

#### Published Topics

- /cmd_vel ([geometry_msgs/Twist]()
  - publish robot velocity to follow requested mobile path

#### Subscribed Topics

- /odom ([nav_msgs/Odometry]()
  - mobile odometry

#### Services

- /set_path ([SetPathTrajectory.srv](srv/SetPathTrajectory.srv))
  - request to the robot controller to execute motion trajectory 

<div style="display:flex; padding-left:50px">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

Request

- trajectory (`nav_msgs/Path`)
  - motion trajectory calculated from mobile path planner
- duration (`float64`)
  - executing time

</div>
<div style="flex:50%; padding-left:10px;">

Response

- result (`int32`)
  - OK = 0
  - ERROR = 1

</div>
</div>

</div>

### Example

Trajectory follower

1. run V-REP simulator after roscore
2. launch the socialrobot packages

   ```
   roslaunch socialrobot_interface init.launch
   ```
3. run the example node
   ```
   rosrun socialrobot_hardware path_follwer.py
   ```

---

- [[Go to the Social Robot Project Main]][SRP_main]
