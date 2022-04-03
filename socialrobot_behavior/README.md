# socialrobot_behavior

<!-- Variables -->
[SRP_main]: https://gitlab.com/social-robot/socialrobot

- Version 1.0.0
- [[Go to the Social Robot Project Main]][SRP_main]

---

<div style="display:flex;">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

**Package summary**
Manage behavior modules to plan motion path based on motion planners in [socialrobot_motion](https://gitlab.com/social-robot/socialrobot_behavior.git) for actions defined in [socialrobot_actionlib](https://gitlab.com/social-robot/socialrobot_actionlib.git).

- Maintainer status: maintained
- Maintainers
  - Jeongmin Jeon (nicky707@daum.net)
  - Hong-ryul Jung (jung.hr.1206@gmail.com)
  - Hyungpil Moon (hyungpil@skku.edu)
- Author
  - Jeongmin Jeon (nicky707@daum.net)
- Source: git https://gitlab.com/social-robot/socialrobot_behavior.git

</div>
<div style="flex:40%; padding-left:10px;">

**Table of Contents**
- [socialrobot_behavior](#socialrobot_behavior)
  - [Overview](#overview)
  - [Installation methods](#installation-methods)
    - [Install manually](#install-manually)
  - [Dependencies](#dependencies)
    - [Frameworks](#frameworks)
    - [Third-party libraries](#third-party-libraries)
    - [Social Robot Project Modules](#social-robot-project-modules)
    - [Hardware requirements](#hardware-requirements)
  - [Quick start](#quick-start)
    - [Guide for adding Behaviors](#guide-for-adding-behaviors)
  - [Nodes](#nodes)
      - [Parameters](#parameters)
    - [{Node1 Name}](#node1-name)
      - [Subscribed Topics](#subscribed-topics)
      - [Services](#services)
    - [Behavior Demo](#behavior-demo)
  - [Descriptions of Behavior module](#descriptions-of-behavior-module)
    - [1. `example_openhand.py`](#1-example_openhandpy)
    - [2. `example_approacharm.py`](#2-example_approacharmpy)

</div>
</div>

---

## Overview

This package manages motion planner and controller package for the actual robot to perform a given action. When the primitive action that the robot can perform is given with the target part and the constraints, it is calculated using the appropriate planner to obtain the corresponding motion. It manages external ROS motion packages and is currently connected to the manipulator planner([MoveIt](https://moveit.ros.org/)), grasp planner([GraspIt]((https://graspit-simulator.github.io/))).

## Installation methods

### Install manually

1. Install the ROS. [Instructions for Ubuntu 16.04](http://wiki.ros.org/indigo/Installation/Ubuntu)
2. [Setup your ROS environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
3. Install vision_msgs ROS package
   ```
   sudo apt-get install ros-kinetic-vision-msgs
   ```   
4. Install moveit ROS package
   ```
   sudo apt-get install ros-kinetic-moveit
   ```
5. make
   ```
   catkin_make
   ```
6. If pyassimp error occurred when launch the package, upgrade pyassimp version upper than 4.1.3

## Dependencies

### Frameworks

- ROS Kinetic/Melodic

### Third-party libraries

- [vision_msgs](https://github.com/Kukanani/vision_msgs)
- [moveit](https://github.com/ros-planning/moveit)
- [GraspIt](https://graspit-simulator.github.io/)
- pyassimp > 4.1.3

### Social Robot Project Modules

- [socialrobot_motion](../socialrobot_motion/)
- [socialrobot_hardware](../socialrobot_hardware/)

### Hardware requirements

This package does not require any hardware device.

## Quick start

### Guide for adding Behaviors

1. Motion planners for primitive actions must be written as python modules in /script/behabiors.

   ex) approaching motion approach.py, robot gripper's open/close motionopenclose.py, robot manipulator motion movearm.py

2. Behavior module classes import behavior.py in script/behavir to inherit BehaviorBase, and the class name must be '{Action name}Behavior'.

   ```
   from behavior import BehaviorBase

   class ApproachBehavior(BehaviorBase):
   ```

3. When defining an action class, the following functions should be implemented as an override.

   ```
   def check_requirements()         #Respond to information needed for action (for hardware required, for controller, etc.)
   def prepare_behavior()           #Set up the controller before starting the action
   def run_behavior()               #Action start command
   def finish_behavior()            #Action response
   def get_motion()                 #Response to Motion planning Results
   ```

## Nodes

#### Parameters

- ~robot_name (string, default: socialrobot)
  - set robot description name.
- ~robot_hw (string, default: hw)
  - set robot mode as hardware or v-rep simulator 

### {Node1 Name}
<div style="padding-left:40px;">
</div>

  
#### Subscribed Topics

- ~/socialrobot/state ([stds_msgs/Int32]())
  - READY_FOR_ACTION = 0
  - ACTION_RUNNING = 1:
  
#### Services

- ~get_behavior_list ([socialrobot_behavior/GetBehaviorList](./srv/GetBehaviorList.srv))
  - request behavior module lists

<div style="display:flex; padding-left:50px">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

Request

- query (`std_msgs/Empty`)
  - no request parameter
</div>
<div style="flex:50%; padding-left:10px;">

Response

- behavior_list (`string`)
  - behavior module names

</div>
</div>

- ~get_requirements ([socialrobot_behavior/GetRequirements](./srv/GetRequirements.srv))
  - request behavior module lists

<div style="display:flex; padding-left:50px">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

Request

- behavior_name (`string`)
  - no request parameter
</div>
<div style="flex:50%; padding-left:10px;">

Response

- result (`bool`)
  - request result (False/True)
  - 
- requirements (`string[]`)
  - required parameters in [`socialrobot_msgs/Behavior.msg`](../socialrobot_msgs/msg/Behavior.msg)


</div>
</div>

- ~get_motion ([socialrobot_behavior/GetMotion](./srv/GetMotion.srv))
  - request behavior module lists

<div style="display:flex; padding-left:50px">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

Request

- behavior_name (`string`)
  - no request parameter
</div>
<div style="flex:50%; padding-left:10px;">

Response

- result (`bool`)
  - request result (False/True)
  - 
- requirements (`string[]`)
  - required parameters in [`socialrobot_msgs/Behavior.msg`](../socialrobot_msgs/msg/Behavior.msg)


</div>
</div>

- ~set_behavior ([socialrobot_behavior/SetBehavior](./srv/SetBehavior.srv))
  - request execute behavior following motion trajectory

<div style="display:flex; padding-left:50px">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

Request

- header (`std_msgs/Header`)
  - header for behavior motion
  - 
- trajectory (`trajectory_msgs/JointTrajectory`)
  - motion trajectory for manipulator

- path (`nav_msgs/Path`)
  - motion trajectory for mobile
</div>
<div style="flex:50%; padding-left:10px;">

Response

- result (`bool`)
  - request result 
  - OK = 0
  - ERROR = 1

</div>
</div>

### Behavior Demo

Executing behavior demo for V-REP simulator.

1. Install [socialrobot packages](../).
2. run V-REP simulator after roscore
3. Launch the socialrobot packages

   ```
   roslaunch socialrobot_interface init.launch
   ```

4. request motion trajectory via `GetMotion.srv` 
5. request executing motion from `GetMotion.srv` via `SetBehavior.srv`
   ```
   rosrun socialrobot_behavior example_approacharm.py
   ```

examples of behavior are in `example/` path.

## Descriptions of Behavior module 

### 1. `example_openhand.py`

Open the gripper corresponding to `robot_group`.

- Input parameters : robot_group

### 2. `example_approacharm.py`
Motion planning to move the manipulator corresponding to the `robot_group` to the position before grasping the `target_object`.

- Input parameters : robot_group, target_object, static_object, dynamic_object

`example 1`

1. launch [`socialrobot_motion`](../socialrobot_motion/) package
   ```
   rosparam set /robot_name 'social_robot'
   roslaunch socialrobot_motion motion_plan.launch 
   ```

2. launch [`socialrobot_behavior`](./) package.
   ```
   roslaunch socialrobot_behavior behavior.launch 
   ```
3. run the example node
   ```
   rosrun socialrobot_behavior example_approacharm.py
   ```

---

- [[Go to the Social Robot Project Main]][SRP_main]
