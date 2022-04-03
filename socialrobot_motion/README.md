# socialrobot_motion 

<!-- Variables -->
[SRP_main]: https://gitlab.com/social-robot/socialrobot

- Version 1.0.0
- [[Go to the Social Robot Project Main]][SRP_main]

---

<div style="display:flex;">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

**Package summary**

socialrobot motion planner package.

- Maintainer status: maintained
- Maintainers
  - Jeongmin Jeon (nicky707@daum.net)
  - Hong-ryul Jung (jung.hr.1206@gmail.com)
  - Hyungpil Moon (hyungpil@skku.edu)
- Author
  - Jeongmin Jeon (nicky707@daum.net)
- License: {License Name}
- Source: git https://gitlab.com/social-robot/socialrobot_motion.git

</div>
<div style="flex:40%; padding-left:10px;">

**Table of Contents**
- [socialrobot_motion](#socialrobot_motion)
  - [Overview](#overview)
  - [Installation methods](#installation-methods)
    - [Install manually](#install-manually)
  - [Dependencies](#dependencies)
    - [Frameworks](#frameworks)
    - [Third-party libraries](#third-party-libraries)
    - [Social Robot Project Modules](#social-robot-project-modules)
    - [Hardware requirements](#hardware-requirements)
  - [Features](#features)
    - [arm_planner](#arm_planner)
    - [grasp_planner](#grasp_planner)

</div>
</div>

---

## Overview

ROS package for Motion planning interface. This module managing MoveIt, GraspIt planner packages.


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
5. Install graspit and dependencies
   ```
   sudo apt install libqt4-dev libqt4-opengl-dev libqt4-sql-psql libcoin80-dev libsoqt4-dev libblas-dev liblapack-dev libqhull-dev libeigen3-dev
   ```

   clone graspit source 

   ```bash
   git clone https://github.com/graspit-simulator/graspit.git
   ```

   make & install

   ```
   cd graspit
   mkdir build
   cd build
   cmake ..
   make -j5
   sudo make install
   ```

   add environment variable to ~/.bashrc (or zshrc)

   ```
   export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
   export GRASPIT=~/.graspit
   ```
6. Install graspit ROS packages
   ```
   //move to ROS workspace path
   cd <WORKSPACE_DIR>/src
   
   //clone packages
   git clone https://github.com/graspit-simulator/graspit_interface.git
   git clone https://github.com/graspit-simulator/graspit_commander.git

   //build workspace
   cd graspit_ros_ws
   catkin_make
   ```

## Dependencies

### Frameworks

- ROS Kinetic/Melodic

### Third-party libraries

- [moveit](https://github.com/ros-planning/moveit)
- [GraspIt](https://graspit-simulator.github.io/)
- pyassimp > 4.1.3

### Social Robot Project Modules

- Robot description packages including urdf file.

### Hardware requirements

This package does not require any hardware device.

## Features

### arm_planner

Object mesh file path : `../socialrobot_motion/mesh/moveit/`

1. Caculate manipulability
   - targetPose에서의 maipulability를 구함
      - Input : targetBody, find_manipulability, goalType, targetPose
      - Output : planResult, manipulability
2. Update scene, detach & attach objects
   - 물체의 상태나 환경을 업데이트
      - Input : targetBody, obstacle_ids, obstacles, targetObject
3. Compute path
   - 정해진 위치 또는 각도로 움직이기 위한 joint trajectory 생성
      - Input : targetBody, obstacle_ids, obstacles, goalType, targetPose or targetJointState, currentJointState(default :current joint state in moveit!)
      - Output : planResult, jointTrajectory

`example`

```
from socialrobot_motion.srv import *

def main():
   srv_arm_plan = rospy.ServiceProxy('/motion_plan/move_arm', MotionPlan)

   arm_req = MotionPlanRequest()

   // set input value
   arm_req.targetBody = MotionPlanRequest.LEFT_ARM
   arm_req.obstacle_ids = <obstacle_ids>
   arm_req.obstacles = <obstacles>
   arm_req.goalType = MotionPlanRequest.CARTESIAN_SPACE_GOAL
   arm_req.targetPose = <targetPose>
   arm_req.currentJointState = <currentJointState>
   
   Result = srv_arm_plan(arm_req)
```

### grasp_planner

- Gripper model
   - name : robot_name + "_left", "_right"
   - path : `../.graspit/models/robots/`
   - 필요한 gripper model은 [[Robocare 패키지에서 social_robot_description/graspit_model]](https://gitlab.com/social-robot/robocare/-/tree/master/social_robot_arm/social_robot_description/graspit_model)로 들어가면 찾을 수 있다.
- Object model
   - path : `../socialrobot_motion/mesh/graspit/`
   - 기존 물체 목록: [socialrobot_commons/object_dataset/object_lists.md](https://gitlab.com/social-robot/socialrobot_commons/-/blob/master/object_dataset/object_lists.md)
- Input value
   - targetBody, obstacle_ids, obstacles, targetObject, gripper_pose(default :current gripper pose in moveit!)
- Output value
   - planResult, endEffectorPose, graspQuality, dofs

`example`

```
from socialrobot_motion.srv import *

def main():
   srv_grasp_plan = rospy.ServiceProxy('/motion_plan/grasp_plan', MotionPlan)

   grasp_req = MotionPlanRequest()

   // set input value
   grasp_req.targetBody = MotionPlanRequest.LEFT_GRIPPER
   grasp_req.obstacle_ids = <obstacle_ids>
   grasp_req.obstacles = <obstacles>
   grasp_req.targetObject = <targetObject>
   grasp_req.gripper_pose = <gripper_pose>

   Result = srv_grasp_plan(grasp_req)
```

---

- [[Go to the Social Robot Project Main]][SRP_main]
