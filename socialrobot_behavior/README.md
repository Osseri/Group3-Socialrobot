# socialrobot_behavior

<!-- Variables -->
[SRP_main]: https://gitlab.com/social-robot/socialrobot

- Version 1.0.0
- [[Go to the Social Robot Project Main]][SRP_main]

---

<div style="display:flex;">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

**Package summary**

Lorem ipsum dolor sit amet, consectetur adipisicing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.

- Maintainer status: maintained
- Maintainers
  - John Doe (john1@organization.com)
  - John Doe (john2@organization.com)
  - John Doe (john3@organization.com)
- Author
  - John Doe (john0@organization.com)
- License: {License Name}
- Source: git https://gitlab.com/social-robot/socialrobot_behavior.git

</div>
<div style="flex:40%; padding-left:10px;">

**Table of Contents**
1. [Overview](#overview)
2. [Installation methods](#installation-methods)
   1. [Install manually](#install-manually)
3. [Dependencies](#dependencies)
   1. [Frameworks](#frameworks)
   2. [Third-party libraries](#third-party-libraries)
   3. [Social Robot Project Modules](#social-robot-project-modules)
   4. [Hardware requirements](#hardware-requirements)
4. [Quick start](#quick-start)
   1. [Guide for adding Behaviors](#guide-for-adding-behaviors)
   2. [Behavior Demo](#behavior-demo)
5. [Descriptions of Behavior module](#descriptions-of-behavior-module)
   1. [1. Pick.py](#1-pickpy)
   2. [2. `Pick_test.py`](#2-pick_testpy)

</div>
</div>

---

## Overview

Lorem ipsum dolor sit amet, consectetur adipisicing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.

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
- pyassimp > 4.1.3

### Social Robot Project Modules

- socialrobot_motion
- socialrobot_hardware

### Hardware requirements

This package does not require any hardware device.

## Quick start

### Guide for adding Behaviors

1. 행위에 대한 motion planner들은 script/behaviors 안에 python 모듈로 작성해야함.
   
   예) 물체 접근 행위`approach.py`, 로봇 그리퍼 open/close 행위`openclose.py`, 로봇 팔 이동 행위`movearm.py`

2. 행위 모듈 클래스들은 script/behaviros 안의 behavior.py를 import하여 BehaviorBase를 상속받고 클래스명은 행위이름 뒤에 `Behavior`를 붙임.
   
   예) approach행위:

   ```
   from behavior import BehaviorBase

   class ApproachBehavior(BehaviorBase):
   ```

3. 행위클래스를 정의할때 다음의 함수들이 오버라이딩으로 구현되어야 함.

   ```
   def check_requirements()         #행위에 필요한 정보들(필요 하드웨어 대상, 컨트롤러 등)을 응답
   def prepare_behavior()           #행위 시작 전 컨트롤러 셋업
   def run_behavior()               #행위 시작 명령
   def finish_behavior()            #행위 종료 응답
   def get_motion()                 #행위 모션 결과 응답
   ```

### Behavior Demo

V-REP 시뮬레이션을 통한 검증은 다음과 같음.

1. socialrobot 관련 패키지들을 설치. Sociaorobot gitlab 참고.
2. roscore 실행 후, vrep 실행
3. 필요 ROS 패키지들을 launch

   ```
   roslaunch socialrobot_interface init.lauunch
   ```

4. 행위에 필요한 파라미터들을 `GetMotion.srv` 서비스를 통해 request
5. responce로 받은 motion trajectory를 `SetBehavior.srv` 서비스를 통해 request하여 vrep 시뮬레이션 내 결과 확인

`example/`의 예제 코드 참고.

## Descriptions of Behavior module 

### 1. Pick.py

targetBody를 바탕으로 원하는 물체(targetObject)를 manipulability를 고려하여 잡음

- Variable : approach pos's length(line. 155, approach_pos)
- Input value : targetBody, obstacle_ids, obstacles, targetObject, grasp_point
- Output value : planResult, jointTrajectory

### 2. `Pick_test.py`
왼쪽, 오른쪽 매니퓰레이터를 바탕으로 원하는 물체(targetObject)를 manipulability를 고려하여 잡음

- Variable : approach pos's length(line. 155, approach_pos)
- Input value : obstacle_ids, obstacles, targetObject, grasp_point
- Output value : planResult, jointTrajectory

`예제 1`

1. 다음의 노드들을 실행
   ```
   roslaunch social_robot_description demo.launch
   ```
   ```
   rosparam set /robot_name 'social_robot'
   roslaunch socialrobot_behavior behavior.launch 
   ```
   ```
   rosparam set /robot_name 'social_robot'
   roslaunch socialrobot_motion motion_plan.launch
   ```

2. Moveit!의 planning 부분에서 left_eef, right_eef의 상태를 open_left_hand, open_right_hand로 update 후 plan and execute(2번 실행).

3. example_pick.py 또는 example_pick_test.py 실행
   - example_pick.py
      - 오렌지 쥬스(obj_juice)를 왼쪽 팔로 잡는 예제.
      - 30도 각도마다 2방향의 그리퍼 방향을 갖는 24개의 grasp_point를 설정.
   
   ```
   rosparam set /robot_name 'social_robot'
   rosrun socialrobot_behavior example_pick.py
   ```

   - example_pick_test.py
      - 스프레이(obj_spray)를 잡는 예제.
      - 30도 각도마다 2방향의 그리퍼 방향을 갖는 24개의 grasp_point를 설정.

   ```
   rosparam set /robot_name 'social_robot'
   rosrun socialrobot_behavior example_pick_test.py
   ```

`예제 2`

1. 다음의 노드들을 실행
   ```
   roslaunch skkurobot_description demo.launch
   ```
   ```
   rosparam set /robot_name 'skkurobot'
   roslaunch socialrobot_behavior behavior.launch 
   ```
   ```
   rosparam set /robot_name 'skkurobot'
   roslaunch socialrobot_motion motion_plan.launch
   ```

2. example_pick.py 또는 example_pick_test.py 실행
   - example_pick.py
      - 오렌지 쥬스(obj_juice)를 왼쪽 팔로 잡는 예제.
      - 30도 각도마다 2방향의 그리퍼 방향을 갖는 24개의 grasp_point를 설정.
   
   ```
   rosparam set /robot_name 'skkurobot'
   rosrun socialrobot_behavior example_pick.py
   ```

   - example_pick_test.py
      - 스프레이(obj_spray)를 잡는 예제.
      - 30도 각도마다 2방향의 그리퍼 방향을 갖는 24개의 grasp_point를 설정.

   ```
   rosparam set /robot_name 'skkurobot'
   rosrun socialrobot_behavior example_pick_test.py
   ```

---

- [[Go to the Social Robot Project Main]][SRP_main]
