# socialrobot_actionlib

<!-- Variables -->
[SRP_main]: https://gitlab.com/social-robot/socialrobot

- Version 1.0.0
- [[Go to the Social Robot Project Main]][SRP_main]

---

<div style="display:flex;">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

**Package summary**

The Action Library module manages the action model definition of knowledge level and its associations with robot motion planning and control.

- Maintainer status: maintained
- Maintainers
  - Jeongmin Jeon (nicky707@daum.net)
  - Hong-ryul Jung (jung.hr.1206@gmail.com)
  - Hyungpil Moon (hyungpil@skku.edu)
- Author
  - Jeongmin Jeon (nicky707@daum.net)
- Source: git https://gitlab.com/social-robot/socialrobot_actionlib.git

</div>
<div style="flex:40%; padding-left:10px;">

**Table of Contents**
- [socialrobot_actionlib](#socialrobot_actionlib)
  - [Overview](#overview)
  - [Installation methods](#installation-methods)
    - [Install manually](#install-manually)
  - [Dependencies](#dependencies)
    - [Frameworks](#frameworks)
    - [Third-party libraries](#third-party-libraries)
    - [Social Robot Project Modules](#social-robot-project-modules)
    - [Hardware requirements](#hardware-requirements)
  - [Quick start](#quick-start)
  - [Guide for adding action models](#guide-for-adding-action-models)
        - [:types](#types)
        - [:constants](#constants)
        - [:pridicates](#pridicates)
        - [:action](#action)
    - [Example](#example)
  - [Nodes](#nodes)
    - [{Node1 Name}](#node1-name)
      - [Messages](#messages)
      - [Services](#services)
      - [Parameters](#parameters)

</div>
</div>

---

## Overview

Action Library is the set of actions that social robots can perform, and is written for the establishment of a knowledge system for operation. The actions that can be performed by the robot are defined in the symbolic level based on the PDDL including the pre-condition, the post-effect, and the hardware requirements in the metric level. The ROS service provides a list of action list that can be performed by specific robot and converts the commpound actions obtained through Task Planning into the primitive actions. Action description file should be written in pddl format file.

- move_arm
- grasp_object
- push_object
- rotate_object
- etc.

## Installation methods

### Install manually

1. Install the ROS melodic. [Instructions for Ubuntu 18.04](http://wiki.ros.org/melodic/Installation/Ubuntu)  
2. [Setup your ROS environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

3. make and launch 
    ```
    catkin_make
    source devel/setup.bash
    roslaunch socialrobot_actionlib actionlib.launch
    ```

## Dependencies

### Frameworks

- ROS Kinetic/Melodic
- 
### Third-party libraries

- 

### Social Robot Project Modules

- socialrobot_description (for initialization)

### Hardware requirements

This package does not require any hardware device.

## Quick start 

1. Launch Action Library
    ```
    roslaunch socialrobot_actionlib actionlib.launch
    ```

## Guide for adding action models

로봇 action을 pddl syntax 기반하여 `config/action_library.pddl`에 작성.

##### :types
- motion 계획 시 필요한 geometric parameter의 symbolic parameter
  - [Position](../socialrobot_msgs/msg/Position.msg)
  - [Object](../socialrobot_msgs/msg/Object.msg)

##### :constants
  - constant parameters 정의
  
##### :pridicates
  - predicates 정의
  - 
##### :action
  - Compound action과 Primitive action을 구분하여 정의
  - Compound action
    - :primitives 
      - 하위 레벨의 primitive action과 parameter들을 추가
  - Primitive action
    - :constraints
      - controller : 하드웨어 컨트롤러 종류 (e.g. position)
      - hardware_group : action 수행에 필요한 로봇 그룹 명 (e.g. mobile, left_arm, dual_arm)
      - planner : 모션계획을 수행하기위한 `socialrobot_behavior`에 정의된 모듈 명 
### Example

- 로봇 플랫폼에 대한 action domain요청 
  ```
  rospy example/testRequestDomainInfo.py 
  ```

- 로봇 플랫폼이 현재 action domain에서 수행가능한 action list 요청
  ```
  rospy example/testRequestActionInfo.py 
  ```

- action 정보(precondition, effect, parameter) 요청
  ```
  rospy example/testRequestActionInfo.py 
  ```

- Compound action -> Primitive action decompose 요청
  ```
  rospy example/testDecodeAction.py 
  ```

## Nodes

### {Node1 Name}

<div style="padding-left:40px;">


#### Messages

- Action.msg
  - string (`name`)
    - action's name
  - string[] (`type`)
    - action's type (ALL, AVAILABLE_ACTIONS) : 하드웨어 플랫폼에 따라 선택
  - string[] (`parameters`)
    - action's parameters
  - string[] (`values`)
    - action's symbolic value
  - string[] (`primitives`)
    - action이 compound action일 경우, 하위 레벨의 primitive action list
  - string[] (`controller`)
    - action이 primitive action일 경우, 수행하는데 필요한 controller 종류(e.g. 'position')
  - string[] (`group`)
    - action 을 수행하는데 필요한 robot_group (e.g. 'mobile', 'left_arm', 'right_arm')
  - string[] (`planner`)
    - motion plan에 필요한 behavior module 이름
  - socialrobot_actionlib/Condition (`precondition`)
    - action을 수행하는데 필요한 predicate state
  - socialrobot_actionlib/Condition (`effect`)
    - action 수행 후 predicate state

- Condition.msg
  - negatives (`socialrobot_actionlib/Fluent[]`)
    - Negative condition list
  - positives (`socialrobot_actionlib/Fluent[]`)
    - Positive condition list

- Fluent.msg
  - predicate (`string`)
    - predicate type
  - args (`string[]`)
    - predicate arguments

- Problem.msg
  - domain_name (`string`)
    - Task domain name
  - objects (`diagnostic_msgs/KeyValue[] objects`)
    - Pre-defined objects list
  - facts (`socialrobot_actionlib/Predicate[]`)
    - Predicate list of current state
  - goals (`socialrobot_actionlib/Predicate[]`)
    - Predicate list of goal state
  
#### Services

- /actionlib/get_action_list ([socialrobot_actionlib/GetActionList](./srv/GetActionList.srv))
  - 로봇이 수행가능한 action list 요청

<div style="display:flex; padding-left:50px">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

Request

- action_name (`string`)
  - Action 이름
- params (`string[]`)
  - action pddl parameters
  - 
</div>
<div style="flex:50%; padding-left:10px;">

Response

- result (`bool`)
  - 서비스 요청 결과
- action (`socialrobot_actionlib/Action`)
  - action PDDL model

</div>
</div>

- /actionlib/get_action_info ([socialrobot_actionlib/GetActionInfo](./srv/GetActionInfo.srv))
  - `config/action.library.pddl`에 정의된 특정 action model에 대한 정보 요청 

<div style="display:flex; padding-left:50px">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

Request

- action_type (`string`)
  - ALL=1 , AVAILABLE_ACTIONS=2
  - 
</div>
<div style="flex:50%; padding-left:10px;">

Response

- actions (`std_msgs/String[]`)
  - action lists

</div>
</div>

- /actionlib/get_domain ([socialrobot_actionlib/GetDomain](./srv/GetDomain.srvl))
  - 현재 로봇 플랫폼에 대한 pddl domain 요청


<div style="display:flex; padding-left:50px">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

Request

- group_list (`string[]`)
  - 현재 로봇이 포함하는 robot_group 리스트
  
</div>
<div style="flex:50%; padding-left:10px;">

Response

- result (`bool`)
  - service respond result
- actions (`string`)
  - compressed key value of PDDL actions
- types (`string`)
  - compressed key value of PDDL types
- predicates (`string`)
  - compressed key value of PDDL predicates
- requirements (`string[]`)
  - string array of PDDL requirements

</div>
</div>

- /actionlib/decode_action ([socialrobot_actionlib/GetPrimitiveActionList](./srv/GetPrimitiveActionList.srv))
  - Compound action을 Primitive action으로 변환 요청
  
<div style="display:flex; padding-left:50px">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

Request

- compound_action (`socialrobot_actionlib/Action`)
  - Compound action 
  
</div>
<div style="flex:50%; padding-left:10px;">

Response

- result (`socialrobot_actionlib/Action[]`)
  - Primitive action lists

</div>
</div>

#### Parameters

- 

</div>

---

- [[Go to the Social Robot Project Main]][SRP_main]
