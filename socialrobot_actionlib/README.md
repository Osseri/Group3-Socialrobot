# socialrobot_actionlib

<!-- Variables -->
[SRP_main]: https://gitlab.com/social-robot/socialrobot

- Version 1.0.0
- [[Go to the Social Robot Project Main]][SRP_main]

---

<div style="display:flex;">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

**Package summary**

Action Library 모듈은 지식수준의 action model 정의와 로봇 모션계획 및 제어와 연관관계를 관리

- Maintainer status: maintained
- Maintainers
  - Jeongmin Jeon (nicky707@daum.net)
  - Hong-ryul Jung (jung.hr.1206@gmail.com)
  - Hyungpil Moon (hyungpil@skku.edu)
- Author
  - Jeongmin Jeon (nicky707@daum.net)
- License: {License Name}
- Source: git https://gitlab.com/social-robot/socialrobot_actionlib.git

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
5. [Features](#features)
   1. [Example](#example)
6. [Nodes](#nodes)
   1. [{Node1 Name}](#node1-name)

</div>
</div>

---

## Overview

Action Library 모듈은 지식수준의 action model 정의와 로봇 모션계획 및 제어와 연관관계를 관리

- move_arm
- grasp_object
- push_object
- rotate_object
- etc.

## Installation methods

### Install manually

1. Install the ROS melodic. [Instructions for Ubuntu 18.04](http://wiki.ros.org/melodic/Installation/Ubuntu)  
2. [Setup your ROS environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
3. Install [rdflib](https://github.com/RDFLib/rdflib) python library
    ```
    pip install rdflib --user
    ```
4. make and launch 
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

- [rdflib](https://github.com/RDFLib/rdflib)
  - RDFLib is a Python library for working with RDF, a simple yet powerful language for representing information.

### Social Robot Project Modules

- socialrobot_description (for initialization)

### Hardware requirements

This package does not require any hardware device.

## Quick start 

1. Lorem ipsum dolor sit amet, consectetur adipisicing elit
1. Lorem ipsum dolor sit amet, consectetur adipisicing elit
1. Lorem ipsum dolor sit amet, consectetur adipisicing elit

## Features

Lorem ipsum dolor sit amet, consectetur adipisicing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.

### Example

Lorem ipsum dolor sit amet, consectetur adipisicing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.

## Nodes

### {Node1 Name}

<div style="padding-left:40px;">

#### Subscribed Topics

- ~<some_name>/<topic_name> ([geometry_msgs/Point](http://docs.ros.org/api/geometry_msgs/html/msg/Point.html))
  - Lorem ipsum dolor sit amet, consectetur adipisicing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.
- ~<some_name>/<topic_name> ([geometry_msgs/Point](http://docs.ros.org/api/geometry_msgs/html/msg/Point.html))
  - Lorem ipsum dolor sit amet, consectetur adipisicing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.

#### Published Topics

- ~<some_name>/<topic_name> ([geometry_msgs/Point](http://docs.ros.org/api/geometry_msgs/html/msg/Point.html))
  - Lorem ipsum dolor sit amet, consectetur adipisicing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.

#### Messages

- {message_name}.msg
  - header (`Header`)
    - Standard metadata for higher-level stamped data types.
  - paramA (`type`)
    - Lorem ipsum dolor sit amet, consectetur adipisicing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.
  - paramB (`type`)
    - Lorem ipsum dolor sit amet, consectetur adipisicing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.

#### Services

- {Service Name} (pkg_name/srv_file.srv)
  - Lorem ipsum dolor sit amet, consectetur adipisicing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.

<div style="display:flex; padding-left:50px">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

Request

- InputParamX (`float64[]`)
  - Lorem ipsum dolor sit amet, consectetur adipisicing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.
- InputParamY (`float64[]`)
  - Lorem ipsum dolor sit amet, consectetur adipisicing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.

</div>
<div style="flex:50%; padding-left:10px;">

Response

- OutputParam1 (`int64`)
  - Lorem ipsum dolor sit amet, consectetur adipisicing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.
- OutputParam2 (`int64[]`)
  - Lorem ipsum dolor sit amet, consectetur adipisicing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.

</div>
</div>

#### Services Called

- ~<some_name>/<service_name> ([nav_msgs/GetMap](http://docs.ros.org/api/nav_msgs/html/srv/GetMap.html))
  - This node calls this service to do {something}.

#### Parameters

- ~parameter_name (int, default: 100)
  - Lorem ipsum dolor sit amet, consectetur adipisicing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.
- ~parameter_name (int, default: 100)
  - Lorem ipsum dolor sit amet, consectetur adipisicing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.

</div>

---

- [[Go to the Social Robot Project Main]][SRP_main]
