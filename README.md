# Social Robot Project Group

<!-- Variables -->

- Version 1.0.0

---

**Package summary**

�Ҽȷκ� ������Ʈ�� ���� repository�Դϴ�. ���� ����, �׽�Ʈ �� ���� �뵵�� ���˴ϴ�.

**Table of Contents**
- [Social Robot Project Group](#social-robot-project-group)
  - [Overview](#overview)
  - [Installation methods](#installation-methods)
    - [Install manually](#install-manually)
      - [1. ROS](#1-ros)
      - [2. �߰� ROS ��Ű�� ��ġ](#2-�߰�-ros-��Ű��-��ġ)
      - [3. GRASPIT](#3-graspit)
      - [4. GraspIt ROS](#4-graspit-ros)
      - [5. Social robot ROS ��Ű�� ��ġ](#5-social-robot-ros-��Ű��-��ġ)
      - [6. V-REP](#6-v-rep)
  - [Dependencies](#dependencies)
    - [Frameworks](#frameworks)
    - [Third-party libraries](#third-party-libraries)
  - [Quick start (Simualtion Demo)](#quick-start-simualtion-demo)

---

## Overview

[![ROS](http://www.ros.org/wp-content/uploads/2013/10/rosorg-logo1.png)](http://www.ros.org/)

- �Ҽȷκ� ������Ʈ�� ���� repository�Դϴ�. ���� ����, �׽�Ʈ �� ���� �뵵�� ���˴ϴ�.
- Integration test�� ����� ������ �����մϴ�.
- ���� ����� ������ �ش� repository�� devel branch���� ��Ź�帳�ϴ�.

```mermaid
graph TB

subgraph socialrobot_interface
   socialrobot_task -->|service req.| socialrobot_actionlib
   socialrobot_actionlib -->|service resp: Action model| socialrobot_task

   socialrobot_task -->|? query/subscription| socialrobot_knowledge
   socialrobot_knowledge -->|? Inital state, Goal state| socialrobot_task

   socialrobot_task -->|? Action sequence| socialrobot_behavior
   socialrobot_actionlib -->|?| socialrobot_behavior
end

socialrobot_behavior -->|? ��� ��ȹ���| socialrobot_motion

socialrobot_motion -->|?| socialrobot_control
socialrobot_control --> vrep
socialrobot_control --> socialrobot_hardware

subgraph Robot in the real
socialrobot_hardware
end
subgraph Robot in the V-rep
vrep
end
```

## Installation methods

### Install manually

ROS �� GraspIt�� ������ ��ġ�Ǿ� �־�� �մϴ�.

#### 1. ROS

ros-kinetic �� ros-melodic �������� �׽�Ʈ �Ǿ����ϴ�.

���� ��ũ���� �ڼ��� ��ġ ����� Ȯ���� �� �ֽ��ϴ�. 
[ROS Install](http://wiki.ros.org/melodic/Installation/Ubuntu)

#### 2. �߰� ROS ��Ű�� ��ġ

```bash
apt install ros-$ROS_DISTRO-vision-msgs ros-$ROS_DISTRO-moveit
apt install ros-$ROS_DISTRO-navigation ros-$ROS_DISTRO-rosjava
apt install flex ros-$ROS_DISTRO-move-base-msgs ros-$ROS_DISTRO-nav-msgs ros-$ROS_DISTRO-tf2-bullet freeglut3-dev ros-$ROS_DISTRO-mongodb-store
```

ROSPLAN�� ���ؼ��� �Ʒ� ��Ű���� �ʿ���.
```
apt install ros-kinetic-navigation ros-kinetic-rosjava
apt install flex ros-kinetic-move-base-msgs ros-kinetic-nav-msgs ros-kinetic-tf2-bullet freeglut3-dev ros-kinetic-mongodb-store
```

#### 3. GRASPIT

���� ��Ű�� ��ġ
```
sudo apt install libqt4-dev libqt4-opengl-dev libqt4-sql-psql libcoin80-dev libsoqt4-dev libblas-dev liblapack-dev libqhull-dev libeigen3-dev
```

graspit �ҽ� clone
```bash
git clone https://github.com/graspit-simulator/graspit.git
```

���� & ��ġ
```
cd graspit
mkdir build
cd build
cmake ..
make -j5
sudo make install
```

bash�� ����Ѵٸ� ~/.bashrc ���� �ȿ� �Ʒ� ��ɾ �߰��ϸ� �˴ϴ�. (zsh�� ����ϸ� ~/.zshrc)
```
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export GRASPIT=~/.graspit
```

������ �����ϱ� ���� �Ʒ� ��ɾ �����մϴ�.
```
source ~/.bashrc
```

#### 4. GraspIt ROS

GraspIt�� ��ġ�� �Ŀ� ROS���� �� �� �ֵ��� [�������̽�](https://github.com/graspit-simulator/graspit_interface#ros-setup)�� �߰��ؾ� �մϴ�.


```
//clone packages into catkin workspace
git clone https://github.com/graspit-simulator/graspit_interface.git
git clone https://github.com/graspit-simulator/graspit_commander.git

//build workspace
catkin_make
```

#### 5. Social robot ROS ��Ű�� ��ġ

�Ҽȷκ� ������Ʈ�� ������ ROS ��Ű������ �ʿ���մϴ�.

- socialrobot_actionlib
- socialrobot_behavior
- socialrobot_hardware
- socialrobot_interface
- socialrobot_knowledge
- socialrobot_motion
- socialrobot_perception
- socialrobot_reasoner
- socialrobot_task

#### 6. V-REP

socialrobot ������Ʈ�� CoppeliaSim(�� v-rep) �ùķ����͸� ����մϴ�.(version 4.0.0 �̻�)
�ùķ��̼� �𵨵��� [Vortex engine](https://www.coppeliarobotics.com/helpFiles/en/dynamicsModule.htm) ���̼����� �ʿ���մϴ�.
[CM-labs](https://www.cm-labs.com/vortex-studio/)���� ���� �� ������ ���̼����� �߱޹��� �� �ֽ��ϴ�.


�ٿ�ε�: [CoppeliaSim Linux](http://www.coppeliarobotics.com/downloads.html)

���� ����
```
tar -xvzf {file_name}
```

����
```
cd {v-rep directory path}
./vrep.sh
```

ROS�� �����ϱ� ���� socialrobot ������Ʈ�� remote api�� ����մϴ�. Ȱ��ȭ�ϱ� ���� vrep ���丮 ���� *remoteApiConnections.txt* ������ �����մϴ�.
```
portIndex1_port             = 19997
```

## Dependencies

### Frameworks

- ROS Kinetic/Melodic

### Third-party libraries

TBA

## Quick start (Simualtion Demo)

1. `roscore` ����
2. V-REP ����
    1. vrep remote api�� �ùķ��̼��� ����ǹǷ�, roscore�� ���� ����Ǿ����
3. `sudo service mongodb stop`
4. ���� �ʿ��� ������ roslaunch�� ����
    ```
    roslaunch socialrobot_interface init.launch
    ```

    �κ� �ϵ����� SKKUrobot�� RobocareRobot 2������ ������ robot_name �Ķ���ͷ� ���氡��
    ```
    # SKKURobot
    roslaunch socialrobot_state demo_init.launch robot_name:=skkurobot

    # RobocareRobot
    roslaunch socialrobot_state demo_init.launch robot_name:=social_robot
    ```
5. Task������ ���� State machine ����
    ```
    rosrun socialrobot_interface demo.launch
    ```
6. Service ��ɾ �̿��� ���� �۾� ��û
    '''
	rosservice call /socialrobot/set_command 'default' '{}'
    '''
