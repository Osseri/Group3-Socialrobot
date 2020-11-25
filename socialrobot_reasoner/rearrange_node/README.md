# Rearrange Task Planner

<!-- Variables -->
[SRP_main]: https://gitlab.com/social-robot/socialrobot

- Version 1.0.0
- [[Go to the Social Robot Project Main]][SRP_main]

---

<div style="display:flex;">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

**Package summary**

This package determines the object to be relocated in order to clear all obstacles that prevent grasping a target object in clutter.

- Maintainer status: maintained
- Maintainers
  - Sang Hun Cheong (welovehun@kist.re.kr)
  - Jinhwi Lee (jinhooi@kist.re.kr)
  - Changjoo Nam (cjnam@kist.re.kr)
- Author
  - Sang Hun Cheong (welovehun@kist.re.kr)
- License: {License Name}
- Source: git https://gitlab.com/social-robot/socialrobot_reasoner.git

</div>
<div style="flex:40%; padding-left:10px;">

**Table of Contents**
1. [Overview](#overview)
2. [Installation methods](#installation-methods)
3. [Dependencies](#dependencies)
   1. [Frameworks](#frameworks)
   2. [Third-party libraries](#third-party-libraries)
   3. [Social Robot Project Modules](#social-robot-project-modules)
   4. [Hardware requirements](#hardware-requirements)
4. [Quick start](#quick-start)
5. [Features](#features)
   1. [Example Input test](#example-input-test)
6. [Nodes](#nodes)
   1. [rearrange_node](#rearrange_node)

</div>
</div>

---

## Overview

This package determines the object to be relocated in order to clear all obstacles that prevent grasping a target object in clutter.

<!-- 
```mermaid
```
 -->

## Installation methods

.

## Dependencies

### Frameworks

- ROS Kinetic/Melodic

### Third-party libraries

- .

### Social Robot Project Modules

- .

### Hardware requirements

This package does not require any hardware device.

## Quick start

1. Install the package through catkin build system.
2. ```rosrun rearrange_node rearrange_task_planner.py```
3. run test_module file(`./src/test_module.py`)

## Features

### Example Input test

**./src/test_module.py**

> /usr/bin/python2.7 ~/catkin_ws/src/rearrange_node/src/test_module.py

```
#!/usr/bin/env python

import rospy
from rearrange_node.srv._rearrange_env_srv import *
from rearrange_node.msg._env_object_info_msg import *

def example():
    rospy.wait_for_service('rearrange_srv')
    try:
        f_check_srv = rospy.ServiceProxy('rearrange_srv', rearrange_env_srv)
        pub_msg = rearrange_env_srvRequest()

        pub_msg.target.object_name = ['target']
        pub_msg.target.object_position.x, pub_msg.target.object_position.y, pub_msg.target.object_position.z = 0.9, 0.05, 1.0 + 0.1
        pub_msg.target.object_orientation.x, pub_msg.target.object_orientation.y, pub_msg.target.object_orientation.z, pub_msg.target.object_orientation.w = 0.0, 0.0, 0.0, 0.0
        pub_msg.target.object_scale.x, pub_msg.target.object_scale.y, pub_msg.target.object_scale.z = 0.06, 0.06, 0.2

        for i in range(3):
            obs_tmp=env_object_info_msg()
            pub_msg.objects.append(obs_tmp)

        pub_msg.objects[0].object_name = ['obj1']
        pub_msg.objects[0].object_position.x, pub_msg.objects[0].object_position.y, pub_msg.objects[0].object_position.z = 0.75, 0.1, 1.0 + 0.1
        pub_msg.objects[0].object_orientation.x, pub_msg.objects[0].object_orientation.y, pub_msg.objects[0].object_orientation.z, pub_msg.objects[0].object_orientation.w = 0.0, 0.0, 0.0, 0.0
        pub_msg.objects[0].object_scale.x, pub_msg.objects[0].object_scale.y, pub_msg.objects[0].object_scale.z = 0.06, 0.06, 0.2

        pub_msg.objects[1].object_name = ['obj2']
        pub_msg.objects[1].object_position.x, pub_msg.objects[1].object_position.y, pub_msg.objects[1].object_position.z = 0.75, 0.0, 1.0 + 0.1
        pub_msg.objects[1].object_orientation.x, pub_msg.objects[1].object_orientation.y, pub_msg.objects[1].object_orientation.z, pub_msg.objects[1].object_orientation.w = 0.0, 0.0, 0.0, 0.0
        pub_msg.objects[1].object_scale.x, pub_msg.objects[1].object_scale.y, pub_msg.objects[1].object_scale.z = 0.06, 0.06, 0.2

        pub_msg.objects[2].object_name = ['obj3']
        pub_msg.objects[2].object_position.x, pub_msg.objects[2].object_position.y, pub_msg.objects[2].object_position.z = 0.75, -0.1, 1.0 + 0.1
        pub_msg.objects[2].object_orientation.x, pub_msg.objects[2].object_orientation.y, pub_msg.objects[2].object_orientation.z, pub_msg.objects[2].object_orientation.w = 0.0, 0.0, 0.0, 0.0
        pub_msg.objects[2].object_scale.x, pub_msg.objects[2].object_scale.y, pub_msg.objects[2].object_scale.z = 0.06, 0.06, 0.2

        pub_msg.workspace.object_name = ['table']
        pub_msg.workspace.object_position.x, pub_msg.workspace.object_position.y, pub_msg.workspace.object_position.z = 0.7, 0.0, 0.5
        pub_msg.workspace.object_orientation.x, pub_msg.workspace.object_orientation.y, pub_msg.workspace.object_orientation.z, pub_msg.workspace.object_orientation.w = 0.0, 0.0, 0.0, 0.0
        pub_msg.workspace.object_scale.x, pub_msg.workspace.object_scale.y, pub_msg.workspace.object_scale.z = 0.5, 0.8, 1.0

        resp1 = f_check_srv(pub_msg)
        # f_ori[0]
        print resp1

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == '__main__':
    print "Example"
    example()
```

Example return:

```
object_name: [obj1]
rearrange_positions: 
  - 
    x: 0.62
    y: 0.31
    z: 1.1
  - 
    x: 0.73
    y: 0.28
    z: 1.1
  - 
    x: 0.66
    y: -0.07
    z: 1.1
  - 
    x: 0.88
    y: 0.26
    z: 1.1
  - 
    x: 0.84
    y: -0.19
    z: 1.1
  - 
    x: 0.81
    y: -0.33
    z: 1.1
  - 
    x: 0.51
    y: 0.04
    z: 1.1

Process finished with exit code 0
```

## Nodes

### rearrange_node

rearrange_task_planner.py

<div style="padding-left:40px;">

#### Subscribed Topics

- None

#### Published Topics

- None

#### Messages

- env_object_info_msg.msg
  - header (`Header`)
    - Standard metadata for higher-level stamped data types.
  - object_name (`string[]`)
    - The name of the object.
  - object_position (`geometry_msgs/Point`)
    - The position of the object.
  - object_orientation (`geometry_msgs/Quaternion`)
    - The orientation of the object in quaternions(x, y, z, w)
  - object_scale (`geometry_msgs/Vector3`)
    - The scale of the object in each x, y, z-axis.

#### Services

- ~/rearrange_srv (rearrange_node/rearrange_env_srv.srv)

<div style="display:flex; padding-left:50px">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

Request

- workspace (`env_object_info_msg`)
  - The workspace object (ex.table).
- target (`env_object_info_msg`)
  - The target object on the workspace object.
- objects (`env_object_info_msg[]`)
  - The obstacles on the workspace object.

</div>
<div style="flex:50%; padding-left:10px;">

Response

- object_name (`string[]`)
  - The object to be relocated.
- rearrange_positions (`geometry_msgs/Point[]`)
  - The list of positions that the object to be rearranged.

</div>
</div>



#### Services Called

- None

#### Parameters

- None

</div>

---

- [[Go to the Social Robot Project Main]][SRP_main]
