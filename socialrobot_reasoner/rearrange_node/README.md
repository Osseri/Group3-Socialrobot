# socialrobot_rearrangement

<!-- Variables -->
[SRP_main]: https://gitlab.com/Jinhwi/socialrobot_rearrangement

- Version 1.0.0
- [[Go to the Social Robot Project Main]][SRP_main]

---

<div style="display:flex;">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

**Package summary**

This package determines the object to be relocated in order to clear all obstacles that prevent grasping a target object in clutter.
- Maintainer status : maintained
- Maintainer :
  - Sang Hun Cheong (welovehun@kist.re.kr)
  - Jinhwi Lee (jinhooi@kist.re.kr)
  - Changjoo Nam (cjnam@kist.re.kr)
- Author :
  - Sang Hun Cheong (welovehun@kist.re.kr)
- License: {License Name}
- Source: git https://{Git URL}.git

</div>
<div style="flex:40%; padding-left:10px;">

**Table of Contents**
1. [Overview](#overview)
2. [Dependencies](#dependencies)
   1. [Frameworks](#frameworks)
   2. [Social Robot Project Modules](#social-robot-project-modules)
   3. [Hardware requirements](#hardware-requirements)
3. [Quick start](#quick-start)
   1. [Example Input test(src/src/test_module.py)](#example-input-testsrcsrctest_modulepy)
   2. [Return](#return)

</div>
</div>

---

## Overview

This package determines the rearrange position of the object to be relocated.

## Dependencies

### Frameworks

- ROS Kinetic/Melodic
  - ROS Moveit!

### Social Robot Project Modules

- socialrobot_perception

### Hardware requirements

The package does not require any hardware device.

## Quick start 
1. Install the package through catkin build system.
2. `rosrun rearrange_node rearrange_task_planner.py`
3. run test_module file(src/src/test_module.py)

### Example Input test(src/src/test_module.py)

```py
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

### Return

```sh
/usr/bin/python2.7 ~/catkin_ws/src/rearrange_node/src/test_module.py
Example
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

#### Service

- {rearrange_srv}(rearrange_node/rearrange_env_srv.srv)

##### Input/Service Request

- env_object_info_msg.msg<br>
  - header ('header')
    - Standard metadata for higher-level stamped data types.<br>
  - object_name ('string[]')  
    - the name of the object.<br>
  - object_position ('geometry_msgs/Point')
    - the position of the object.<br>
  - object_orientation ('geometry_msgs/Quaternion')  
    - the orientation of the object in quaternions(x, y, z, w).<br>
  - object_scale ('geometry_msgs/Vector3') 
    - the scale of the object in each x, y, z-axis.<br>

- rearrange_env_srv.srv (request)<br>
  - workspace ('env_object_info_msg')
    - the workspace object (ex.table).<br>
  - target ('env_object_info_msg')
    - the target object on the workspace object.<br>
  - objects ('env_object_info_msg[]') 
    - the obstacles on the workspace object.<br>

# 7. Output/Service Response

- rearrange_env_srv.srv (response)<br>
  - object_name ('string[]')
    - the object to be relocated.<br>
  - rearrange_positions ('geometry_msgs/Point[]')
    - the list of positions that the object to be rearranged.<br>

# 8. Parameters

N/A
