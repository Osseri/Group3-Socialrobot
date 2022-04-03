# socialrobot_msgs

<!-- Variables -->
[SRP_main]: https://gitlab.com/social-robot/socialrobot

- Version 1.0.0
- [[Go to the Social Robot Project Main]][SRP_main]

---

<div style="display:flex;">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

**Package summary**

ROS msgs packages for social-robot project.

- Maintainer status: maintained
- Maintainers
  - Jeongmin Jeon (nicky707@daum.net)
  - Hong-ryul Jung (jung.hr.1206@gmail.com)
  - Hyungpil Moon (hyungpil@skku.edu)
- Author
  - Jeongmin Jeon (nicky707@daum.net)
- Source: git https://gitlab.com/social-robot/socialrobot_msgs.git

</div>
<div style="flex:40%; padding-left:10px;">

**Table of Contents**
- [socialrobot_msgs](#socialrobot_msgs)
  - [Installation methods](#installation-methods)
    - [Install manually](#install-manually)
  - [Dependencies](#dependencies)
    - [Frameworks](#frameworks)
    - [Third-party libraries](#third-party-libraries)
      - [Messages](#messages)
      - [Services](#services)

</div>
</div>

---

## Installation methods

### Install manually

1. make
   ```
   catkin_make
   ```
## Dependencies

### Frameworks

- ROS Kinetic/Melodic

### Third-party libraries

- [vision_msgs](https://github.com/Kukanani/vision_msgs)


#### Messages

- [Behavior.msg](./msg/Behavior.msg)
  - name (`string`)
    - behavior planner name
  - robot_group (`int32[]`)
    - robot group name
    - WHOLE_BODY = 0
    - LEFT_ARM = 1
    - RIGHT_ARM = 2
    - LEFT_ARM_WITHOUT_WAIST = 12
    - RIGHT_ARM_WITHOUT_WAIST = 22
    - BOTH_ARM = 3
    - LEFT_GRIPPER = 4
    - RIGHT_GRIPPER = 8
    - BOTH_GRIPPER = 16
    - MOBILE_BASE = 32
  - constraints (`string[]`)
    - constraints for motion planning
  - goal_position ([`Position[]`](./msg/Position.msg))
    - geometry msgs for behavior goal
  - current_position ([`Position`](./msg/Position.msg))
    - geometry msgs for current robot pose
  - target_object ([`Object[]`](./msg/Object.msg))
    - object information for manipulate target object
  - static_object ([`Object[]`](./msg/Object.msg))
    - object information for static objects
  - dynamic_object ([`Object[]`](./msg/Object.msg))
    - object information for dynamic objects

- [Position.msg](./msg/Position.msg)
  - waypoint (`string`)
    - pre-defined position id
  - joint_state (sensor_msgs/JointState)
    - goal pose for robot manipulator
  - pose (`geometry_msgs/Pose`)
    -goal pose for robot mobile
  
- [Object.msg](./msg/Object.msg)
  - header (`std_msgs/Header`)
    - object header
  - id (`string`)
    - object id in dataset
  - type (`string`)
    - object class
  - mesh (`shape_msgs/Mesh`)
    - 3D mesh of object 
  - bb2d (`vision_msgs/BoundingBox2D`)
    - bounding box in 2D image
  - bb3d (`vision_msgs/BoundingBox3D`)
    - bounding box in 3D space
  - grasp_point (`geometry_msgs/Pose[]`)
    - approaching direction for grasping
  - affordance ([`socialrobot_msgs/Affordance[]`](./msg/Affordance.msg))
    - object's functionality information

- [Objects.msg](./msg/Objects.msg)
  - detected_objects ([`socialrobot_msgs/Object[]`](./msg/Object.msg))
    - object array

- [Affordance.msg](./msg/Affordance.msg)
  - type (`string`)
    - affordance id in dataset
  - mesh (`shape_msgs/Mesh`)
    - 3D mesh for object affordance
  - bb2d (`vision_msgs/BoundingBox2D`)
    - bounding box in 2D image
  - bb3d (`vision_msgs/BoundingBox3D`)
    - bounding box in 3D space

#### Services

  [socialrobot_msgs/SetTask](./srv/SetTask.srv)
  - request behavior module lists

<div style="display:flex; padding-left:50px">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

Request

- actionName (`string`)
  - available action name 
- actionParam1 (`string`)
  - action parameter
- actionParam2 (`string`)
  - action parameter
- actionParam3 (`string`)
  - action parameter
- actionParam4 (`string`)
  - action parameter
- actionParam5 (`string`)
  - action parameter
- actionConstraint (`string`)
  - action constaint
- actionID (`int32`)
  - action ID for ABBI framework
</div>
<div style="flex:50%; padding-left:10px;">

Response

- result (`int32`)
  - SUCCESS = 1
  - FAIL = 0

</div>
</div>


---

- [[Go to the Social Robot Project Main]][SRP_main]
