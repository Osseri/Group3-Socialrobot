# socialrobot_perception

<!-- Variables -->
[SRP_main]: https://gitlab.com/social-robot/socialrobot

- Version 1.0.0
- [[Go to the Social Robot Project Main]][SRP_main]

---

<div style="display:flex;">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

**Package summary**
ROS meta-package for perception modules.

- Maintainer status: maintained
- Maintainers
  - Jeongmin Jeon (nicky707@daum.net)
  - Hong-ryul Jung (jung.hr.1206@gmail.com)
  - Hyungpil Moon (hyungpil@skku.edu)
- Author
  - Jeongmin Jeon (nicky707@daum.net)
- Source: git https://gitlab.com/social-robot/socialrobot_perception.git

</div>
<div style="flex:40%; padding-left:10px;">

**Table of Contents**
- [socialrobot_perception](#socialrobot_perception)
  - [Overview](#overview)
  - [Installation methods](#installation-methods)
    - [Install using the Docker images](#install-using-the-docker-images)
  - [Dependencies](#dependencies)
    - [Frameworks](#frameworks)
    - [Third-party libraries](#third-party-libraries)
  - [hardware requirements](#hardware-requirements)
  - [Quick start](#quick-start)
  - [Nodes](#nodes)
    - [Aff_node](#aff_node)
      - [Subscribed topics](#subscribed-topics)
      - [Published topics](#published-topics)
    - [gen_grasp_node](#gen_grasp_node)
      - [Subscribed topics](#subscribed-topics-1)
      - [Published topics](#published-topics-1)
  - [Messages](#messages)
    - [/obj_aff](#obj_aff)
    - [/gr_info](#gr_info)
    - [/obj_info](#obj_info)

</div>
</div>

---

## Overview

This package detect affordances of objects and generate grasp information using grasp affordance and point clouds. 


## Installation methods

### Install using the Docker images
`$ docker pull kape67/sr3_affnet_grsp:1.0.0`

Lorem ipsum dolor sit amet, consectetur adipisicing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua.

## Dependencies

### Frameworks

- ROS Kinetic/Melodic

### Third-party libraries
- caffe
- opencv

## hardware requirements
The GPU ram must be over 8GB.

## Quick start
Launch camera package, then luanch perception module
```
$roslaunch realsense2_camera rs_rgbs.launch
$ roslaunch obj_detect_3d obj_pose.launch
```

## Nodes

### Aff_node

#### Subscribed topics
- /kinect2/qhd/image_color ([sensor_msgs/Image])
   - To extract affordances of objects, use RGB data

#### Published topics
- /aff_det_img ([sensor_msgs/Image])
   - Show detected bounding boxes and mask of objects
- /aff_img ([sensor_msgs/Image])
   - Show detected affordance mask
- /obj_aff ([obj_msg/AffArray])
   - id, score, bounding box, affordance mask and affordance mask score per object

### gen_grasp_node

#### Subscribed topics
- /obj_aff ([obj_msg/AffArray])
   - duplicate

#### Published topics
- /obj_info([obj_msg/ObjInfoArrayMsg])
   - id, score, 8 points of bounding volume, 3-axis scale of bounding volume and 3-axis unit vector of bounding volume per objects
- /gr_info([obj_msg/GraspArray])
   - The important information when grasping
- /grsp_point_cloud([sensors_msgs/PointCloud2])
   - point clouds of grasp affordances

## Messages
Main published topics
- /obj_aff : Information about affordance 
- /gr_info : Information needed when grasping an object
- /obj_info : Grasp affordance information of object (3d)

### /obj_aff 
obj_aff : AffArray.msg 

AffArray.msg :  std_msgs/Header 

                AffBboxElem[] ( AffBboxElem[] : the array of AffBboxElem.msg  )

AffBboxElem.msg :   id ( object id )

                    score ( probability of the id )

                    bb_lt_p ( x coordiante of left top point of bounding box. pixel unit ) 

                    bb_lt_q ( y coordiante of left top point of bounding box. pixel unit )

                    bb_rb_p ( x coordiante of right bottom point of bounding box. pixel unit )

                    bb_rb_q ( y coordiante of right bottom point of bounding box. pixel unit )

                    aff_mask ( mask map which has affordance id per img pixel of bounding box )

                    aff_score_mask ( probability of aff id per img pixel of bounding box )


### /gr_info 

gr_info : GraspArray.msg 

GraspArray.msg  :   std_msgs/Header

                    GraspMsg[] gr ( GraspMsg[] : the list of GraspMsg.msg)

GraspMsg.msg    :   num_type

                    id ( object id )

                    grasp_cx ( grasp affordance center x. meter unit )

                    grasp_cy ( grasp affordance y. meter unit )

                    grasp_cz ( grasp affordance z. meter unit )

                    GraspElem[] gr_elements ( GraspElem[] : the list of GraspElem.msg )

GraspElem.msg   :   grasp_0x, y, z ( the direction in which **the** robot **finger** approaches the object )

                    grasp_1x, y, z ( the direction in which **the other** robot **finger** approaches the object )
                    
                    grasp_appx, y, z ( the direction in which robot **hand** approaches the object ) 

### /obj_info

obj_info : ObjInfoArrayMsg.msg

ObjInfoArrayMsg.msg :   std_msgs/Header

                        Obj3dInfo[] pt ( Obj3dInfo[] : the list of Obj3dInfo.msg )

Obj3dInfo.msg  :  id ( object id )
                  score ( probability of the id)
                  bb_pts ( 8 points coordinates of grasp affordance 3d bounding box , ([geometry_msgs/Point[8]]) ) 
                  bb_uv ( 3-axis unit vector of grasp affordance 3d bounding box, ([geometry_msgs/Vector3[3]]))
                  bb_sc ( 3-axis scale of vector of grasp affordance 3d bounding box, ([geometry_msgs/Vector3]))

</div>


---

- [[Go to the Social Robot Project Main]][SRP_main]
