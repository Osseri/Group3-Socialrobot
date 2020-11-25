# Relocation Node

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
6. [Nodes](#nodes)
   1. [relocate_planner_node](#relocate_planner_node)
      1. [Subscribed Topics](#subscribed-topics)
      2. [Published Topics](#published-topics)
      3. [Messages](#messages)
      4. [Services](#services)

</div>
</div>

---

## Overview

relocation_node

```mermaid
sequenceDiagram

other ->>+ relocate_planner: relocate_env_srv.srv.Request
Note right of relocate_planner: relocation_task_planner.py

relocate_planner -->>- other: relocate_env_srv.srv.Response
```

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

.

## Features

.

## Nodes

### relocate_planner_node

relocation_task_planner.py

#### Subscribed Topics

- None

#### Published Topics

- None

#### Messages

- None

#### Services

- ~/relocation_srv (relocation_node/relocate_env_srv.srv)

<div style="display:flex; padding-left:50px">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

Request

- robot_height (`float64`)
- robot_pose (`float64[]`)
- target_id (`int64`)
- N (`int64`)
- R (`float64[]`)
- H (`float64[]`)
- X (`float64[]`)
- Y (`float64[]`)
- x_min (`float64`)
- x_max (`float64`)
- y_min (`float64`)
- y_max (`float64`)

</div>
<div style="flex:50%; padding-left:10px;">

Response

- accessibility (`int64`)
- relocate_id (`int64`)
- relocate_coordinates (`float64[]`)

</div>
</div>

---

- [[Go to the Social Robot Project Main]][SRP_main]
