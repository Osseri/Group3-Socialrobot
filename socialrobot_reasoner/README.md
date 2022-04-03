# Social Robot Metric Reasoner

<!-- Variables -->
[SRP_main]: https://gitlab.com/social-robot/socialrobot

- Version 1.0.0
- [[Go to the Social Robot Project Main]][SRP_main]

---

<div style="display:flex;">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

**Package summary**

The meta-package of metric reasoner for the social robot project.

- Maintainer status: maintained
- Maintainers
  - Jeongmin Jeon (nicky707@daum.net)
  - Hong-ryul Jung (jung.hr.1206@gmail.com)
  - Hyungpil Moon (hyungpil@skku.edu)
- Author
  - Jeongmin Jeon (nicky707@daum.net)
- Source: git https://gitlab.com/social-robot/socialrobot_reasoner.git

</div>
<div style="flex:40%; padding-left:10px;">

**Table of Contents**
- [Social Robot Metric Reasoner](#social-robot-metric-reasoner)
  - [Overview](#overview)
  - [Contents](#contents)

</div>
</div>

## Overview

The meta-package of metric reasoner for the social robot project.

```mermaid
graph LR

subgraph socialrobot_reasoner
a[socialrobot_relocation]
end

socialrobot_interface --- a
```

## Contents

- [rearrange_node](https://gitlab.com/social-robot/socialrobot_reasoner/-/tree/master/rearrange_node)
  - Services
    - ~/rearrange_srv (rearrange_node/rearrange_env_srv.srv)
- [relocation_node](https://gitlab.com/social-robot/socialrobot_reasoner/-/tree/master/relocation_node)
  - Services
    - ~/relocation_srv (relocation_node/relocate_env_srv.srv)

---

- [[Go to the Social Robot Project Main]][SRP_main]