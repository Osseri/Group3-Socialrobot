string name

# body type
int32[] robot_group
int32 WHOLE_BODY = 0
int32 LEFT_ARM = 1
int32 RIGHT_ARM = 2
int32 LEFT_ARM_WITHOUT_WAIST = 12
int32 RIGHT_ARM_WITHOUT_WAIST = 22
int32 BOTH_ARM = 3
int32 LEFT_GRIPPER = 4
int32 RIGHT_GRIPPER = 8
int32 BOTH_GRIPPER = 16
int32 MOBILE_BASE = 32

string[] constraints

# Position-metric
Position[] goal_position
Position current_position

# Object-metric
Object[] target_object
Object[] static_object
Object[] dynamic_object

