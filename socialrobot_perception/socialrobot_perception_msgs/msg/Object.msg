# Object 
std_msgs/Header header
std_msgs/String name
std_msgs/String type

# Bounding box
vision_msgs/BoundingBox2D bb2d
vision_msgs/BoundingBox3D bb3d
geometry_msgs/PoseWithCovariance pose

# Affordance
int32 label

# Grasp points
float64[] grasp_points
