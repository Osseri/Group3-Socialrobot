// ROS
#include <ros/ros.h>
#include <pluginlib/class_loader.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// ROS message
#include <socialrobot_motion/MotionPlan.h>
#include <sensor_msgs/JointState.h>

// motion base
#include "motion.h"

namespace socialrobot_motion
{

class GripperOpenClose: public MotionBase
{
public:
    GripperOpenClose();
    ~GripperOpenClose();
    virtual void update_scene(socialrobot_motion::MotionPlan::Request& req);
    virtual void compute_path(socialrobot_motion::MotionPlan::Request& req);
    virtual void visualize_trajectory(void);

    void joint_callback(const sensor_msgs::JointState::ConstPtr& msg);
    sensor_msgs::JointState joint_state;


private:
    // node handle
    ros::NodeHandle nh;

    //optional: visualize
    ros::Publisher display_publisher;
    moveit_msgs::DisplayTrajectory display_trajectory;

    ros::Subscriber sub;
    // manage collision object ids
    //std::set<std::string> collision_ids;
};

}