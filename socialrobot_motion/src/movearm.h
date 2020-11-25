// ROS
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// ROS message
#include <socialrobot_motion/MotionPlan.h>

// motion base
#include "motion.h"

namespace socialrobot_motion
{

class MoveArm: public MotionBase
{
public:
    MoveArm();
    ~MoveArm();
    virtual void update_scene(socialrobot_motion::MotionPlan::Request& req);
    virtual void compute_path(socialrobot_motion::MotionPlan::Request& req);
    virtual void visualize_trajectory(void);

private:
    // node handle
    ros::NodeHandle nh;

    // for moveit::move_group
    moveit::planning_interface::MoveGroupInterface leftarm_group;
    moveit::planning_interface::MoveGroupInterface rightarm_group;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface; 

    // plan result
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    //optional: visualize
    ros::Publisher display_publisher;
    moveit_msgs::DisplayTrajectory display_trajectory;

    // manage collision object ids
    std::set<std::string> collision_ids;
    std::string left_attached_object;
    std::string right_attached_object;

    //robot_model_loader::RobotModelLoader robot_model_loader_;
    //robot_model::RobotModelPtr robot_model_;
    //planning_scene::PlanningScenePtr planning_scene_;
    //planning_pipeline::PlanningPipelinePtr planning_pipeline_;

    //const robot_state::JointModelGroup* left_joint_model_group;
    //const robot_state::JointModelGroup* right_joint_model_group;

    //const std::string left_joint_group_name;
    //const std::string right_joint_group_name;
    //std::string left_eef_name;
    //std::string right_eef_name;
};

}