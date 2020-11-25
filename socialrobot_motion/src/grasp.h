// ROS
#include <ros/ros.h>
#include <pluginlib/class_loader.h>

// ROS message
#include <socialrobot_motion/MotionPlan.h>

// motion base
#include "motion.h"

namespace socialrobot_motion
{

class GraspObject: public MotionBase
{
public:
    GraspObject();
    ~GraspObject();
    virtual void update_scene(socialrobot_motion::MotionPlan::Request& req);
    virtual void compute_path(socialrobot_motion::MotionPlan::Request& req);
    virtual void visualize_trajectory(void);

private:
    // node handle
    ros::NodeHandle nh;

};

}