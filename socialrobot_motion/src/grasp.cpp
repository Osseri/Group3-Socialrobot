#include "grasp.h"

using namespace socialrobot_motion;

GraspObject::GraspObject(void):
    nh("~")
{
}

GraspObject::~GraspObject(void)
{
    ;
}


void GraspObject::update_scene(socialrobot_motion::MotionPlan::Request& req)
{
    ROS_INFO("(GraspObject) update_scene...");
  
}

void GraspObject::compute_path(socialrobot_motion::MotionPlan::Request& req)
{
    ROS_INFO("(MoveArm) compute_path...");
}

void GraspObject::visualize_trajectory()
{
   
}