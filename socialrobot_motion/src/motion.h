#ifndef _MOTION_H
#define _MOTION_H

// ROS
#include <ros/ros.h>


// ROS message
#include <socialrobot_motion/MotionPlan.h>

namespace socialrobot_motion
{

class MotionBase
{
public:
    virtual bool plan_motion(socialrobot_motion::MotionPlan::Request& req,
                    socialrobot_motion::MotionPlan::Response& res)
    {
        ROS_INFO("plan motion...");

        // main process
        update_scene(req);
        compute_path(req);
        visualize_trajectory();

        res.planResult = plan_status;
        res.jointTrajectory = joint_trajectory;

        return true;
    }

    // pure virtual function
    virtual void update_scene(socialrobot_motion::MotionPlan::Request& req) = 0;

    virtual void compute_path(socialrobot_motion::MotionPlan::Request& req) = 0;

    virtual void visualize_trajectory(void) = 0;

protected:
    //static const std::string MOTION_NAME;
    int32_t plan_status;
    trajectory_msgs::JointTrajectory joint_trajectory;
    
};

}
#endif