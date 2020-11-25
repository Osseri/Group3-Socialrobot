#include "gripper_openclose.h"

using namespace socialrobot_motion;

GripperOpenClose::GripperOpenClose(void):
    nh("~")
{
    sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, &GripperOpenClose::joint_callback, this);

    display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
}

GripperOpenClose::~GripperOpenClose(void)
{
    ;
}

void GripperOpenClose::joint_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    joint_state.name = msg->name;
    joint_state.position = msg->position;
}

void GripperOpenClose::update_scene(socialrobot_motion::MotionPlan::Request& req)
{
    ROS_INFO("(GripperOpenClose) update_scene...");

    //update current state(ROBOT)
    if(req.targetBody == req.LEFT_GRIPPER
        || req.targetBody == req.RIGHT_GRIPPER)
    {
        plan_status = socialrobot_motion::MotionPlan::Response::SUCCESS;
    }
    else
    {
        plan_status = socialrobot_motion::MotionPlan::Response::ERROR_INPUT;
    }

   
}

void GripperOpenClose::compute_path(socialrobot_motion::MotionPlan::Request& req)
{
    ROS_INFO("(GripperOpenClose) compute_path...");
    if(plan_status != 0)
        return;

    // interpolation
    bool result = true;
    sensor_msgs::JointState target_state = req.targetJointState;

    std::map<std::string, double>  current_state;
    for(size_t i=0; i<joint_state.name.size(); i++)
    {
        current_state[joint_state.name[i]] = joint_state.position[i];
    }
    

    trajectory_msgs::JointTrajectory traj;
    traj.joint_names = target_state.name;
    traj.points.resize(2); // sequence length is 3.

    for(size_t i=0; i<traj.joint_names.size(); i++)
    {
        double goal = target_state.position[i];
        double start = 0;
        if ( current_state.find(traj.joint_names[i]) == current_state.end() )
        {
            // not found
        } else {
            // found
            start = current_state.find(traj.joint_names[i])->second;
        }
        double mid = (start + goal)/2;

        traj.points[0].positions.push_back(start);
        //traj.points[1].positions.push_back(mid);
        traj.points[1].positions.push_back(goal);
    }


    
    // restore the result
    joint_trajectory.joint_names.clear();
    joint_trajectory.points.clear();
    joint_trajectory = traj;

    if(result)
        plan_status = socialrobot_motion::MotionPlan::Response::SUCCESS;
    else
        plan_status = socialrobot_motion::MotionPlan::Response::ERROR_NO_SOLUTION;
}

void GripperOpenClose::visualize_trajectory()
{
    //display_trajectory.trajectory_start = my_plan.start_state_;
    //display_trajectory.trajectory.push_back(my_plan.trajectory_);
    //display_publisher.publish(display_trajectory);
}