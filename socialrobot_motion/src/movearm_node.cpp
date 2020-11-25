// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "socialrobot_motion_movearm");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    SocialMotionPlanner smp;

    ros::Rate r(10);
    while(ros::ok())
    {r.sleep();}

    return 0;
}