#ifndef MOVEITCONTROLLER_H
#define MOVEITCONTROLLER_H
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>

class ArmController
{
public:
  ArmController(std::string name_, ros::NodeHandle& nh);
  void compute();

private:
  void ArmGoalCB();
  void ArmPreemptCB();

  sensor_msgs::JointState joint_command_msg;
  ros::Publisher joint_command_pub;
  ros::NodeHandle nh_;
  
  std::string action_name_;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  std::vector<trajectory_msgs::JointTrajectoryPoint> joint_trajectory_;


  ros::Time goal_start_time;
  ros::Duration traj_time;

  control_msgs::FollowJointTrajectoryFeedback feedback_;
  control_msgs::FollowJointTrajectoryResult result_;
  control_msgs::FollowJointTrajectoryGoalConstPtr goal_;

  int feedback_header_stamp_;
  int as_joint_size;
};

class GripperController
{
public:
  GripperController(std::string name_, ros::NodeHandle& nh);

  void compute();
private:
  void gGoalCB();
  void gPreemptCB();

  ros::Time goal_start_time;
  ros::Duration traj_time;

  sensor_msgs::JointState joint_command_msg;
  ros::Publisher joint_command_pub;
  ros::NodeHandle nh_;

  std::string action_name_;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;

  control_msgs::FollowJointTrajectoryFeedback feedback_;
  control_msgs::FollowJointTrajectoryResult result_;
  control_msgs::FollowJointTrajectoryGoalConstPtr goal_;
  
  int as_joint_size;
};

#endif // MOVEITCONTROLLER_H
