#include <iostream>
#include <ros/ros.h>

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <socialrobot_motion/ArmPlanning.h>

namespace socialrobot_motion
{

class SocialMotionPlanner
{
public:
  SocialMotionPlanner() : nh_("~"), robot_model_loader_("robot_description")
  {
    init();
    planner_server_ = nh_.advertiseService("plan_arm", &SocialMotionPlanner::armPlanningCallback, this);
  }

private:
  void init()
  {
    robot_model_ = robot_model_loader_.getModel();

    planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));

    std::string planner_plugin_name = "ompl_interface/OMPLPlanner";

    try
    {
      planner_plugin_loader_.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
                                     "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try
    {
      planner_instance_.reset(planner_plugin_loader_->createUnmanagedInstance(planner_plugin_name));
      if (!planner_instance_->initialize(robot_model_, nh_.getNamespace()))
        ROS_FATAL_STREAM("Could not initialize planner instance");
      ROS_INFO_STREAM("Using planning interface '" << planner_instance_->getDescription() << "'");
    }
    catch (pluginlib::PluginlibException& ex)
    {
      const std::vector<std::string>& classes = planner_plugin_loader_->getDeclaredClasses();
      std::stringstream ss;
      for (std::size_t i = 0; i < classes.size(); ++i)
        ss << classes[i] << " ";
      ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                       << "Available plugins: " << ss.str());
    }

    display_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

  }


  bool armPlanningCallback(ArmPlanningRequest &request, ArmPlanningResponse& response)
  {
    // update current state
    sensor_msgs::JointState joints = request.start_state;
    robot_state::RobotState& robot_state = planning_scene_->getCurrentStateNonConst();
    for(int i=0; i<joints.name.size(); i++)
    {
      robot_state.setJointPositions(joints.name[i], &(joints.position[i]));
    }
    robot_state.update();

    planning_interface::MotionPlanRequest motion_req;
    planning_interface::MotionPlanResponse motion_res;

    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    moveit_msgs::Constraints pose_goal;

    geometry_msgs::PointStamped goal_point;
    goal_point.point = request.pose.pose.position;
    goal_point.header = request.pose.header;
    if(request.arm_type == ArmPlanningRequest::LEFT_ARM)
    {
      motion_req.group_name = "left_arm";
      //pose_goal = kinematic_constraints::constructGoalConstraints("7dof_RISE_wrist_link", goal_point);
      pose_goal =
          kinematic_constraints::constructGoalConstraints("7dof_RISE_wrist_link", request.pose, tolerance_pose, tolerance_angle);
    }
    else if (request.arm_type == ArmPlanningRequest::RIGHT_ARM)
    {
      motion_req.group_name = "right_arm";
      pose_goal =
          kinematic_constraints::constructGoalConstraints("6dof_RISE_wrist_link", request.pose, tolerance_pose, tolerance_angle);
    }

    motion_req.goal_constraints.push_back(pose_goal);
    planning_interface::PlanningContextPtr context =
        planner_instance_->getPlanningContext(planning_scene_, motion_req, motion_res.error_code_);
    context->solve(motion_res);

    if (motion_res.error_code_.val != motion_res.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully");
      return true;
    }

    int n = motion_res.trajectory_->getWayPointCount();



    trajectory_msgs::JointTrajectoryPoint point;
    if(request.arm_type == ArmPlanningRequest::LEFT_ARM)
    {
      /*point.positions.resize(7);
      for (int i=0; i<n; i++)
      {
        const double *pos = motion_res.trajectory_->getWayPoint(i).getJointPositions("7dof_RISE_joint_1");
        for (int j=0; j<7; j++)
        {
          point.positions[j] = pos[j];
        }
        response.trajectory.points.push_back(point);
      }
      for (int i=0; i<n; i++)
      {
        response.trajectory.joint_names.push_back(std::string("7dof_RISE_joint_"));
        ROS_INFO_STREAM(i<<":- "<<response.trajectory.joint_names.back());
      }*/
      moveit_msgs::MotionPlanResponse res;
      motion_res.getMessage(res);

      for(int i=0; i<res.trajectory.joint_trajectory.joint_names.size(); i++)
      {
        response.trajectory.joint_names.push_back(res.trajectory.joint_trajectory.joint_names[i]);
      }

      for(int j=0; j<res.trajectory.joint_trajectory.points.size(); j++)
      {
        trajectory_msgs::JointTrajectoryPoint pt;
        pt.positions = res.trajectory.joint_trajectory.points[j].positions;
        //ROS_INFO_STREAM(i<<"size of points: "<<res.trajectory.joint_trajectory.points.size());
        response.trajectory.points.push_back(pt);
      }
    }
    else if (request.arm_type == ArmPlanningRequest::RIGHT_ARM)
    {
     /* point.positions.resize(6);
      for (int i=0; i<n; i++)
      {
        const double *pos = motion_res.trajectory_->getWayPoint(i).getJointPositions("6dof_RISE_joint_1");
        for (int j=0; j<6; j++)
        {
          point.positions[j] = pos[j];
        }
        response.trajectory.points.push_back(point);
      }
      for (int i=0; i<n; i++)
      {
        response.trajectory.joint_names.push_back(std::string("6dof_RISE_joint_"+i+1));
      }*/
      moveit_msgs::MotionPlanResponse res;
      motion_res.getMessage(res);

      for(int i=0; i<res.trajectory.joint_trajectory.joint_names.size(); i++)
      {
        response.trajectory.joint_names.push_back(res.trajectory.joint_trajectory.joint_names[i]);
      }

      for(int j=0; j<res.trajectory.joint_trajectory.points.size(); j++)
      {
        trajectory_msgs::JointTrajectoryPoint pt;
        pt.positions = res.trajectory.joint_trajectory.points[j].positions;
        //ROS_INFO_STREAM(i<<"size of points: "<<res.trajectory.joint_trajectory.points.size());
        response.trajectory.points.push_back(pt);
      }
    }

    // Call the planner and visualize the trajectory
    moveit_msgs::MotionPlanResponse motion_res_vis;
    moveit_msgs::DisplayTrajectory display_trajectory;

    motion_res.getMessage(motion_res_vis);
    display_trajectory.trajectory_start = motion_res_vis.trajectory_start;
    display_trajectory.trajectory.push_back(motion_res_vis.trajectory);

    display_publisher_.publish(display_trajectory);

    return true;
  }



  ros::NodeHandle nh_;

  ros::ServiceServer feasibility_server_;
  ros::ServiceServer planner_server_;

  robot_model_loader::RobotModelLoader robot_model_loader_;
  robot_model::RobotModelPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;

  std::shared_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader_;
  planning_interface::PlannerManagerPtr planner_instance_;

  ros::Publisher display_publisher_;

};




}

using namespace socialrobot_motion;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "social_motion_planner");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  SocialMotionPlanner smp;

  ros::Rate r(10);
  while(ros::ok())
  {r.sleep();}

  return 0;
}
