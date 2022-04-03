#include "interface_controller.h"

static Eigen::Vector3d QuinticSpline(
                   double time,       ///< Current time
                   double time_0,     ///< Start time
                   double time_f,     ///< End time
                   double x_0,        ///< Start state
                   double x_dot_0,    ///< Start state dot
                   double x_ddot_0,   ///< Start state ddot
                   double x_f,        ///< End state
                   double x_dot_f,    ///< End state
                   double x_ddot_f )  ///< End state ddot
{
  double a1,a2,a3,a4,a5,a6;
  double time_s;

  Eigen::Vector3d result;

  if(time < time_0)
  {
    result << x_0, x_dot_0, x_ddot_0;
    return result;
  }
  else if (time > time_f)
  {
    result << x_f, x_dot_f, x_ddot_f;
    return result;
  }


  time_s = time_f - time_0;
  a1=x_0;
  a2=x_dot_0;
  a3=x_ddot_0/2.0;

  Eigen::Matrix3d Temp;
  Temp<<pow(time_s, 3), pow(time_s, 4), pow(time_s, 5),
        3.0 * pow(time_s, 2), 4.0 * pow(time_s, 3), 5.0 * pow(time_s, 4),
        6.0 * time_s, 12.0 * pow(time_s, 2), 20.0 * pow(time_s, 3);

  Eigen::Vector3d R_temp;
  R_temp<<x_f-x_0-x_dot_0*time_s-x_ddot_0*pow(time_s,2)/2.0,
        x_dot_f-x_dot_0-x_ddot_0*time_s,
        x_ddot_f-x_ddot_0;

  Eigen::Vector3d RES;

  RES = Temp.inverse()*R_temp;

  a4=RES(0);
  a5=RES(1);
  a6=RES(2);

  double time_fs = time - time_0;

  double position = a1+a2*pow(time_fs,1)+a3*pow(time_fs,2)+a4*pow(time_fs,3)+a5*pow(time_fs,4)+a6*pow(time_fs,5);
  double velocity = a2+2.0*a3*pow(time_fs,1)+3.0*a4*pow(time_fs,2)+4.0*a5*pow(time_fs,3)+5.0*a6*pow(time_fs,4);
  double acceleration =2.0*a3+6.0*a4*pow(time_fs,1)+12.0*a5*pow(time_fs,2)+20.0*a6*pow(time_fs,3);


  result<<position,velocity,acceleration;

  return result;
}



ArmController::ArmController(std::string name, ros::NodeHandle& nh) :
  nh_(nh),
  action_name_(name),
  as_(nh_, action_name_, false)
{
  as_.registerGoalCallback(boost::bind(&ArmController::ArmGoalCB, this));
  as_.registerPreemptCallback(boost::bind(&ArmController::ArmPreemptCB, this));

  joint_command_pub=nh_.advertise<sensor_msgs::JointState>("/vrep_interface/set_joint",1);
  joint_command_msg.header.frame_id="";

  as_.start();
}


void ArmController::ArmGoalCB()
{
  feedback_header_stamp_ = 0;
  goal_ = as_.acceptNewGoal();

  ROS_INFO("Arm Goal Received ");

  goal_start_time = ros::Time::now();

  traj_time = goal_->trajectory.points.back().time_from_start;
  as_joint_size = goal_->trajectory.points[0].positions.size();

  joint_command_msg.name.resize(as_joint_size);
  joint_command_msg.position.resize(as_joint_size);

  for(int i=0;i<as_joint_size;i++){
    joint_command_msg.name[i] = goal_->trajectory.joint_names[i];
  }

  feedback_.joint_names.resize(as_joint_size);
  feedback_.actual.positions.resize(as_joint_size);
  feedback_.actual.velocities.resize(as_joint_size);
  feedback_.actual.accelerations.resize(as_joint_size);
}

void ArmController::ArmPreemptCB()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
    as_.setPreempted();
}


void ArmController::compute()
{
  if (!as_.isActive())
      return;
  ros::Duration passedTime = ros::Time::now() - goal_start_time;
  if((ros::Time::now() >=goal_start_time)&&(passedTime<traj_time))
  { //If, current time is between start time of trajectory and end time of trajectory
    joint_command_msg.header.stamp=ros::Time::now();
    int point_size = static_cast<int>(goal_->trajectory.points.size());

    Eigen::Vector3d position_now;
    feedback_.joint_names=goal_->trajectory.joint_names;
    for(int j=0;j<as_joint_size;j++){ // j = joint number

      for(int i=0;i<point_size-1;i++){
        if((passedTime>=goal_->trajectory.points[i].time_from_start)&&(passedTime<goal_->trajectory.points[i+1].time_from_start)){
          position_now=QuinticSpline(passedTime.toSec(),goal_->trajectory.points[i].time_from_start.toSec(),goal_->trajectory.points[i+1].time_from_start.toSec(),
              goal_->trajectory.points[i].positions[j],goal_->trajectory.points[i].velocities[j],goal_->trajectory.points[i].accelerations[j],
              goal_->trajectory.points[i+1].positions[j],goal_->trajectory.points[i+1].velocities[j],goal_->trajectory.points[i+1].accelerations[j]);
        }
      }
      feedback_.actual.positions[j] = position_now(0);
      feedback_.actual.velocities[j] = position_now(1);
      feedback_.actual.accelerations[j] = position_now(2);

      joint_command_msg.position[j]=position_now(0);
      joint_command_pub.publish(joint_command_msg);
    }

    feedback_.actual.time_from_start=ros::Time::now()-goal_start_time;
    feedback_.header.seq=feedback_header_stamp_;

    feedback_header_stamp_++;
    as_.publishFeedback(feedback_);
  }

  if(ros::Time::now().toSec() > (goal_start_time.toSec() +  traj_time.toSec() + 0.5 ))
  {
    as_.setSucceeded();
  }
}

GripperController::GripperController(std::string name_, ros::NodeHandle &nh):
  nh_(nh),
  action_name_(name_),
  as_(nh_, action_name_, false)
{
  as_.registerGoalCallback(boost::bind(&GripperController::gGoalCB, this));
  as_.registerPreemptCallback(boost::bind(&GripperController::gPreemptCB, this));
  joint_command_pub=nh_.advertise<sensor_msgs::JointState>("/vrep_interface/set_joint",100);

  as_.start();
}

void GripperController::gGoalCB(){
  goal_ = as_.acceptNewGoal();
  ROS_INFO("Gripper Goal Received");

  goal_start_time = ros::Time::now();

  traj_time = goal_->trajectory.points.back().time_from_start;
  as_joint_size = goal_->trajectory.points[0].positions.size();


  joint_command_msg.header.stamp = goal_start_time;

  joint_command_msg.name.resize(as_joint_size);
  joint_command_msg.position.resize(as_joint_size);

  for(int i=0;i<as_joint_size;i++){
    joint_command_msg.name[i] = goal_->trajectory.joint_names[i];
    joint_command_msg.position[i] = goal_->trajectory.points.back().positions[i];
  }

  joint_command_pub.publish(joint_command_msg);

}

void GripperController::compute(){
  if (!as_.isActive())
      return;
  if(ros::Time::now().toSec()>(goal_start_time.toSec()+1.0)){
    as_.setSucceeded();
  }

}

void GripperController::gPreemptCB(){
  ROS_INFO("[%s] Preempted", action_name_.c_str());
    as_.setPreempted();
}


