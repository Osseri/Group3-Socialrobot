#include "ROS_bridge.hpp"

ROS_bridge::ROS_bridge(ros::NodeHandle nh_,double hz_):
    rate_(hz_)
  {    
    is_first_run = true;
    tick = 0;
    sim_step_done_ = false;
    sim_time_ = 0.0f;

    current_ql_.resize(dof);
    current_ql_dot_.resize(dof);
    desired_ql_.resize(dof);
    desired_ql_.setZero();
    target_x_.resize(3);

    current_qr_.resize(dof);
    current_qr_dot_.resize(dof);
    desired_qr_.resize(dof);
    desired_qr_.setZero();
    target_xr_.resize(3);

    desired_base_vel_.resize(4);
    desired_base_vel_.setZero();

    // current_base_.setZero();

    ljoint_cmd_.name.resize(dof);
    ljoint_cmd_.position.resize(dof);

    lgripper_cmd_.name.resize(2);
    lgripper_cmd_.position.resize(2);

    rjoint_cmd_.name.resize(dof);
    rjoint_cmd_.position.resize(dof);

    rgripper_cmd_.name.resize(2);
    rgripper_cmd_.position.resize(2);

    base_cmd_.name.resize(4);
    base_cmd_.velocity.resize(4);

	// velocity_l = 3.0;
	// velocity_r = 3.0;
	desired_grasping_l = 0.1;
	desired_grasping_r = 0.1;

    for(size_t i=0; i<dof; i++)
    {
      ljoint_cmd_.name[i]= L_JOINT_NAME[i];
	    rjoint_cmd_.name[i]= R_JOINT_NAME[i];
    }

        for(size_t i=0; i<2; i++)
    {
      lgripper_cmd_.name[i]= L_GRIPPER_NAME[i];
	    rgripper_cmd_.name[i]= R_GRIPPER_NAME[i];
    }
        for(size_t i=0; i<4; i++)
    {
	    base_cmd_.name[i]= BASE_JOINT_NAME[i];

    }

    vrep_sim_start_pub_ = nh_.advertise<std_msgs::Bool>("/startSimulation", 5);
    vrep_sim_stop_pub_ = nh_.advertise<std_msgs::Bool>("/stopSimulation", 5);
    vrep_sim_step_trigger_pub_ = nh_.advertise<std_msgs::Bool>("/triggerNextStep", 100);
    vrep_sim_enable_syncmode_pub_ = nh_.advertise<std_msgs::Bool>("/enableSyncMode", 5);

    // Queue_size = 발행하는 메세지를 몇 개까지 저장해 둘 것인지
    vrep_ljoint_set_pub_ = nh_.advertise<sensor_msgs::JointState>("/panda/left_joint_set", 1);
    vrep_lgripper_set_pub_ = nh_.advertise<sensor_msgs::JointState>("/panda/left_gripper_joint_set", 1);

    vrep_rjoint_set_pub_ = nh_.advertise<sensor_msgs::JointState>("/panda/right_joint_set", 1);
    vrep_rgripper_set_pub_ = nh_.advertise<sensor_msgs::JointState>("/panda/right_gripper_joint_set", 1);

    vrep_base_set_pub_ = nh_.advertise<sensor_msgs::JointState>("/husky/base_joint_set", 1);
    
    vrep_ljoint_state_sub_ = nh_.subscribe("/panda/left_joint_states", 100, &ROS_bridge::ljoint_cb, this);
    vrep_rjoint_state_sub_ = nh_.subscribe("/panda/right_joint_states", 100, &ROS_bridge::rjoint_cb, this);
    vrep_base_com_state_pub_ = nh_.subscribe("/husky/base_com_states", 100, &ROS_bridge::base_com_cb, this);

    vrep_sim_step_done_sub_ = nh_.subscribe("/simulationStepDone", 100, &ROS_bridge::sim_step_done_cb, this);
    vrep_sim_time_sub_ = nh_.subscribe("/simulationTime",100,&ROS_bridge::sim_time_cb,this);
    vrep_sim_status_sub_ = nh_.subscribe("/simulationState",100,&ROS_bridge::sim_status_cb,this);

  }
  ROS_bridge::~ROS_bridge()
  {       
  }

  void ROS_bridge::set_exec_time(float t)
  {
    exec_time_ = t;
  }
  
  void ROS_bridge::ljoint_cb(const sensor_msgs::JointStateConstPtr& msg)
  {
	  for(size_t i=0; i< msg->name.size(); i++)
      {
        current_ql_[i] = msg->position[i];
        current_ql_dot_[i] = msg->velocity[i];        
      }
}

  void ROS_bridge::rjoint_cb(const sensor_msgs::JointStateConstPtr& msg)
  {
	  for(size_t i=0; i< msg->name.size(); i++)
    {
      current_qr_[i] = msg->position[i];
      current_qr_dot_[i] = msg->velocity[i];        
    }
  }

  void ROS_bridge::base_com_cb(const geometry_msgs::Pose2DConstPtr &msg)
  {
    current_base_(0) = msg->x;
    current_base_(1) = msg->y;
    current_base_(2) = msg->theta;

  }

  void ROS_bridge::sim_status_cb(const std_msgs::Int32ConstPtr& msg)
  {
    vrep_sim_status = msg->data;
  }

  void ROS_bridge::read_vrep()
  {
    ros::spinOnce();
  }
  

  void ROS_bridge::write_vrep()
  {
    
    for(size_t i=0;i<dof;i++) {
        ljoint_cmd_.position[i] = desired_ql_(i);
        rjoint_cmd_.position[i] = desired_qr_(i);
    }

   for(size_t i=0;i<2;i++) {
        lgripper_cmd_.position[i] = desired_grasping_l;
        rgripper_cmd_.position[i] = desired_grasping_r;
    }

    for (size_t i = 0; i<4;i++){
      base_cmd_.velocity[i] = desired_base_vel_(i);
    }
     
    vrep_ljoint_set_pub_.publish(ljoint_cmd_);
    vrep_lgripper_set_pub_.publish(lgripper_cmd_);

	  vrep_rjoint_set_pub_.publish(rjoint_cmd_);
	  vrep_rgripper_set_pub_.publish(rgripper_cmd_);

    vrep_base_set_pub_.publish(base_cmd_);
    
    vrepStepTrigger();
  }
  void ROS_bridge::wait()
  {
    while(ros::ok() && !sim_step_done_)
    {
      ros::spinOnce();
    }
    sim_step_done_ = false;
    rate_.sleep();
  }

  void ROS_bridge::sim_time_cb(const std_msgs::Float32ConstPtr& msg)
  {
    sim_time_ = msg->data;
    tick = (sim_time_*100)/(SIM_DT*100);    
  }

  void ROS_bridge::sim_step_done_cb(const std_msgs::BoolConstPtr &msg)
  {
    sim_step_done_ = msg->data;
  }

  void ROS_bridge::vrepStart()
  {
    ROS_INFO("Starting V-REP Simulation");
    std_msgs::Bool msg;
    msg.data = true;
    std_msgs::Bool msg2;
    msg2.data=false;
    vrep_sim_start_pub_.publish(msg);
    vrep_sim_enable_syncmode_pub_.publish(msg2);
  }
  void ROS_bridge::vrepStop()
  {
    ROS_INFO("Stopping V-REP Simulation");
    std_msgs::Bool msg;
    msg.data = true;
    vrep_sim_stop_pub_.publish(msg);
  }
  void ROS_bridge::vrepStepTrigger()
  {
    std_msgs::Bool msg;
    msg.data = true;
    vrep_sim_step_trigger_pub_.publish(msg);
  }
  void ROS_bridge::vrepEnableSyncMode()
  {
    ROS_INFO("Sync Mode On");
    std_msgs::Bool msg;
    msg.data = true;
    vrep_sim_enable_syncmode_pub_.publish(msg);
  }

