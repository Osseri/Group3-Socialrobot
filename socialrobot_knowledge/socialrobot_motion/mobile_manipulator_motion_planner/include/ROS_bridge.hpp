
#ifndef SIMULATION_INTERFACE_H
#define SIMULATION_INTERFACE_H

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "Controller.h"

#define TOTAL_DOF 9 // 9 dof robot(7 + 2 gripper)
#define SIM_DT    0.01 // 10ms
#define PI 3.14159265359
#define deg2rad(deg)  ((deg) * PI / 180.0)
#define rad2deg(rad)  ((rad) * 180.0 / PI)

const std::string R_JOINT_NAME[dof] = {"panda_joint1_r","panda_joint2_r","panda_joint3_r","panda_joint4_r","panda_joint5_r","panda_joint6_r","panda_joint7_r"};
const std::string R_GRIPPER_NAME[2] = {"panda_finger_joint1_r","panda_finger_joint2_r"};

const std::string L_JOINT_NAME[dof] = {"panda_joint1_l","panda_joint2_l","panda_joint3_l","panda_joint4_l","panda_joint5_l","panda_joint6_l","panda_joint7_l"};
const std::string L_GRIPPER_NAME[2] = {"panda_finger_joint1_l","panda_finger_joint2_l"};

const std::string BASE_JOINT_NAME[4] = {"joint_1", "joint_2", "joint_3", "joint_4"};


    class ROS_bridge
    {
    public:
      ROS_bridge(ros::NodeHandle nh_,double hz_);
      ~ROS_bridge();
    
    void ljoint_cb(const sensor_msgs::JointStateConstPtr& msg);
    void rjoint_cb(const sensor_msgs::JointStateConstPtr& msg);
    
    void base_com_cb(const geometry_msgs::Pose2DConstPtr& msg);

    void sim_status_cb(const std_msgs::Int32ConstPtr& msg);
    void sim_time_cb(const std_msgs::Float32ConstPtr& msg);
    void vrepStart();
    void vrepStop();
    void vrepStepTrigger();
    void vrepEnableSyncMode();
    void sim_step_done_cb(const std_msgs::BoolConstPtr& msg);

    void set_desired_q(std::vector<float> dq);
    void set_exec_time(float t);
    void read_vrep();
    void write_vrep();
    void wait();

    Eigen::VectorXd current_ql_, current_qr_;
	Eigen::VectorXd current_ql_dot_, current_qr_dot_;
	Eigen::VectorXd desired_ql_, desired_qr_;
	Eigen::VectorXd target_x_, target_xr_;
	double desired_grasping_l, desired_grasping_r;

	Eigen::Vector3d euler_;
	Eigen::VectorXd desired_base_vel_;
    Eigen::Vector3d current_base_;

    private:
    ros::Publisher vrep_ljoint_set_pub_;
    ros::Publisher vrep_lgripper_set_pub_;

	ros::Publisher vrep_rjoint_set_pub_;
	ros::Publisher vrep_rgripper_set_pub_;

    ros::Publisher vrep_base_set_pub_;
    ros::Publisher vrep_sim_start_pub_;
    ros::Publisher vrep_sim_stop_pub_;
    ros::Publisher vrep_sim_step_trigger_pub_;
    ros::Publisher vrep_sim_enable_syncmode_pub_;
    ros::Publisher vrep_gripper_set_pub;
    /**
    * @brief controller to vrep simulator subscriber message lists
    * @param vrep_joint_state_sub_(sensor_msgs::JointState) : get vrep simulator robot current joint value
    * @param vrep_sim_step_done_sub_(std_msgs::Bool) : get simulation trigger step done signal
    * @param vrep_sim_time_sub_(std_msgs::Float32) : get vrep simulator time tick
    * @param vrep_sim_status_sub_(std_msgs::Int32) : get vrep simulation status(0:sim not running, 1: sim paused, 2:sim stopped, 3:sim running)
    */
    ros::Subscriber vrep_ljoint_state_sub_;
	ros::Subscriber vrep_rjoint_state_sub_;
	ros::Subscriber vrep_base_joint_state_pub_;
    ros::Subscriber vrep_base_com_state_pub_;

    ros::Subscriber vrep_sim_step_done_sub_;
    ros::Subscriber vrep_sim_time_sub_;
    ros::Subscriber vrep_sim_status_sub_;

    /**
    * @brief Other parameters for using ros interface and control    
    */

    bool sim_step_done_;
    float sim_time_; // from v-rep simulation time
    int tick;
    ros::Rate rate_;
    sensor_msgs::JointState ljoint_cmd_;
    sensor_msgs::JointState lgripper_cmd_;

    sensor_msgs::JointState rjoint_cmd_;
    sensor_msgs::JointState rgripper_cmd_;

    sensor_msgs::JointState base_cmd_;
    sensor_msgs::JointState gripper_cmd_;
    
    bool is_first_run;
    double start_time;
    double current_time;
    double final_time;    
    int vrep_sim_status;
    float exec_time_;
    };




#endif
