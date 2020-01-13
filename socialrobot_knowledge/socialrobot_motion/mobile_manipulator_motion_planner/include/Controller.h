#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#define Hz 100.0

#include <iostream>
#include <Eigen/Dense>
#include <memory>
#include "RRT_planner2.h"
#include "NHRRT_planner.h"

#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <ros/ros.h>
#include <ros/node_handle.h>

using namespace RigidBodyDynamics;
using namespace std;
using namespace Eigen;


class Controller
{

public: 
	Controller(ros::NodeHandle nh_);
	~Controller();
	enum CONTROL_MODE
	{
		INIT,
		MOVE_MOBILE1,
		MOVE_MOBILE2,
		REACH_CUP,
		REACH_MILK,
		MOVE_CUP,
		MOVE_MILK,
		GRASP,
		RELEASE,
		DEFAULT
	};
public:
	void compute();
	void initModel();
	void initVariable();
	bool MobilePIDControl(double duration);
	bool JointPIDControl(double duration, bool left);
	void setMode(CONTROL_MODE mode);
	void readdata(VectorXd &ql, VectorXd &qr, VectorXd &qldot, VectorXd &qrdot, Vector3d base_pos);
	void writedata(VectorXd& ql, VectorXd& qr, VectorXd& base);
	void initPosition(VectorXd &ql, VectorXd &qr);

	void getUpdateKinematics(const VectorXd & q_l, const VectorXd & qdot_l, const VectorXd & q_r, const VectorXd & qdot_r);
// Math library

// variable Setting

	// Manipulator joint states
	struct jointState {
		VectorXd qInit_;
		VectorXd qGoal_;
		VectorXd q_;
		VectorXd qdot_;
	};
	struct JointLimit {
		VectorXd lower;
		VectorXd upper;
		VectorXd lower_rad;
		VectorXd upper_rad;
	};

	// Mobile base joint states
	struct baseState {
		VectorXd qInit_;
		VectorXd qGoal_;
		VectorXd q_;
		VectorXd qdot_;
		Matrix3d rotInit_;

		VectorXd target_q_;
		VectorXd cubic_q_;
		VectorXd q_pre_;
	};
	struct BaseLimit {
		VectorXd lower;
		VectorXd upper;
	};


	struct taskState {
		Vector3d xInit_;
		Vector3d x_;
		Vector3d x_des_;
		Vector3d x_cubic_;
		Vector3d x_prev_;
		VectorXd x_dot;
		VectorXd x_dotd;
		VectorXd x_error;

		Matrix3d rot_;
		Matrix3d rotInit_;
		Vector3d phi_;

		VectorXd xdot_; // 6d
		VectorXd x_error_; // 6d
		VectorXd torque_;
		MatrixXd J_;
		MatrixXd J_inv_;
		MatrixXd JT_;
		MatrixXd lambda_;
		MatrixXd lambda_inv_;

		MatrixXd A_;
		MatrixXd A_inv_;
		MatrixXd Lambda_;
	};

	struct jointTarget {
		VectorXd q_;
		VectorXd cubic_q_;
		VectorXd desired_q_;
	};


	Matrix3d Rot_l;
	Matrix3d Rot_r;
	Matrix6d Rot_l_tot;
	Matrix6d Rot_r_tot;



	//// RBDL ////
	double mass_[dof];
	Math::Vector3d inertia_[dof];

	shared_ptr<Model> model_l, model_r;
	unsigned int body_id_l[dof], body_id_r[dof];
	unsigned int base_id_[2]; // virtual joint
	unsigned int virtual_body_id_[4];

	Body body_l[dof], body_r[dof];
	Body base_body;
	Body virtual_body_[6];
	Joint joint_l[dof], joint_r[dof];
	Joint virtual_joint_[6];


	Vector3d axis_l[dof];
	Vector3d axis_r[dof];

	Math::Vector3d joint_position_global_l[dof];
	Math::Vector3d joint_position_local_l[dof];
	Math::Vector3d com_position_l[dof+1];

	Math::Vector3d joint_position_global_r[dof];
	Math::Vector3d joint_position_local_r[dof];
	Math::Vector3d com_position_r[dof];

	VectorXd q_l, qdot_l, q_r, qdot_r;


	jointState joint_left_, joint_right_;
	taskState task_left_, task_right_;
	jointTarget joint_target_left_, joint_target_right_;
	JointLimit joint_limit_left_, joint_limit_right_;
	baseState base_;
	BaseLimit base_limit_;

	double playTime_;
	double Hz_;
	double controlStartTime_;


	CONTROL_MODE controlMode_;
	bool isModeChanged;

	rrt _rrt;
	NH_RRT _nh_rrt;

	Robotmodel _robot_left, _robot_right;
	int target_num, target_state;
	MatrixXd _joint_target, _joint_target2;
	MatrixXd target_1;

	double base_nhrrt_vel;
	double base_nhrrt_b;
	double base_nhrrt_thereshold;

	Vector3d base_pos_;
	Vector4d base_desired_vel_;

	double dist_base_to_point;
	double nhrrt_buf;
	double nhrrt_r;
	double nhrrt_v_r;
	double nhrrt_v_l;

	double mobile_b;
	double mobile_c;
	double mobile_d;
	MatrixXd mobile_S;
	MatrixXd mobile_v;
	MatrixXd mobile_p_err;
	MatrixXd mobile_v_err;

	double temp;
	VectorXd target_to_vrep_;
	int base_control_mode;
	bool mobile_flag;

	void NHRRT_controller();
	void RRT_planning(VectorXd q_start, VectorXd q_goal, MatrixXd& joint_target, bool left);
	void CRRT_planning(VectorXd q_start, VectorXd q_goal, MatrixXd& joint_target, bool left);
	void NHRRT_planning(VectorXd q_start, VectorXd q_goal, MatrixXd& joint_target);

	Matrix3d rot_diff ;
	Vector3d rot_diff_vec;
	VectorXd target_;
	void IK_solver(Vector3d x_pos, Vector3d rot_angle, bool left);
	void TRAC_IK_solver(Vector6d pose, bool left, Robotmodel &model);
	TRAC_IK::TRAC_IK ik_solver(KDL::Chain chain, KDL::JntArray lower_joint_limits, KDL::JntArray upper_joint_limits, double timeout_in_secs=0.005, double error=1e-5, TRAC_IK::SolveType type=TRAC_IK::Speed);  
	std::string urdf_param;
	
	double fRand(double min, double max)
	{
	double f = (double)rand() / RAND_MAX;
	return min + f * (max - min);
	};

	static const Matrix3d Vec_2_Rot(const Vector3d& vec) {
		Matrix3d Rot;
		double angle = vec.norm();

		if (angle == 0.0)
			Rot = Matrix3d::Identity();
		else {
			Vector3d axis = vec / angle;
			Matrix3d V;
			V.setZero();
			V(0, 1) = -axis(2);
			V(0, 2) = axis(1);
			V(1, 0) = axis(2);
			V(1, 2) = -axis(0);
			V(2, 0) = -axis(1);
			V(2, 1) = axis(0);

			Rot = Matrix3d::Identity() + V * sin(angle) + (1 - cos(angle))* V* V;
		}
		return Rot;
	};
};
#endif