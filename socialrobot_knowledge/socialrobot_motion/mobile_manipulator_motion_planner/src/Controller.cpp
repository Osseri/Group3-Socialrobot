#include "Controller.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>

#define DEGREE M_PI/180.0

using std::ofstream;

Controller::Controller(ros::NodeHandle nh_)
{

}
Controller::~Controller()
{       
  }
void Controller::initModel() {
	model_l = make_shared<Model>();
	model_r = make_shared<Model>();

	// virtual_body_[0] = Body(0.0, Math::Vector3d(0.0, 0.0, 0.0), Math::Matrix3d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
	// virtual_joint_[0] = Joint(JointTypePrismatic, Eigen::Vector3d::UnitX());
	// virtual_body_[1] = Body(0.0, Math::Vector3d(0.0, 0.0, 0.0), Math::Matrix3d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
	// virtual_joint_[1] = Joint(JointTypePrismatic, Eigen::Vector3d::UnitY());
	// virtual_body_[2] = Body(0.0, Math::Vector3d(0.0, 0.0, 0.0), Math::Matrix3d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
	// virtual_joint_[2] = Joint(Math::SpatialVector(0.0, 0.0, 1.0, 0.0, 0.0, 0.0));

	// virtual_body_id_[0] = model_l->AddBody(0, Math::Xtrans(Math::Vector3d(0.0, 0.0, 0.0)), virtual_joint_[0], virtual_body_[0]);
	// virtual_body_id_[1] = model_l->AddBody(virtual_body_id_[0], Math::Xtrans(Math::Vector3d(0.0, 0.0, 0.0)), virtual_joint_[1], virtual_body_[1]);

	// virtual_body_id_[2] = model_r->AddBody(0, Math::Xtrans(Math::Vector3d(0.0, 0.0, 0.0)), virtual_joint_[0], virtual_body_[0]);
	// virtual_body_id_[3] = model_r->AddBody(virtual_body_id_[0], Math::Xtrans(Math::Vector3d(0.0, 0.0, 0.0)), virtual_joint_[1], virtual_body_[1]);

	// double mass = 30.0;
	// base_body = Body(mass, Math::Vector3d(0.0, 0.0, 0.0), Math::Vector3d(0.5, 100.0, 100.0));
	// base_id_[0] = model_l->AddBody(virtual_body_id_[1], Math::Xtrans(Math::Vector3d(0.0, 0.0, 0.0)), virtual_joint_[2], base_body); //body frame = joint frame
	// base_id_[1] = model_r->AddBody(virtual_body_id_[3], Math::Xtrans(Math::Vector3d(0.0, 0.0, 0.0)), virtual_joint_[2], base_body); //body frame = joint frame


	model_l->gravity = Vector3d(0., 0, -9.81);
	model_r->gravity = Vector3d(0., 0, -9.81);

	Rot_l(0, 0) = 0.4698;
	Rot_l(0, 1) = 0.8660;
	Rot_l(0, 2) = 0.1710;
	Rot_l(1, 0) = -0.8138;
	Rot_l(1, 1) = 0.5;
	Rot_l(1, 2) = -0.2962;
	Rot_l(2, 0) = -0.3420;
	Rot_l(2, 1) = 0.0;
	Rot_l(2, 2) = 0.9397;

	Rot_r(0, 0) = 0.4698;
	Rot_r(0, 1) = -0.8660;
	Rot_r(0, 2) = 0.1710;
	Rot_r(1, 0) = 0.8138;
	Rot_r(1, 1) = 0.5;
	Rot_r(1, 2) = 0.2962;
	Rot_r(2, 0) = -0.3420;
	Rot_r(2, 1) = 0.0;
	Rot_r(2, 2) = 0.9397;

	for (int i = 0;i < 2;i++) {
		Rot_l_tot.block(3 * i, 3 * i, 3, 3) = Rot_l;
		Rot_r_tot.block(3 * i, 3 * i, 3, 3) = Rot_r;
	}

	axis_l[0] = Rot_l * Eigen::Vector3d::UnitZ();
	axis_l[1] = Rot_l * Eigen::Vector3d::UnitY();
	axis_l[2] = Rot_l * Eigen::Vector3d::UnitZ();
	axis_l[3] = Rot_l * (-1.0*Eigen::Vector3d::UnitY());
	axis_l[4] = Rot_l * Eigen::Vector3d::UnitZ();
	axis_l[5] = Rot_l * (-1.0*Eigen::Vector3d::UnitY());
	axis_l[6] = Rot_l * (-1.0*Eigen::Vector3d::UnitZ());


	axis_r[0] = Rot_r * Eigen::Vector3d::UnitZ();
	axis_r[1] = Rot_r * Eigen::Vector3d::UnitY();
	axis_r[2] = Rot_r * Eigen::Vector3d::UnitZ();
	axis_r[3] = Rot_r * (-1.0*Eigen::Vector3d::UnitY());
	axis_r[4] = Rot_r * Eigen::Vector3d::UnitZ();
	axis_r[5] = Rot_r * (-1.0*Eigen::Vector3d::UnitY());
	axis_r[6] = Rot_r * (-1.0*Eigen::Vector3d::UnitZ());


	for (int i = 0; i < dof; i++) {
		mass_[i] = 1.0;
		inertia_[i] = Vector3d(1.0, 1.0, 1.0);
	}


	joint_position_global_l[0] = Vector3d(0.2663, -0.2688, 0.9639);
	joint_position_global_l[1] = Vector3d(0.2663, -0.2688, 0.9639);
	joint_position_global_l[2] = Vector3d(0.3203, -0.3624, 1.2609);
	joint_position_global_l[3] = Vector3d(0.3591, -0.4295, 1.2326);
	joint_position_global_l[4] = Vector3d(0.3860, -0.4762, 1.6217);
	joint_position_global_l[5] = Vector3d(0.3860, -0.4762, 1.6217);
	joint_position_global_l[6] = Vector3d(0.4273, -0.5478, 1.5916);


	joint_position_local_l[0] = joint_position_global_l[0];
	for (int i = 1; i < dof; i++)
		joint_position_local_l[i] = joint_position_global_l[i] - joint_position_global_l[i - 1];

	joint_position_global_r[0] = Vector3d(0.2663, 0.2690, 0.9639);
	joint_position_global_r[1] = Vector3d(0.2663, 0.2690, 0.9639);
	joint_position_global_r[2] = Vector3d(0.3203, 0.3626, 1.2609);
	joint_position_global_r[3] = Vector3d(0.3591, 0.4297, 1.2326);
	joint_position_global_r[4] = Vector3d(0.3860, 0.4763, 1.6217);
	joint_position_global_r[5] = Vector3d(0.3860, 0.4763, 1.6217);
	joint_position_global_r[6] = Vector3d(0.4273, 0.5479, 1.5916);

	joint_position_local_r[0] = joint_position_global_r[0];
	for (int i = 1; i < dof; i++)
		joint_position_local_r[i] = joint_position_global_r[i] - joint_position_global_r[i - 1];


	com_position_l[0] = Vector3d(0.2234, -0.2638, 0.8929);
	com_position_l[1] = Vector3d(0.3093, -0.2744, 1.0356);
	com_position_l[2] = Vector3d(0.3520, -0.3640, 1.2105);
	com_position_l[3] = Vector3d(0.3201, -0.4152, 1.2894);
	com_position_l[4] = Vector3d(0.4047, -0.4239, 1.5191);
	com_position_l[5] = Vector3d(0.3995, -0.5200, 1.6215);
//	com_position_l[6] = Vector3d(0.4090, -0.5400, 1.5129); // link7_respondable1
	com_position_l[6] = Vector3d(0.4062, -0.5084, 1.4721); // panda_hand_visual1

	com_position_l[7] = Vector3d(0.3902, -0.4866, 1.3821); // cup

	for (int i = 0; i < dof; i++)
		com_position_l[i] -= joint_position_global_l[i];

	com_position_r[0] = Vector3d(0.2834, 0.2294, 0.8929);
	com_position_r[1] = Vector3d(0.2496, 0.3090, 1.0356);
	com_position_r[2] = Vector3d(0.3059, 0.3908, 1.2105);
	com_position_r[3] = Vector3d(0.3662, 0.3888, 1.2894);
	com_position_r[4] = Vector3d(0.3314, 0.4664, 1.5191);
	com_position_r[5] = Vector3d(0.4172, 0.5099, 1.6216);
	com_position_r[6] = Vector3d(0.4038, 0.5100, 1.4721); // panda_hand_respondable0

	for (int i = 0; i < dof; i++)
		com_position_r[i] -= joint_position_global_r[i];



	for (int i = 0; i < dof; i++) {
		body_l[i] = Body(mass_[i], com_position_l[i], inertia_[i]);
		joint_l[i] = Joint(JointTypeRevolute, axis_l[i]);

		body_r[i] = Body(mass_[i], com_position_r[i], inertia_[i]);
		joint_r[i] = Joint(JointTypeRevolute, axis_r[i]);


		if (i == 0) {
			body_id_l[i] = model_l->AddBody(0, Math::Xtrans(joint_position_local_l[i]), joint_l[i], body_l[i]);
			body_id_r[i] = model_r->AddBody(0, Math::Xtrans(joint_position_local_r[i]), joint_r[i], body_r[i]);
		}
		else {
			body_id_l[i] = model_l->AddBody(body_id_l[i - 1], Math::Xtrans(joint_position_local_l[i]), joint_l[i], body_l[i]);
			body_id_r[i] = model_r->AddBody(body_id_r[i - 1], Math::Xtrans(joint_position_local_r[i]), joint_r[i], body_r[i]);

		}
	}

}
void Controller::initVariable() {
	base_limit_.upper.resize(3);
	base_limit_.lower.resize(3);
	base_limit_.upper(0) = 6.2;
	base_limit_.lower(0) = -1.5;

	base_limit_.upper(1) = 3.7;
	base_limit_.lower(1) = -3.2;

	base_limit_.upper(2) = M_PI;
	base_limit_.lower(2) = -M_PI;

	base_.qGoal_.resize(3);
	base_.qGoal_.setZero();
	base_.qInit_.resize(3);
	base_.qInit_.setZero();

	base_.target_q_.resize(2);
	base_.target_q_.setZero();

	base_.cubic_q_.resize(2);
	base_.cubic_q_.setZero();

	base_.q_pre_.resize(2);
	base_.q_pre_.setZero();

	mobile_S.resize(2, 2);
	mobile_S.setZero();

	mobile_v.resize(2, 1);
	mobile_v.setZero();

	mobile_p_err.resize(2, 1);
	mobile_p_err.setZero();

	mobile_v_err.resize(2, 1);
	mobile_v_err.setZero();

	target_to_vrep_.resize(3);
	target_to_vrep_.setZero();

	_nh_rrt.upper_limit_goal_bias.resize(3);
	_nh_rrt.upper_limit_goal_bias.setZero();
	_nh_rrt.lower_limit_goal_bias.resize(3);
	_nh_rrt.lower_limit_goal_bias.setZero();

	base_desired_vel_.setZero();

	joint_left_.qdot_.resize(dof);
	joint_left_.q_.resize(dof);
	joint_left_.qInit_.resize(dof);
	joint_left_.qGoal_.resize(dof);


	joint_left_.qdot_.setZero();
	joint_left_.q_.setZero();
	joint_left_.qInit_.setZero();
	joint_left_.qGoal_.setZero();

	joint_right_.qdot_.resize(dof);
	joint_right_.q_.resize(dof);
	joint_right_.qInit_.resize(dof);
	joint_right_.qGoal_.resize(dof);

	joint_right_.qdot_.setZero();
	joint_right_.q_.setZero();
	joint_right_.qInit_.setZero();
	joint_right_.qGoal_.setZero();

	joint_target_left_.q_.resize(dof);
	joint_target_left_.q_.setZero();
	joint_target_left_.cubic_q_.resize(dof);
	joint_target_left_.desired_q_.resize(dof);
	joint_target_left_.desired_q_.setZero();

	joint_target_right_.q_.resize(dof);
	joint_target_right_.q_.setZero();
	joint_target_right_.cubic_q_.resize(dof);
	joint_target_right_.desired_q_.resize(dof);
	joint_target_right_.desired_q_.setZero();

	joint_limit_right_.upper.resize(dof);
	joint_limit_right_.lower.resize(dof);
	joint_limit_left_.upper.resize(dof);
	joint_limit_left_.lower.resize(dof);


	joint_limit_right_.upper_rad.resize(dof);
	joint_limit_right_.lower_rad.resize(dof);
	joint_limit_left_.upper_rad.resize(dof);
	joint_limit_left_.lower_rad.resize(dof);

	for (int i = 0; i < dof; i++) {
		joint_limit_left_.lower(i) = -166.0;
		joint_limit_left_.upper(i) = 166.0;
	}
	for (int i = 0; i < dof; i++) {
		joint_limit_right_.lower(i) = -166.0;
		joint_limit_right_.upper(i) = 166.0;
	}

	joint_limit_left_.lower(1) = -101.0;
	joint_limit_left_.upper(1) = 101.0;

	joint_limit_left_.lower(3) = -176.0;
	joint_limit_left_.upper(3) = 4.0;

	joint_limit_left_.lower(5) = -1.0;
	joint_limit_left_.upper(5) = 215;


	joint_limit_right_.lower(1) = -101.0;
	joint_limit_right_.upper(1) = 101.0;

	joint_limit_right_.lower(3) = -176.0;
	joint_limit_right_.upper(3) = 4.0;

	joint_limit_right_.lower(5) = -1.0;
	joint_limit_right_.upper(5) = 215;
	
	///// deg -> rad
	joint_limit_left_.upper_rad = joint_limit_left_.upper / 180.0*M_PI;
	joint_limit_left_.lower_rad = joint_limit_left_.lower / 180.0*M_PI;

	joint_limit_right_.upper_rad = joint_limit_right_.upper / 180.0*M_PI;
	joint_limit_right_.lower_rad = joint_limit_right_.lower / 180.0*M_PI;
	
	target_.resize(6);
	target_.setZero();

	playTime_ = 0.0;

	q_l.resize(dof + 3);
	q_l.setZero();
	qdot_l.resize(dof+3);
	qdot_l.setZero();
	q_r.resize(dof +3);
	q_r.setZero();
	qdot_r.resize(dof+3);
	qdot_r.setZero();

}

void Controller::initPosition(VectorXd &ql, VectorXd &qr){
	
	joint_target_left_.desired_q_ = ql;
	joint_target_right_.desired_q_ = qr;

}
void Controller::setMode(CONTROL_MODE mode) {
	isModeChanged = true;
	controlMode_ = mode;
}
void Controller::compute() {
	_robot_left.model = model_l;
	_robot_left.q = joint_left_.q_;
	for (int i = 0;i < dof;i++) {
		_robot_left.body_id[i] = body_id_l[i];
		_robot_left.com_id[i] = com_position_l[i];
	}
	_robot_left.Rot = Rot_l;

	_robot_right.model = model_r;
	_robot_right.q = joint_right_.q_;
	for (int i = 0;i < dof;i++) {
		_robot_right.body_id[i] = body_id_r[i];
		_robot_right.com_id[i] = com_position_r[i];
	}
	_robot_right.Rot = Rot_r;

	bool joint_flag = false;
	//int target_num = 2;

	if (isModeChanged)
	{
		isModeChanged = false;
		controlStartTime_ = playTime_;

		joint_left_.qInit_ = joint_left_.q_;
		joint_right_.qInit_ = joint_right_.q_;

		base_.qInit_ = base_.q_;
		base_.rotInit_ = Rotate_with_Z(base_.qInit_(2));

		target_state = 1;

	
	

 	}
	switch (controlMode_)
	{
	case INIT:
		joint_target_right_.q_.setZero();
		joint_target_right_.q_(0) = -60.0 * DEGREE;
		joint_target_right_.q_(1) = -45.0 * DEGREE;
		joint_target_right_.q_(2) = 0.0 * DEGREE;
		joint_target_right_.q_(3) = -120.0 * DEGREE;
		joint_target_right_.q_(4) = 0.0 * DEGREE;
		joint_target_right_.q_(5) = 90.0 * DEGREE;
		joint_target_right_.q_(6) = 0.0 * DEGREE;


		JointPIDControl(0.1*Hz, false); // false : Right Arm Control, true : Left Arm Control
		joint_target_left_.q_(0) = 60.0 * DEGREE;
		joint_target_left_.q_(1) = -45.0 * DEGREE;
		joint_target_left_.q_(2) = 0.0 * DEGREE;
		joint_target_left_.q_(3) = -120.0 * DEGREE;
		joint_target_left_.q_(4) = 0.0 * DEGREE;
		joint_target_left_.q_(5) = 90.0 * DEGREE;
		joint_target_left_.q_(6) = 0.0 * DEGREE;
		JointPIDControl(0.1 * Hz, true);

		break;
	case MOVE_MOBILE1:
		if (controlStartTime_ == playTime_) {

			base_.qGoal_(0) = 4.9;
			base_.qGoal_(1) = -1.8;
			base_.qGoal_(2) = -M_PI/2.0;

			_nh_rrt.Obs[0].pos(0) = 2.7251;
			_nh_rrt.Obs[0].pos(1) = -0.5276;
			_nh_rrt.Obs[0].radius = 0.6;

			_nh_rrt.Obs[1].pos(0) = 4.5502;
			_nh_rrt.Obs[1].pos(1) = 2.0;
			_nh_rrt.Obs[1].radius = 0.6;

			_nh_rrt.Obs[2].pos(0) = 4.5;
			_nh_rrt.Obs[2].pos(1) = -2.825;
			_nh_rrt.Obs[2].radius = 0.1;

			_nh_rrt.obs_num = 3;
			_nh_rrt.base_length = 1.2; //1.14
			_nh_rrt.base_width = 0.7;

			// plan path
			//NHRRT_planning(base_.qInit_, base_.qGoal_, _joint_target);

			// saved data // 
			ifstream inFile2("/home/kendrick/catkin_ws/src/mobile_manipulator_motion_planner/path_result_ex1.txt"); // "reading"
			MatrixXd joint_temp(200, 3);

			int size = 0;
			std::vector<std::string> parameters;
			char inputString[1000];
			while (!inFile2.eof())
			{ // eof : end of file
				inFile2.getline(inputString, 1000);
				boost::split(parameters, inputString, boost::is_any_of(","));
				if (parameters.size() == 3)
				{
					for (int j = 0; j < parameters.size(); j++)
						joint_temp(size, j) = atof(parameters[j].c_str()); // c_str() : string -> char* // atof : char* -> float
					size++;
				}
			}
			inFile2.close();
			_joint_target = joint_temp.topLeftCorner(size, 3);
			///////////////////////////////////////////////////////////////////////////////

			target_state = 0;
			base_control_mode = 1;
			mobile_flag = true;
			target_num = _joint_target.rows();

			// _joint_target : Matrix of configurations (x, y, theta) 
		}
		joint_flag = false;

		NHRRT_controller();

		break;

	case MOVE_MOBILE2:
		if (controlStartTime_ == playTime_) {

			base_.qGoal_(0) = 4.55;
			base_.qGoal_(1) = 0.9;
			base_.qGoal_(2) = M_PI/2.0;

			_nh_rrt.Obs[0].pos(0) = 2.7251;
			_nh_rrt.Obs[0].pos(1) = -0.5276;
			_nh_rrt.Obs[0].radius = 0.6;

			_nh_rrt.Obs[1].pos(0) = 4.5502;
			_nh_rrt.Obs[1].pos(1) = 2.0;
			_nh_rrt.Obs[1].radius = 0.5;

			_nh_rrt.Obs[2].pos(0) = 4.5;
			_nh_rrt.Obs[2].pos(1) = -2.825;
			_nh_rrt.Obs[2].radius = 0.1;

			_nh_rrt.obs_num = 3;
			_nh_rrt.base_length = 1.2; //1.14
			_nh_rrt.base_width = 0.7;

			// plan path
			NHRRT_planning(base_.qInit_, base_.qGoal_, _joint_target);

			// saved data // 
			// ifstream inFile2("/home/kendrick/catkin_ws/src/mobile_manipulator_motion_planner/path_result_table_ex1.txt"); // "reading"
			// MatrixXd joint_temp(200, 3);

			// int size = 0;
			// std::vector<std::string> parameters;
			// char inputString[1000];
			// while (!inFile2.eof())
			// { // eof : end of file
			// 	inFile2.getline(inputString, 1000);
			// 	boost::split(parameters, inputString, boost::is_any_of(","));
			// 	if (parameters.size() == 3)
			// 	{
			// 		for (int j = 0; j < parameters.size(); j++)
			// 			joint_temp(size, j) = atof(parameters[j].c_str()); // c_str() : string -> char* // atof : char* -> float
			// 		size++;
			// 	}
			// }
			// inFile2.close();
			// _joint_target = joint_temp.topLeftCorner(size, 3);
			///////////////////////////////////////////////////////////////////////////////

			target_state = 0;
			base_control_mode = 1;
			mobile_flag = true;
			target_num = _joint_target.rows();

			// _joint_target : Matrix of configurations (x, y, theta) 
		}
		joint_flag = false;

		NHRRT_controller();

		break;


	case REACH_CUP:
		if (controlStartTime_ == playTime_) {	
			_rrt.box_num = 5; // 2 -> door and box
			_rrt.box_num2 = 3; // 3 box2 -> manipulator's link

			// // door
			_rrt.Box1[0].fAxis = Vector3d(0.6, 0.13, 0.3); // axis length 
			_rrt.Box1[0].vAxis[0] = base_.rotInit_*Vector3d(1, 0, 0);
			_rrt.Box1[0].vAxis[1] = base_.rotInit_*Vector3d(0, 1, 0);
			_rrt.Box1[0].vAxis[2] = base_.rotInit_*Vector3d(0, 0, 1);
			_rrt.Box1[0].vPos = base_.rotInit_.transpose()*Vector3d(4.9250 - base_.qInit_(0), -2.8250 - base_.qInit_(1), 1.6250); // axis center pos

			_rrt.Box1[1].fAxis = Vector3d(0.6, 0.13, 0.3); // axis length 
			_rrt.Box1[1].vAxis[0] = base_.rotInit_*Vector3d(1, 0, 0);
			_rrt.Box1[1].vAxis[1] = base_.rotInit_*Vector3d(0, 1, 0);
			_rrt.Box1[1].vAxis[2] = base_.rotInit_*Vector3d(0, 0, 1);
			_rrt.Box1[1].vPos = base_.rotInit_.transpose()*Vector3d(4.9250 - base_.qInit_(0), -2.8250 - base_.qInit_(1), 0.4250); // axis center pos

			_rrt.Box1[2].fAxis = Vector3d(0.15, 0.3, 0.5); // axis length 
			_rrt.Box1[2].vAxis[0] = base_.rotInit_*Vector3d(1, 0, 0);
			_rrt.Box1[2].vAxis[1] = base_.rotInit_*Vector3d(0, 1, 0);
			_rrt.Box1[2].vAxis[2] = base_.rotInit_*Vector3d(0, 0, 1);
			_rrt.Box1[2].vPos = base_.rotInit_.transpose()*Vector3d(5.2 - base_.qInit_(0), -2.1250 - base_.qInit_(1), 1.05); // axis center pos
	
			_rrt.Box1[3].fAxis = Vector3d(0.6, 0.13, 0.01); // axis length 
			_rrt.Box1[3].vAxis[0] = base_.rotInit_*Vector3d(1, 0, 0);
			_rrt.Box1[3].vAxis[1] = base_.rotInit_*Vector3d(0, 1, 0);
			_rrt.Box1[3].vAxis[2] = base_.rotInit_*Vector3d(0, 0, 1);
			_rrt.Box1[3].vPos = base_.rotInit_.transpose()*Vector3d(4.925 - base_.qInit_(0), -2.825 - base_.qInit_(1), 1.04); // axis center pos


			_rrt.Box1[4].fAxis = Vector3d(0.01, 0.01, 1.0); // axis length 
			_rrt.Box1[4].vAxis[0] = base_.rotInit_*Vector3d(1, 0, 0);
			_rrt.Box1[4].vAxis[1] = base_.rotInit_*Vector3d(0, 1, 0);
			_rrt.Box1[4].vAxis[2] = base_.rotInit_*Vector3d(0, 0, 1);
			_rrt.Box1[4].vPos = base_.rotInit_.transpose()*Vector3d(4.3050 - base_.qInit_(0), -2.6850 - base_.qInit_(1), 1.00); // axis center pos

			//for (int i=0;i<4;i++)
			//cout << _rrt.Box1[i].vPos.transpose() << endl;




			rot_diff = Rotate_with_X(0.0) * Rotate_with_Y(-M_PI / 2.0) * Rotate_with_Z(-M_PI / 4.0);
			Quaterniond a1(rot_diff);
			rot_diff_vec = a1.toRotationMatrix().eulerAngles(0, 1, 2);
			target_.head(3) << 4.75, -2.55, 1.15;
			target_.tail(3) = rot_diff_vec;

			TRAC_IK_solver(target_, true, _robot_left);
			RRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target, true); // true -> left planning

/************************************************************************************************************************/

			_rrt.box_num = 5;
			_rrt.box_num2 = 3;
			_rrt.C.resize(6, 2);
			_rrt.C.setZero(); // max , min
			_rrt.C(0, 0) = 0.15;
			_rrt.C(1, 0) = 0.01;
			_rrt.C(2, 0) = 0.01;

			_rrt.C(0, 1) = -0.1;
			_rrt.C(1, 1) = -0.00;
			_rrt.C(2, 1) = -0.00;
	
			target_(1) -= 0.13;

			joint_left_.qInit_ = joint_left_.qGoal_;
			TRAC_IK_solver(target_, true, _robot_left);
			VectorXd q_goal[5];
			q_goal[0] = joint_left_.qGoal_;
			TRAC_IK_solver(target_, true, _robot_left);
			q_goal[1] = joint_left_.qGoal_;
			TRAC_IK_solver(target_, true, _robot_left);
			q_goal[2] = joint_left_.qGoal_;
			TRAC_IK_solver(target_, true, _robot_left);
			q_goal[3] = joint_left_.qGoal_;
			TRAC_IK_solver(target_, true, _robot_left);
			q_goal[4] = joint_left_.qGoal_;

			int min = 0;
			double mindist = _rrt.vector_norm(joint_left_.qInit_, q_goal[0], dof);
			for (int i = 1; i< 5; i++) { // Ignoring index 0 as it is preselected
			double current = _rrt.vector_norm(joint_left_.qInit_, q_goal[i], dof);
			if (current < mindist) {
				min = i;
				mindist = current;
			}
			}
			joint_left_.qGoal_ = q_goal[min];

			Vector3d pos_temp1 = CalcBodyToBaseCoordinates(*model_l, joint_left_.qInit_,  body_id_l[dof - 1], com_position_l[dof - 1],true ); // get position and rotation in EE frame;
			Vector3d pos_temp2 = CalcBodyToBaseCoordinates(*model_l, joint_left_.qGoal_,  body_id_l[dof - 1], com_position_l[dof - 1],true ); // get position and rotation in EE frame;

			_rrt.refer_pos = pos_temp1; // axis center pos
			_rrt.refer_rot = Rotate_with_X(0.0) * Rotate_with_Y(-M_PI / 2.0) * Rotate_with_Z(-M_PI / 4.0);

			_rrt.constraint_axis[0] = true;
			_rrt.constraint_axis[1] = true;
			_rrt.constraint_axis[2] = true;

			CRRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target2, true);

			int case_num;
			int rows_ = _joint_target2.rows();
			if(rows_ < 20)
			case_num = 0;
			else
			case_num = 1;


			target_1 = _rrt.MergeRRTResults(_joint_target, _joint_target2, case_num);

			//cout << target_1 << endl;
			// _joint_target : Matrix of configurations (degrees) 

		}
		joint_flag = false;
		target_num = target_1.rows();

		if (target_num != target_state)
			joint_target_left_.q_ = target_1.row(target_state) * DEGREE;

		joint_flag = JointPIDControl(1.0 * Hz, true);

		if (joint_flag && target_state < target_num) {
			controlStartTime_ = playTime_;
			joint_left_.qInit_ = joint_left_.q_; // why update qInit ? Because PID controller consider initial state
			target_state++;
		}

		joint_target_right_.q_.setZero();
		joint_target_right_.q_(0) = -60.0 * DEGREE;
		joint_target_right_.q_(1) = -45.0 * DEGREE;
		joint_target_right_.q_(2) = 0.0 * DEGREE;
		joint_target_right_.q_(3) = -120.0 * DEGREE;
		joint_target_right_.q_(4) = 0.0 * DEGREE;
		joint_target_right_.q_(5) = 90.0 * DEGREE;
		joint_target_right_.q_(6) = 0.0 * DEGREE;
		JointPIDControl(0.1*Hz, false); // false : Right Arm Control, true : Left Arm Control

		break;
	case MOVE_CUP:
		if (controlStartTime_ == playTime_) {
			_rrt.box_num = 5; // 2 -> door and box
			_rrt.box_num2 = 4; // 3 box2 -> manipulator's link

			rot_diff = Rotate_with_X(0.0) * Rotate_with_Y(-M_PI / 2.0) * Rotate_with_Z(-M_PI / 4.0);
			Quaterniond a1(rot_diff);
			rot_diff_vec = a1.toRotationMatrix().eulerAngles(0, 1, 2);
			target_.head(3) << 4.75, -2.4, 1.15;
			target_.tail(3) = rot_diff_vec;

			TRAC_IK_solver(target_, true, _robot_left);
			VectorXd q_goal[5];
			q_goal[0] = joint_left_.qGoal_;
			TRAC_IK_solver(target_, true, _robot_left);
			q_goal[1] = joint_left_.qGoal_;
			TRAC_IK_solver(target_, true, _robot_left);
			q_goal[2] = joint_left_.qGoal_;
			TRAC_IK_solver(target_, true, _robot_left);
			q_goal[3] = joint_left_.qGoal_;
			TRAC_IK_solver(target_, true, _robot_left);
			q_goal[4] = joint_left_.qGoal_;

			int min = 0;
			double mindist = _rrt.vector_norm(joint_left_.qInit_, q_goal[0], dof);
			for (int i = 1; i < 5; i++)
			{ // Ignoring index 0 as it is preselected
				double current = _rrt.vector_norm(joint_left_.qInit_, q_goal[i], dof);
				if (current < mindist)
				{
					min = i;
					mindist = current;
				}
			}

			joint_left_.qGoal_ = q_goal[min];

			_rrt.C.resize(6, 2);
			_rrt.C.setZero(); // max , min
			_rrt.C(0, 0) = 0.01;
			_rrt.C(1, 0) = 0.02;
			_rrt.C(2, 0) = 0.02;

			_rrt.C(0, 1) = -0.01;
			_rrt.C(1, 1) = -0.01;
			_rrt.C(2, 1) = -0.02;

			Vector3d pos_temp1 = CalcBodyToBaseCoordinates(*model_l, joint_left_.qGoal_,  body_id_l[dof - 1], com_position_l[dof - 1],true ); // get position and rotation in EE frame;

			_rrt.refer_pos = pos_temp1; // axis center pos
			_rrt.refer_rot = Rotate_with_X(0.0) * Rotate_with_Y(-M_PI / 2.0) * Rotate_with_Z(-M_PI / 4.0);

			_rrt.constraint_axis[0] = false;
			_rrt.constraint_axis[1] = false;
			_rrt.constraint_axis[2] = false;

			CRRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target2, true);


			int case_num;
			int rows_ = _joint_target2.rows();
			if(rows_ < 20)
			case_num = 0;
			else
			case_num = 1;
			_joint_target = joint_left_.qInit_.transpose()/M_PI*180.0;


			target_1 = _rrt.MergeRRTResults(_joint_target, _joint_target2, case_num);

			cout << target_1 << endl;

		}
		joint_flag = false;
		target_num = target_1.rows();

		if (target_num != target_state)
			joint_target_left_.q_ = target_1.row(target_state) * DEGREE;

		joint_flag = JointPIDControl(2.0 * Hz, true);

		if (joint_flag && target_state < target_num) {
			controlStartTime_ = playTime_;
			joint_left_.qInit_ = joint_left_.q_;
			target_state++;
		}

		joint_target_right_.q_.setZero();
		joint_target_right_.q_(0) = -60.0 * DEGREE;
		joint_target_right_.q_(1) = -45.0 * DEGREE;
		joint_target_right_.q_(2) = 0.0 * DEGREE;
		joint_target_right_.q_(3) = -120.0 * DEGREE;
		joint_target_right_.q_(4) = 0.0 * DEGREE;
		joint_target_right_.q_(5) = 90.0 * DEGREE;
		joint_target_right_.q_(6) = 0.0 * DEGREE;
		JointPIDControl(0.1*Hz, false); // false : Right Arm Control, true : Left Arm Control

		break;
	case REACH_MILK:
		if (controlStartTime_ == playTime_) {
			_rrt.box_num = 6; // 2 -> door and box
			_rrt.box_num2 = 3; // 3 box2 -> manipulator's link

			// // door
			_rrt.Box1[0].fAxis = Vector3d(0.6, 0.13, 0.3); // upper stack
			_rrt.Box1[0].vAxis[0] = base_.rotInit_*Vector3d(1, 0, 0);
			_rrt.Box1[0].vAxis[1] = base_.rotInit_*Vector3d(0, 1, 0);
			_rrt.Box1[0].vAxis[2] = base_.rotInit_*Vector3d(0, 0, 1);
			_rrt.Box1[0].vPos = base_.rotInit_.transpose()*Vector3d(4.9250 - base_.qInit_(0), -2.8250 - base_.qInit_(1), 1.6250); // axis center pos

			_rrt.Box1[1].fAxis = Vector3d(0.6, 0.13, 0.3); // lower stack
			_rrt.Box1[1].vAxis[0] = base_.rotInit_*Vector3d(1, 0, 0);
			_rrt.Box1[1].vAxis[1] = base_.rotInit_*Vector3d(0, 1, 0);
			_rrt.Box1[1].vAxis[2] = base_.rotInit_*Vector3d(0, 0, 1);
			_rrt.Box1[1].vPos = base_.rotInit_.transpose()*Vector3d(4.9250 - base_.qInit_(0), -2.8250 - base_.qInit_(1), 0.4250); // axis center pos

			_rrt.Box1[2].fAxis = Vector3d(0.15, 0.3, 0.5); // left arm
			_rrt.Box1[2].vAxis[0] = base_.rotInit_*Vector3d(1, 0, 0);
			_rrt.Box1[2].vAxis[1] = base_.rotInit_*Vector3d(0, 1, 0);
			_rrt.Box1[2].vAxis[2] = base_.rotInit_*Vector3d(0, 0, 1);
			_rrt.Box1[2].vPos = base_.rotInit_.transpose()*Vector3d(4.55 - base_.qInit_(0), -2.1250 - base_.qInit_(1), 1.05); // axis center pos
	
			_rrt.Box1[3].fAxis = Vector3d(0.6, 0.13, 0.01); // middle stack
			_rrt.Box1[3].vAxis[0] = base_.rotInit_*Vector3d(1, 0, 0);
			_rrt.Box1[3].vAxis[1] = base_.rotInit_*Vector3d(0, 1, 0);
			_rrt.Box1[3].vAxis[2] = base_.rotInit_*Vector3d(0, 0, 1);
			_rrt.Box1[3].vPos = base_.rotInit_.transpose()*Vector3d(4.925 - base_.qInit_(0), -2.825 - base_.qInit_(1), 1.04); // axis center pos

			_rrt.Box1[4].fAxis = Vector3d(0.01, 0.01, 1.0); // left columns
			_rrt.Box1[4].vAxis[0] = base_.rotInit_*Vector3d(1, 0, 0);
			_rrt.Box1[4].vAxis[1] = base_.rotInit_*Vector3d(0, 1, 0);
			_rrt.Box1[4].vAxis[2] = base_.rotInit_*Vector3d(0, 0, 1);
			_rrt.Box1[4].vPos = base_.rotInit_.transpose()*Vector3d(5.495 - base_.qInit_(0), -2.6850 - base_.qInit_(1), 1.00); // axis center pos

			_rrt.Box1[5].fAxis = Vector3d(0.03, 0.03, 0.125); // milk
			_rrt.Box1[5].vAxis[0] = base_.rotInit_*Vector3d(1, 0, 0);
			_rrt.Box1[5].vAxis[1] = base_.rotInit_*Vector3d(0, 1, 0);
			_rrt.Box1[5].vAxis[2] = base_.rotInit_*Vector3d(0, 0, 1);
			_rrt.Box1[5].vPos = base_.rotInit_.transpose()*Vector3d(5.1 - base_.qInit_(0), -2.825 - base_.qInit_(1), 1.175); // axis center pos


			//for (int i=0;i<4;i++)
			//cout << _rrt.Box1[i].vPos.transpose() << endl;


			rot_diff = Rotate_with_X(0.0) * Rotate_with_Y(-M_PI / 2.0) * Rotate_with_Z(-M_PI / 4.0);
			Quaterniond a1(rot_diff);
			rot_diff_vec = a1.toRotationMatrix().eulerAngles(0, 1, 2);
			target_.head(3) << 5.1, -2.675, 1.175;
			target_.tail(3) = rot_diff_vec;

			TRAC_IK_solver(target_, false, _robot_right);
			RRT_planning(joint_right_.qInit_, joint_right_.qGoal_, _joint_target, false); // true -> left planning

/************************************************************************************************************************/

			_rrt.box_num = 5;
			_rrt.box_num2 = 3;
			_rrt.C.resize(6, 2);
			_rrt.C.setZero(); // max , min
			_rrt.C(0, 0) = 0.15;
			_rrt.C(1, 0) = 0.01;
			_rrt.C(2, 0) = 0.01;

			_rrt.C(0, 1) = -0.1;
			_rrt.C(1, 1) = -0.00;
			_rrt.C(2, 1) = -0.00;
	
			target_(1) -= 0.05;

			joint_right_.qInit_ = joint_right_.qGoal_;


			TRAC_IK_solver(target_, false, _robot_right);
			VectorXd q_goal[5];
			q_goal[0] = joint_right_.qGoal_;
			TRAC_IK_solver(target_, false, _robot_right);
			q_goal[1] = joint_right_.qGoal_;
			TRAC_IK_solver(target_, false, _robot_right);
			q_goal[2] = joint_right_.qGoal_;
			TRAC_IK_solver(target_, false, _robot_right);
			q_goal[3] = joint_right_.qGoal_;
			TRAC_IK_solver(target_, false, _robot_right);
			q_goal[4] = joint_right_.qGoal_;

			int min = 0;
			double mindist = _rrt.vector_norm(joint_right_.qInit_, q_goal[0], dof);
			for (int i = 1; i < 5; i++)
			{ // Ignoring index 0 as it is preselected
				double current = _rrt.vector_norm(joint_right_.qInit_, q_goal[i], dof);
				if (current < mindist)
				{
					min = i;
					mindist = current;
				}
			}
			joint_right_.qGoal_ = q_goal[min];

			Vector3d pos_temp1 = CalcBodyToBaseCoordinates(*model_r, joint_right_.qInit_,  body_id_r[dof - 1], com_position_r[dof - 1],true ); // get position and rotation in EE frame;
			Vector3d pos_temp2 = CalcBodyToBaseCoordinates(*model_r, joint_right_.qGoal_,  body_id_r[dof - 1], com_position_r[dof - 1],true ); // get position and rotation in EE frame;

			_rrt.refer_pos = pos_temp1; // axis center pos
			_rrt.refer_rot = Rotate_with_X(0.0) * Rotate_with_Y(-M_PI / 2.0) * Rotate_with_Z(-M_PI / 4.0);

			_rrt.constraint_axis[0] = true;
			_rrt.constraint_axis[1] = true;
			_rrt.constraint_axis[2] = true;

			CRRT_planning(joint_right_.qInit_, joint_right_.qGoal_, _joint_target2, false);

			int case_num;
			int rows_ = _joint_target2.rows();
			if(rows_ < 20)
			case_num = 0;
			else
			case_num = 1;


			target_1 = _rrt.MergeRRTResults(_joint_target, _joint_target2, case_num);

			cout << target_1 << endl;
			//_joint_target : Matrix of configurations (degrees) 
		}

		joint_flag = false;
		target_num = target_1.rows();

		if (target_num != target_state)
			joint_target_right_.q_ = target_1.row(target_state) * DEGREE;

		joint_flag = JointPIDControl(1.0 * Hz, false);

		if (joint_flag && target_state < target_num) {
			controlStartTime_ = playTime_;
			joint_right_.qInit_ = joint_right_.q_; // why update qInit ? Because PID controller consider initial state
			target_state++;
		}


		joint_target_left_.q_.setZero();
		joint_target_left_.q_(0) = 60.0 * DEGREE;
		joint_target_left_.q_(1) = -45.0 * DEGREE;
		joint_target_left_.q_(2) = 0.0 * DEGREE;
		joint_target_left_.q_(3) = -90.0 * DEGREE;
		joint_target_left_.q_(4) = 0.0 * DEGREE;
		joint_target_left_.q_(5) = 90.0 * DEGREE;
		joint_target_left_.q_(6) = 0.0 * DEGREE;
		JointPIDControl(0.1*Hz, true); // false : Right Arm Control, true : Left Arm Control

		break;

	case MOVE_MILK:
		if (controlStartTime_ == playTime_) {
			_rrt.box_num = 4;
			_rrt.box_num2 = 1;


			_rrt.Box1[0].fAxis = Vector3d(0.04, 0.04, 0.12); // axis length 
			_rrt.Box1[0].vAxis[0] = Vector3d(1, 0, 0);
			_rrt.Box1[0].vAxis[1] = Vector3d(0, 1, 0);
			_rrt.Box1[0].vAxis[2] = Vector3d(0, 0, 1);
			_rrt.Box1[0].vPos = Vector3d(0.8250, -0.250, 0.8242); // axis center pos

			_rrt.Box1[1].fAxis = Vector3d(0.05, 0.05, 0.20);
			_rrt.Box1[2].fAxis = Vector3d(0.05, 0.05, 0.20);


			_rrt.Box1[3].fAxis = Vector3d(0.6, 0.6, 0.6); // axis length 
			_rrt.Box1[3].vAxis[0] = Vector3d(1, 0, 0);
			_rrt.Box1[3].vAxis[1] = Vector3d(0, 1, 0);
			_rrt.Box1[3].vAxis[2] = Vector3d(0, 0, 1);
			_rrt.Box1[3].vPos = Vector3d(1.1, -0.15, 0.2); // axis center pos

			//_rrt.Box1[1].vAxis[0] = Vector3d(1, 0, 0);
			//_rrt.Box1[1].vAxis[1] = Vector3d(0, 1, 0);
			//_rrt.Box1[1].vAxis[2] = Vector3d(0, 0, 1);
			//_rrt.Box1[1].vPos = Vector3d(0.7750, -0.0, 1.2250);
//			cout << "qinit" << joint_left_.qInit_.transpose() << endl;

		//	IK_solver(Vector3d(0.65, -0.25, 0.85), Vector3d(0.0, -M_PI / 2.0, -M_PI / 4.0), true);
			RRT_planning(joint_left_.qInit_, joint_left_.qGoal_, _joint_target, true); // true -> left planning
																					   // _joint_target : Matrix of configurations (degrees) 
		}
		joint_flag = false;
		target_num = _joint_target.rows();

		if (target_num != target_state)
			joint_target_left_.q_ = _joint_target.row(target_state) * DEGREE;

		joint_flag = JointPIDControl(2.0 * Hz, true);

		if (joint_flag && target_state < target_num) {
			controlStartTime_ = playTime_;
			joint_left_.qInit_ = joint_left_.q_; // why update qInit ? Because PID controller consider initial state
			target_state++;
		}

		joint_target_right_.q_.setZero();
		joint_target_right_.q_(0) = -60.0 * DEGREE;
		joint_target_right_.q_(1) = -45.0 * DEGREE;
		joint_target_right_.q_(2) = 0.0 * DEGREE;
		joint_target_right_.q_(3) = -120.0 * DEGREE;
		joint_target_right_.q_(4) = 0.0 * DEGREE;
		joint_target_right_.q_(5) = 90.0 * DEGREE;
		joint_target_right_.q_(6) = 0.0 * DEGREE;
		JointPIDControl(0.1*Hz, false); // false : Right Arm Control, true : Left Arm Control

		break;
	case DEFAULT:
		joint_target_left_.desired_q_ = joint_left_.qInit_;
		joint_target_right_.desired_q_ = joint_right_.qInit_;
		break;
	}

	playTime_++;
}

// \B3\BB\B9\AB function
bool Controller::JointPIDControl(double duration, bool left) {
	bool res = false;
	if (left == true) {
		for (int i = 0; i < dof; i++)
			joint_target_left_.cubic_q_(i) = JCubic(playTime_, controlStartTime_, controlStartTime_ + duration, joint_left_.qInit_(i), 0.0, joint_target_left_.q_(i), 0.0);
		joint_target_left_.desired_q_ = joint_target_left_.cubic_q_;
	}
	else {
		for (int i = 0; i < dof; i++)
			joint_target_right_.cubic_q_(i) = JCubic(playTime_, controlStartTime_, controlStartTime_ + duration, joint_right_.qInit_(i), 0.0, joint_target_right_.q_(i), 0.0);
		joint_target_right_.desired_q_ = joint_target_right_.cubic_q_;
	}
	if (controlStartTime_ + duration < playTime_)
		res = true;

	return res;
}
bool Controller::MobilePIDControl(double duration) {
	bool res = false;

	for (int i = 0; i < 2; i++)
		base_.cubic_q_(i) = Cubic(playTime_, controlStartTime_, controlStartTime_ + duration, base_.qInit_(i), 0.0, base_.target_q_(i), 0.0);

	return res;
}
void Controller::readdata(VectorXd &ql, VectorXd &qr, VectorXd &qldot, VectorXd &qrdot, Vector3d base_pos)
{
	joint_left_.q_ = ql;
	joint_left_.qdot_ = qldot;
	joint_right_.q_ = qr;
	joint_right_.qdot_ = qrdot;
	base_.q_ = base_pos;
}
void Controller::writedata(VectorXd& ql, VectorXd& qr, VectorXd& base) {
	ql = joint_target_left_.desired_q_;
	qr = joint_target_right_.desired_q_;
	base = base_desired_vel_;
}
void Controller::TRAC_IK_solver(Vector6d pose, bool left, Robotmodel& model)
{
	double eps = 1e-7;
	double num_samples = 1000;
	double timeout = 0.005;
	std::string chain_start = "panda_link0";
	std::string chain_end = "panda_link8";
	TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);

	KDL::Chain chain;
	KDL::JntArray ll, ul; //lower joint limits, upper joint limits

	bool valid = tracik_solver.getKDLChain(chain);

	if (!valid)
	{
		ROS_ERROR("there was no valid KDL chain found");
		return;
	}

	valid = tracik_solver.getKDLLimits(ll, ul);

	if (!valid)
	{
		ROS_ERROR("there was no valid KDL joint limits found");
		return;
	}

	assert(chain.getNrOfJoints() == ll.data.size());
	assert(chain.getNrOfJoints() == ul.data.size());


	// Set up KDL IK
	KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver
	KDL::ChainIkSolverVel_pinv vik_solver(chain); // PseudoInverse vel solver
	KDL::ChainIkSolverPos_NR_JL kdl_solver(chain, ll, ul, fk_solver, vik_solver, 1, eps); // Joint Limit Solver
  	// 1 iteration per solve (will wrap in timed loop to compare with TRAC-IK)

	// Create Nominal chain configuration midway between all joint limits
  	KDL::JntArray nominal(chain.getNrOfJoints());

	  for (size_t j = 0; j < nominal.data.size(); j++)
  	{	
    	nominal(j) = (ll(j) + ul(j)) / 2.0;
  	}

	// Create desired number of valid, random joint configurations
 	std::vector<KDL::JntArray> JointList;
  	KDL::JntArray q(chain.getNrOfJoints());
	
	for (uint i = 0; i < num_samples; i++)
  	{
    	for (uint j = 0; j < ll.data.size(); j++)
    	{
      		q(j) = fRand(ll(j), ul(j));
    	}
    	JointList.push_back(q);
 	}

	boost::posix_time::ptime start_time;
	boost::posix_time::time_duration diff;

	KDL::JntArray result;
	KDL::Frame end_effector_pose;

	Vector3d temp;
	Vector3d base_to_joint;
	temp.setZero();
	Vector4d x_pos;
	Matrix3d Rot_d;
	Matrix3d Rot_base;
	Matrix4d trans;
	trans.setIdentity();
	_rrt.left = left; // necessary for checking collision


	if (left)
	{
		Rot_base = Rotate_with_Z(base_.qInit_(2))*Rot_l;

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				trans(i, j) = Rot_base(i, j);
			}
		}
		// Global to Local frame
		base_to_joint(0) = 0.2663;
		base_to_joint(1) = -0.2688;
		base_to_joint(2) = 0.9639;

		temp.head(2) = base_.qInit_.head(2);
		temp += Rotate_with_Z(base_.qInit_(2))*base_to_joint; // base to joint 1 (left) : local to global

		temp = Rot_base.transpose() * temp;
		temp(2) -= 0.33;
		temp = Rot_base * temp;
		trans(0, 3) = temp(0);
		trans(1, 3) = temp(1);
		trans(2, 3) = temp(2);

		x_pos << pose(0), pose(1), pose(2), 1;

	//	ROS_INFO_STREAM("Target position(global) : " << x_pos.transpose());
		x_pos = trans.inverse() * x_pos;
	//	ROS_INFO_STREAM("Target position(local) : " << x_pos.transpose());
		
		Rot_d = Rotate_with_X(pose(3)) * Rotate_with_Y(pose(4)) * Rotate_with_Z(pose(5));
		Rot_d = Rot_l.transpose() * Rot_d;
	}
	else
	{
		Rot_base = Rotate_with_Z(base_.qInit_(2))*Rot_r;

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				trans(i, j) = Rot_base(i, j);
			}
		}

		base_to_joint(0) = 0.2663;
		base_to_joint(1) = 0.2690;
		base_to_joint(2) = 0.9639;

		temp.head(2) = base_.qInit_.head(2);
		temp += Rotate_with_Z(base_.qInit_(2))*base_to_joint; // base to joint 1 (left) : local to global

		temp = Rot_base.transpose() * temp;
		temp(2) -= 0.33;
		temp = Rot_base * temp;
		trans(0, 3) = temp(0);
		trans(1, 3) = temp(1);
		trans(2, 3) = temp(2);

		x_pos << pose(0), pose(1), pose(2), 1;

	//	ROS_INFO_STREAM("Target position(global) : " << x_pos.transpose());
		x_pos = trans.inverse() * x_pos;
	//	ROS_INFO_STREAM("Target position(local) : " << x_pos.transpose());

		Rot_d = Rotate_with_X(pose(3)) * Rotate_with_Y(pose(4)) * Rotate_with_Z(pose(5));
		Rot_d = Rot_r.transpose() * Rot_d;

	}

	for (int i = 0; i < 3; i++)
	{
		end_effector_pose.p(i) = x_pos(i);
	}
	
	KDL::Rotation A;
	A.data[0] = Rot_d(0,0); 
	A.data[1] = Rot_d(0,1);
	A.data[2] = Rot_d(0,2);
	A.data[3] = Rot_d(1,0);
	A.data[4] = Rot_d(1,1);
	A.data[5] = Rot_d(1,2);
	A.data[6] = Rot_d(2,0); 
	A.data[7] = Rot_d(2,1);
	A.data[8] = Rot_d(2,2);
	end_effector_pose.M = A;

	int rc;

	double total_time = 0;
	uint success = 0;

  //	ROS_INFO_STREAM("*** Testing TRAC-IK with " << num_samples << " random samples");


	while(true)//for (uint i = 0; i < num_samples; i++)
	{
		//fk_solver.JntToCart(JointList[i], end_effector_pose);
		double elapsed = 0;
		start_time = boost::posix_time::microsec_clock::local_time();
		rc = tracik_solver.CartToJnt(nominal, end_effector_pose, result);
		std::vector<double> config;
		config.clear();
		if (rc >= 0)
		{
			for (int i = 0; i < dof; i++)
			{
				config.push_back(result.data(i) * 180 / M_PI);
			}
			if (!_rrt.CheckCollision(model, config)){
				break;
			}
			else{
				continue;
			}
		}
	}

	ROS_INFO_STREAM("IK solution is : " << result.data.transpose() *180/ M_PI   );

	if (left) {joint_left_.qGoal_ = result.data;}
	else {joint_right_.qGoal_ = result.data;}
}
void Controller::NHRRT_controller()
{
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////// base_control_mode = 0 : rotate to the next target orientation at current position (during path following)
	////// base_control_mode = 1 : path following by using pure pursuit controller
	////// base_control_mode = 2 : path following to last goal point by using S matrix controller
	////// base_control_mode = 3 : rotate to the goal orientation at goal position
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	mobile_b = 0.5;
	mobile_c = 0.165 / (2 * mobile_b);
	mobile_d = 0.02;

	base_nhrrt_vel = 1.5; //2
	base_nhrrt_b = 0.545;
	base_nhrrt_thereshold = 0.5; //0.6

	if ((base_control_mode == 1)||(base_control_mode == 2)) {

		dist_base_to_point = sqrt(pow((_joint_target.row(target_state)(0) - base_.q_(0)), 2) + pow((_joint_target.row(target_state)(1) - base_.q_(1)), 2));

		if ((target_num - 1) <= target_state) {
			base_nhrrt_thereshold = 0.005;
		}

		if (dist_base_to_point < base_nhrrt_thereshold) {
			target_state++;
			base_control_mode = 1;

			if ((target_num) > target_state) {
				temp = base_.q_(2) - _joint_target.row(target_state)(2);
				temp = _nh_rrt.norm_angle(temp, -M_PI);
				if (abs(temp) > 1.047) { // 60 degrees
					base_control_mode = 0;
				}
				target_to_vrep_ = _joint_target.row(target_state);
			}
			if ((target_num - 1) == target_state) {
				base_control_mode = 2;
			}
			if ((target_num) == target_state) {
				base_control_mode = 3;
			}
		}
	}

	switch (base_control_mode) {
	case 0:
		temp = base_.q_(2) - _joint_target.row(target_state)(2);
		temp = 5 * _nh_rrt.norm_angle(temp, -M_PI); // rotation gain: 5
		if (abs(temp) > 1) { // saturation
			temp = 1 * temp / abs(temp); // max_vel : 1
		}

		base_desired_vel_(0) = 1 * temp;
		base_desired_vel_(1) = -1 * temp;
		base_desired_vel_(2) = -1 * temp;
		base_desired_vel_(3) = 1 * temp;

		if (abs(temp) < 0.785) { // rotate until it reaches 45 degrees
			base_control_mode = 1;
		}

		break;
	case 1:
		nhrrt_buf = -(_joint_target.row(target_state)(0) - base_.q_(0))*sin(base_.q_(2)) + (_joint_target.row(target_state)(1) - base_.q_(1))*cos(base_.q_(2));
		nhrrt_r = pow(dist_base_to_point, 2) / (2 * nhrrt_buf);
		nhrrt_v_r = ((base_nhrrt_b / nhrrt_r) + 1) * base_nhrrt_vel;
		nhrrt_v_l = (1 - (base_nhrrt_b / nhrrt_r)) * base_nhrrt_vel;

		base_desired_vel_(0) = nhrrt_v_l;
		base_desired_vel_(1) = nhrrt_v_r;
		base_desired_vel_(2) = nhrrt_v_r;
		base_desired_vel_(3) = nhrrt_v_l;

		break;
	case 2:
		mobile_S(0, 0) = mobile_c * (mobile_b * cos(base_.q_(2)) - mobile_d * sin(base_.q_(2)));
		mobile_S(0, 1) = mobile_c * (mobile_b * cos(base_.q_(2)) + mobile_d * sin(base_.q_(2)));
		mobile_S(1, 0) = mobile_c * (mobile_b * sin(base_.q_(2)) + mobile_d * cos(base_.q_(2)));
		mobile_S(1, 1) = mobile_c * (mobile_b * sin(base_.q_(2)) - mobile_d * cos(base_.q_(2)));

		//mobile_v_err(0) = -(base_.q_(0) - base_.q_pre_(0)) / Hz;
		//mobile_v_err(1) = -(base_.q_(1) - base_.q_pre_(1)) / Hz;

		base_.target_q_(0) = _joint_target.row(target_state)(0);
		base_.target_q_(1) = _joint_target.row(target_state)(1);

		if (mobile_flag) {
			controlStartTime_ = playTime_;
			base_.qInit_(0) = base_.q_(0);
			base_.qInit_(1) = base_.q_(1);
			mobile_flag = false;
		}
		mobile_flag = MobilePIDControl(3.0 * Hz); //cubic trajectory

		mobile_p_err(0) = base_.cubic_q_(0) - base_.q_(0);
		mobile_p_err(1) = base_.cubic_q_(1) - base_.q_(1);

		//mobile_v = mobile_S.inverse() * (0.7 * mobile_p_err + 0.05 * mobile_v_err);
		mobile_v = mobile_S.inverse() * (0.7 * mobile_p_err);

		for (int i = 0; i < 2; i++) {
			if (abs(mobile_v(i)) > 2) {
				mobile_v(i) = 2 * mobile_v(i) / abs(mobile_v(i)); // max_vel : 2
			}
		}

		base_desired_vel_(0) = mobile_v(1);
		base_desired_vel_(1) = mobile_v(0);
		base_desired_vel_(2) = mobile_v(0);
		base_desired_vel_(3) = mobile_v(1);

		break;
	case 3:
		temp = base_.q_(2) - base_.qGoal_(2);
		temp = 3 * _nh_rrt.norm_angle(temp, -M_PI); // rotation gain: 3
		if (abs(temp) > 1) {
			temp = 1 * temp / abs(temp); // max_vel : 1
		}
		base_desired_vel_(0) = 1 * temp;
		base_desired_vel_(1) = -1 * temp;
		base_desired_vel_(2) = -1 * temp;
		base_desired_vel_(3) = 1 * temp;

		//cout << "base ori: " << base_.q_(2) << endl;

		break;
	default:
		break;
	}
	base_.q_pre_ = base_.q_;

}

void Controller::RRT_planning(VectorXd q_start, VectorXd q_goal, MatrixXd& joint_target, bool left)
{


	_rrt.left = left;
	if (left) {
		_rrt.lower_limit = joint_limit_left_.lower;
		_rrt.upper_limit = joint_limit_left_.upper;
		_rrt.DoF_size = dof;
	}
	else
	{
		_rrt.lower_limit = joint_limit_right_.lower;
		_rrt.upper_limit = joint_limit_right_.upper;
		_rrt.DoF_size = dof;
	}
	_rrt.qinit = q_start;
	_rrt.qgoal = q_goal;

	ofstream outFile("path_result.txt", ios::out); // open file for "writing"
	bool a = false;
	while (!a)
		if (left)
			a = _rrt.StartRRT(_robot_left, outFile);
		else
			a = _rrt.StartRRT(_robot_right, outFile);
	ofstream outFile2("path_result2.txt", ios::out); // "writing" 
	// path_result -> Smooth -> path_result2
	ifstream inFile("path_result.txt"); // "reading"
	_rrt.SmoothPath(outFile2, inFile);

	outFile2.close();
	MatrixXd joint_temp(100, dof);
	if (!left)
		joint_temp.resize(100, dof);

	ifstream inFile2("path_result2.txt"); // "reading"
	int size = 0;
	std::vector<std::string> parameters;
	char inputString[1000];
	while (!inFile2.eof()) { // eof : end of file
		inFile2.getline(inputString, 1000);
		boost::split(parameters, inputString, boost::is_any_of(","));
		if (left) {
			if (parameters.size() == dof) {
				for (int j = 0; j < parameters.size(); j++)
					joint_temp(size, j) = atof(parameters[j].c_str()); // c_str() : string -> char* // atof : char* -> float
				size++;
			}
		}
		else {
			if (parameters.size() == dof) {
				for (int j = 0; j < parameters.size(); j++)
					joint_temp(size, j) = atof(parameters[j].c_str());
				size++;
			}
		}
	}
	cout << "trajectory size" << size << endl;
	inFile2.close();
	if (left)
		joint_target = joint_temp.topLeftCorner(size, dof);
	else
		joint_target = joint_temp.topLeftCorner(size, dof);
}
void Controller::CRRT_planning(VectorXd q_start, VectorXd q_goal, MatrixXd& joint_target, bool left)
{


	_rrt.left = left;
	if (left) {
		_rrt.lower_limit = joint_limit_left_.lower;
		_rrt.upper_limit = joint_limit_left_.upper;
		_rrt.DoF_size = dof;
	}
	else
	{
		_rrt.lower_limit = joint_limit_right_.lower;
		_rrt.upper_limit = joint_limit_right_.upper;
		_rrt.DoF_size = dof;
	}
	
	_rrt.qinit = q_start;
	_rrt.qgoal = q_goal;

	ofstream outFile3("path_result3.txt", ios::out);
	bool a = false;
	while (!a)
		if (left)
			a = _rrt.StartCRRT(_robot_left, outFile3);
		else
			a = _rrt.StartCRRT(_robot_right, outFile3);

	outFile3.close();

	MatrixXd joint_temp(5000, dof);
	if (!left)
		joint_temp.resize(5000, dof);

	ifstream inFile3("path_result3.txt");
	int size = 0;
	std::vector<std::string> parameters;
	char inputString[50000];
	while (!inFile3.eof()) {
		inFile3.getline(inputString, 50000);
		boost::split(parameters, inputString, boost::is_any_of(","));
		if (left) {
			if (parameters.size() == dof) {
				for (int j = 0; j < parameters.size(); j++)
					joint_temp(size, j) = atof(parameters[j].c_str());
				size++;
			}
		}
		else {
			if (parameters.size() == dof) {
				for (int j = 0; j < parameters.size(); j++)
					joint_temp(size, j) = atof(parameters[j].c_str());
				size++;
			}
		}
	}
	inFile3.close();
	cout << "size" << size << endl;

	if (left)
		joint_target = joint_temp.topLeftCorner(size, dof);
	else
		joint_target = joint_temp.topLeftCorner(size, dof);
}

void Controller::NHRRT_planning(VectorXd q_start, VectorXd q_goal, MatrixXd& joint_target)
{

	_nh_rrt.lower_limit_base = base_limit_.lower;
	_nh_rrt.upper_limit_base = base_limit_.upper;

	_nh_rrt.qinit = q_start;
	_nh_rrt.qgoal = q_goal;

	_nh_rrt.upper_limit_goal_bias(0) = q_goal(0) + 0.2;
	_nh_rrt.upper_limit_goal_bias(1) = q_goal(1) + 0.2;
	_nh_rrt.upper_limit_goal_bias(2) = q_goal(2) + 0.05;

	_nh_rrt.lower_limit_goal_bias(0) = q_goal(0) - 0.2;
	_nh_rrt.lower_limit_goal_bias(1) = q_goal(1) - 0.2;
	_nh_rrt.lower_limit_goal_bias(2) = q_goal(2) - 0.05;

	ofstream outFile("path_result.txt", ios::out); // open file for "writing"
	bool a = false;
	while (!a)
		a = _nh_rrt.StartNHRRT(outFile);

	outFile.close();
	MatrixXd joint_temp(200, 3);

//	ifstream inFile2("/home/kendrick/catkin_ws/src/mobile_manipulator_motion_planner/path_result_ex1.txt"); // "reading"
	ifstream inFile2("path_result.txt"); // open file for "writing"

	int size = 0;
	std::vector<std::string> parameters;
	char inputString[1000];
	while (!inFile2.eof()) { // eof : end of file
		inFile2.getline(inputString, 1000);
		boost::split(parameters, inputString, boost::is_any_of(","));
		if (parameters.size() == 3) {
			for (int j = 0; j < parameters.size(); j++)
				joint_temp(size, j) = atof(parameters[j].c_str()); // c_str() : string -> char* // atof : char* -> float
			size++;
		}

	}
	inFile2.close();
	joint_target = joint_temp.topLeftCorner(size, 3);
}
