#pragma once
#include "util.h"

#include <iostream>
using namespace Eigen;
using namespace std;

Matrix3d Rotate_with_X(const double rAngle)
{
	Matrix3d _Rotate_wth_X;

	_Rotate_wth_X(0, 0) = 1.0;
	_Rotate_wth_X(1, 0) = 0.0;
	_Rotate_wth_X(2, 0) = 0.0;

	_Rotate_wth_X(0, 1) = 0.0;
	_Rotate_wth_X(1, 1) = cos(rAngle);
	_Rotate_wth_X(2, 1) = sin(rAngle);

	_Rotate_wth_X(0, 2) = 0.0;
	_Rotate_wth_X(1, 2) = -sin(rAngle);
	_Rotate_wth_X(2, 2) = cos(rAngle);

	return(_Rotate_wth_X);
}

Matrix3d Rotate_with_Y(const double rAngle)
{
	Matrix3d _Rotate_wth_Y(3, 3);

	_Rotate_wth_Y(0, 0) = cos(rAngle);
	_Rotate_wth_Y(1, 0) = 0.0;
	_Rotate_wth_Y(2, 0) = -sin(rAngle);

	_Rotate_wth_Y(0, 1) = 0.0;
	_Rotate_wth_Y(1, 1) = 1.0;
	_Rotate_wth_Y(2, 1) = 0.0;

	_Rotate_wth_Y(0, 2) = sin(rAngle);
	_Rotate_wth_Y(1, 2) = 0.0;
	_Rotate_wth_Y(2, 2) = cos(rAngle);

	return(_Rotate_wth_Y);
}

Matrix3d Rotate_with_Z(const double rAngle)
{
	Matrix3d _Rotate_wth_Z(3, 3);

	_Rotate_wth_Z(0, 0) = cos(rAngle);
	_Rotate_wth_Z(1, 0) = sin(rAngle);
	_Rotate_wth_Z(2, 0) = 0.0;

	_Rotate_wth_Z(0, 1) = -sin(rAngle);
	_Rotate_wth_Z(1, 1) = cos(rAngle);
	_Rotate_wth_Z(2, 1) = 0.0;

	_Rotate_wth_Z(0, 2) = 0.0;
	_Rotate_wth_Z(1, 2) = 0.0;
	_Rotate_wth_Z(2, 2) = 1.0;

	return(_Rotate_wth_Z);
}

Matrix3d Rot_arm(VectorXd q_current_) {
	Matrix3d Rot_;
	Rot_ = Rotate_with_Z(q_current_(0))*Rotate_with_Y(q_current_(1))*Rotate_with_Z(q_current_(2))*Rotate_with_Y(-q_current_(3))*Rotate_with_Z(q_current_(4))*Rotate_with_Y(-q_current_(5))*Rotate_with_Z(-q_current_(6));
	return Rot_;
}
Matrix3d Rot_arm_link5(VectorXd q_current_) {
	Matrix3d Rot_;
	Rot_ = Rotate_with_Z(q_current_(0))*Rotate_with_Y(q_current_(1))*Rotate_with_Z(q_current_(2))*Rotate_with_Y(-q_current_(3))*Rotate_with_Z(q_current_(4));
	return Rot_;
}
Matrix3d Rot_arm_link2(VectorXd q_current_) {
	Matrix3d Rot_;
	Rot_ = Rotate_with_Z(q_current_(0))*Rotate_with_Y(q_current_(1));
	return Rot_;
}

Vector3d GetPhi(Matrix3d Rot, Matrix3d Rotd) {


	Vector3d phi;
	Vector3d s[3], v[3], w[3];

	for (int i = 0; i < 3; i++) {
		v[i] = Rot.block(0, i, 3, 1);
		w[i] = Rotd.block(0, i, 3, 1);
		s[i] = v[i].cross(w[i]);
	}
	phi = s[0] + s[1] + s[2];
	phi = -0.5* phi;

	return phi;
}

double Cubic(double rT, double rT_0, double rT_f, double rx_0, double rx_dot_0, double rx_f, double rx_dot_f)
{

	double rx_t;
	if (rT<rT_0)
	{
		rx_t = rx_0;
	}
	else if (rT >= rT_0 && rT<rT_f)
	{
		rx_t = rx_0 + rx_dot_0 * (rT - rT_0)
			+ (3 * (rx_f - rx_0) / ((rT_f - rT_0) * (rT_f - rT_0)) - 2 * rx_dot_0 / ((rT_f - rT_0) * (rT_f - rT_0)) - rx_dot_f / ((rT_f - rT_0) * (rT_f - rT_0)))*(rT - rT_0)*(rT - rT_0)
			+ (-2 * (rx_f - rx_0) / ((rT_f - rT_0) * (rT_f - rT_0) * (rT_f - rT_0)) + (rx_dot_0 + rx_dot_f) / ((rT_f - rT_0) * (rT_f - rT_0) * (rT_f - rT_0)))*(rT - rT_0)*(rT - rT_0)*(rT - rT_0);
	}
	else
	{
		rx_t = rx_f;
	}
	return (rx_t);
}

double JCubic(double rT, double rT_0, double rT_f, double rx_0, double rx_dot_0, double& rx_f, double rx_dot_f)
{


	double rx_t;
	if (rT<rT_0)
	{
		rx_t = rx_0;
	}
	else if (rT >= rT_0 && rT<rT_f)
	{
		//if (abs(rx_0 - rx_f) > abs(rx_0 - (rx_f + 2 * M_PI)))
		//	rx_f += 2 * M_PI;
		rx_t = rx_0 + rx_dot_0 * (rT - rT_0)
			+ (3 * (rx_f - rx_0) / ((rT_f - rT_0) * (rT_f - rT_0)) - 2 * rx_dot_0 / ((rT_f - rT_0) * (rT_f - rT_0)) - rx_dot_f / ((rT_f - rT_0) * (rT_f - rT_0)))*(rT - rT_0)*(rT - rT_0)
			+ (-2 * (rx_f - rx_0) / ((rT_f - rT_0) * (rT_f - rT_0) * (rT_f - rT_0)) + (rx_dot_0 + rx_dot_f) / ((rT_f - rT_0) * (rT_f - rT_0) * (rT_f - rT_0)))*(rT - rT_0)*(rT - rT_0)*(rT - rT_0);
	}
	else
	{
		rx_t = rx_f;
	}
	return (rx_t);
}
