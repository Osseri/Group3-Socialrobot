#pragma once
#include "fwd.h"
#include <rbdl/rbdl.h>

Matrix3d Rotate_with_X(const double rAngle);
Matrix3d Rotate_with_Y(const double rAngle);
Matrix3d Rotate_with_Z(const double rAngle);

Matrix3d Rot_arm(VectorXd q_current_);
Matrix3d Rot_arm_link5(VectorXd q_current_);
Matrix3d Rot_arm_link2(VectorXd q_current_);


Vector3d GetPhi(Matrix3d Rot, Matrix3d Rotd);
double Cubic(double rT, double rT_0, double rT_f, double rx_0, double rx_dot_0, double rx_f, double rx_dot_f);
double JCubic(double rT, double rT_0, double rT_f, double rx_0, double rx_dot_0, double& rx_f, double rx_dot_f);