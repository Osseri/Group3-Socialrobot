#ifndef RRTPLANNER_H_
#define RRTPLANNER_H_

#include "NodeTree.h"
#include <rbdl/rbdl.h>
#include <memory>
#include "util.h"


using namespace RigidBodyDynamics;
using namespace std;

struct Robotmodel {
	unsigned int body_id[dof];
	Math::Vector3d com_id[dof];
	shared_ptr<Model> model;
	VectorXd q;
	Matrix3d Rot;


};
struct ST_OBB
{
	Vector3d vPos;
	Vector3d vAxis[3];
	Vector3d fAxis; 
};

class rrt
{
public:
	virtual ~rrt() {};

	//RRT 
	int Connect(std::vector<double> &node);
	int Extend(std::vector<double> &node, RRTNode* &near);
	int ConstrainedConnect(std::vector<double> &node);
	std::vector<double> ConstrainedExtend(std::vector<double> &node, RRTNode* &near);
	std::vector<double> RandomConfig();

	bool StartCRRT(Robotmodel& model, std::ostream& sout);
	bool StartRRT(Robotmodel& model, std::ostream& sout);
	bool SmoothPath(std::ostream& sout, std::istream& sinput);
	bool CheckCollision(Robotmodel model, std::vector<double> &config);
	std::vector<double> ProjectConfig(Robotmodel model, std::vector<double> qold, std::vector<double> &qs);
	bool OutsideJointLimit(std::vector<double> q);
	bool ShortcutSmoothing();

	bool CheckTraj(std::vector<double> &a, std::vector<double> &b);
	double getDistance(std::vector<double> &a, std::vector<double> &b);
	std::vector<double> getUnitVector(std::vector<double> &a, std::vector<double> &b);

	MatrixXd MergeRRTResults(MatrixXd joint_target1, MatrixXd joint_target2, int case_);

	NodeTree *c_tree;
	NodeTree tree;
	NodeTree gTree;
	//RRTNode node;
	std::vector<double> start;
	std::vector<double> goal;
	std::vector<double> g2;

	VectorXd lower_limit, upper_limit, qinit, qgoal;
	bool left;
	std::vector<double> DOF_weights;
	std::vector<std::vector<double> > path;

	double step_size;
	int t_turn, count;
	bool isGoal;
	int DoF_size;
	MatrixXd C;

	Vector3d refer_pos;
	Matrix3d refer_rot;
	bool constraint_axis[3];

	Robotmodel _model;
	ST_OBB Box1[10], Box2[10];
	int box_num, box_num2;

	//Collision Check
	bool CheckOBBCollision(ST_OBB* Box1, ST_OBB* Box2);
	float FDotProduct(const Vector3d v0, const Vector3d v1)
	{
		return v0(0) * v1(0) + v0(1) * v1(1) + v0(2) * v1(2);
	}
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
	}
	static const double std_norm(const std::vector<double> a, const std::vector<double> b, int size) {
		double res = 0.0;
		for (int i = 0; i < size; i++)
			res += pow(a[i] - b[i], 2);

		res = sqrt(res);
		return res;
	}
		double vector_norm(const VectorXd a, const VectorXd b, int size) {
		double res = 0.0;
		for (int i = 0; i < size; i++)
			res += pow(a(i) - b(i), 2);

		res = sqrt(res);
		return res;
	}
};


#endif