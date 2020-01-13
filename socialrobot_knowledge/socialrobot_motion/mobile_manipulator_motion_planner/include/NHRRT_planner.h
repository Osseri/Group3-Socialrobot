#include <rbdl/rbdl.h>
#include <memory>
#include "util.h"


using namespace std;

struct Obs_para
{
	Vector2d pos;
	double radius;
};

struct vnew
{
	int id;
	Vector3d pose;
	std::vector<Vector3d> edgeq;
	int parent_id; // parent id
	double cost;

};

class NH_RRT
{
public:
	virtual ~NH_RRT() {};

	//NH-RRT 
	bool StartNHRRT(std::ostream& sout);

	vnew Extend_base(vnew & v_near, Vector3d &q_rand);
	double Compute_cost(std::vector<Vector3d> & path, Vector3d &q_0);
	Vector3d RandomConfig_base();

	bool CheckCollision_base(Obs_para* Obs, Vector3d q);
	int Check_edge(Obs_para* Obs, vnew vnew_i);

	std::vector<Vector3d> PositionCTRL_WP(Vector3d & q_start, Vector3d & q_end, int dir, double delta_T, double b);
	void PositionCTRL_Step_WP(double t, Vector3d q_current, Vector3d q_end, int dir, double b, double & vl, double & vr, int & eot, double &vm, double &vd);
	double norm_angle(double& angle, double min);
	VectorXd qinit, qgoal, lower_limit_base, upper_limit_base, lower_limit_goal_bias, upper_limit_goal_bias;

	Obs_para Obs[10];
	int cntId;
	int obs_num;
	int addedVert;
	double base_width, base_length;


};
