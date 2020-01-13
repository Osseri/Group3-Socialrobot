#include <iostream>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include "RRT_planner2.h"
#include <string>
#include <cstring>

using namespace std;
using namespace Eigen;
static const long MAX_TIME = 180.0f;
static const int DoF = 7;

#define REACHED 0
#define ADVANCED 1
#define TRAPPED -1
#define q_Max  180.0

bool rrt::StartRRT(Robotmodel& model, std::ostream& sout) {
	_model = model;

	tree._nodes.clear(); // NodeTree
	gTree._nodes.clear(); // NodeTree
	goal.clear(); // std::vector<double>
	start.clear();// std::vector<double>
	DOF_weights.clear();// std::vector<double>

	int ActiveDoFs = dof;
	step_size = 0.5;
	std::string start_string[DoF];

	for (int i = 0; i < ActiveDoFs; i++) {
		start.push_back(qinit(i) * 180.0 / M_PI);
		goal.push_back(qgoal(i) * 180.0 / M_PI);
		DOF_weights.push_back(1.0);
	}

	RRTNode n(start, 0);
	tree.addNode(n);
	c_tree = &tree; // c_tree : pointer of NodeTree
	t_turn = 0;
	g2 = goal;
	RRTNode g(goal, 0); 
	gTree.addNode(g);
	// That's why we called "BiRRT"

	bool finished = false;
	int iter = 1;

	while (iter < 500000) {
		count = 0;
		if (t_turn == 1) { // BiRRT 
			t_turn = 2;
			g2 = tree._nodes.back()->getConfiguration(); // back() : access last element of vector
			c_tree = &gTree;
		}
		else {
			t_turn = 1;
			g2 = gTree._nodes.back()->getConfiguration();
			c_tree = &tree;
		}
		std::vector<double> node = RandomConfig();

		if (this->Connect(node) == REACHED && isGoal) {
			std::cout << "FINISHED " << " " << "iteration:" << " " << iter << std::endl;
			finished = true;
			break;
		}
		iter++;
	}
	if (finished) {
		path = tree.getPath();
		vector<vector<double>> p2;
		p2 = gTree.getPath();
		reverse(p2.begin(), p2.end());
		path.insert(path.end(), p2.begin(), p2.end());

		for (int i = 0; i < path.size(); i++) {
			std::vector<double> node = path[i];
			for (int j = 0; j<node.size() - 1; j++)
				sout << node[j] << ",";

			sout << node[node.size() - 1] << "\n";
		}
	}
	else {
		return false;
	}

	return true;
}
bool rrt::StartCRRT(Robotmodel& model, std::ostream& sout) {
	_model = model;

	tree._nodes.clear();
	gTree._nodes.clear();
	goal.clear();
	start.clear();
	DOF_weights.clear();

	int ActiveDoFs = dof;
	step_size = 0.05/M_PI*180.0;
	std::string start_string[DoF];

	for (int i = 0; i < ActiveDoFs; i++) {
		start.push_back(qinit(i) * 180.0 / M_PI);
		goal.push_back(qgoal(i) * 180.0 / M_PI);
		DOF_weights.push_back(1.0);
	}


	RRTNode n(start, 0); // ������ RRT
	tree.addNode(n);
	c_tree = &tree;
	t_turn = 0;
	g2 = goal;
	RRTNode g(goal, 0); // ���� RRT
	gTree.addNode(g);

	bool finished = false;
	int iter = 1;
    int a;
    std::vector<double> robot;
    std::vector<double> q_rand;
    RRTNode* q_a_near;
    std::vector<double> q_a_reached ;
    RRTNode* q_b_near;
    std::vector<double> q_b_reached ;
	while (iter < 5000000)
	{

		g2 = gTree._nodes.back()->getConfiguration(); // goal tree
		c_tree = &tree;																//		// start tree
	    q_rand = RandomConfig();		// last node of goal tree(g2) or random config
	
	    q_a_near = c_tree->getNearest(robot, q_rand, DOF_weights); // random config�� ���� ����� c_tree�� ��� ã��
	    q_a_reached = this->ConstrainedExtend(q_rand, q_a_near); // near��� ���� random config�� Extend

    	c_tree = &gTree;
        q_b_near = c_tree->getNearest(robot, q_a_reached, DOF_weights); // random config�� ���� ����� c_tree�� ��� ã��

	    q_b_reached = this->ConstrainedExtend(q_a_reached, q_b_near); // near��� ���� random config�� Extend
        if (std_norm(q_a_reached, q_b_reached, dof) < step_size)
        {
            a = REACHED;
        }
        else
        {
            g2 = tree._nodes.back()->getConfiguration(); // goal tree
            c_tree = &gTree;                             //		// start tree
            q_rand = RandomConfig();
            q_a_near = c_tree->getNearest(robot, q_rand, DOF_weights); // random config�� ���� ����� c_tree�� ��� ã��

            q_a_reached = this->ConstrainedExtend(q_rand, q_a_near);   // near��� ���� random config�� Extend

            c_tree = &tree;
            q_b_near = c_tree->getNearest(robot, q_a_reached, DOF_weights); // random config�� ���� ����� c_tree�� ��� ã��

            q_b_reached = this->ConstrainedExtend(q_a_reached, q_b_near);   // near��� ���� random config�� Extend
            if (std_norm(q_a_reached, q_b_reached, dof) < step_size)
            {
                cout << std_norm(q_a_reached, q_b_reached, dof)  << endl;
                cout <<"reached" << endl;
                a = REACHED;
            }
        }

        if (a == REACHED)
		{
			std::cout << "FINISHED "
								<< " "
								<< "iteration:"
								<< " " << iter << std::endl;
			finished = true;
			break;
		}
		iter++;
	}
	if (finished) {
		path = tree.getPath();
		vector<vector<double>> p2;
		p2 = gTree.getPath();
		reverse(p2.begin(), p2.end());
		path.insert(path.end(), p2.begin(), p2.end());
		for (int i = 0; i < path.size(); i++) {
			std::vector<double> node = path[i];
			for (int j = 0; j<node.size() - 1; j++)
				sout << node[j] << ",";

			sout << node[node.size() - 1] << "\n";
		}
	}
	else {
		return false;
	}

	return true;
}
std::vector<double> rrt::RandomConfig() {
	double goalb = (double)rand() / RAND_MAX;
	if (goalb < 0.5) {
		isGoal = true;
		return g2;
	} 

	isGoal = false;
	std::vector<double> R;
	do {
		for (int i = 0; i < start.size(); i++) {
			double jointrange = upper_limit(i) - lower_limit(i);
			double r = ((double)rand() / (double)RAND_MAX)*jointrange;
			R.push_back(lower_limit(i) + r);
		}
	} while (R.size() != goal.size());

	return R;
}
int rrt::Connect(std::vector<double> &node) { // Connect current node & current tree

	std::vector<double> robot; // useless
	RRTNode* near = c_tree->getNearest(robot, node, DOF_weights);  // Get nearest node of NodeTree
	int s = this->Extend(node, near);
	while (s != TRAPPED) {
		if (s == REACHED) {
			return REACHED;
		}
		s = this->Extend(node, near);
	}
	return TRAPPED;
}
int rrt::Extend(std::vector<double> &node, RRTNode* &near) {
	std::vector<double> robot;
	double distance = sqrt(near->getDistance(robot, node, DOF_weights));
	std::vector<double> qnew;

	// 'near' is close to 'node'
	if (distance < step_size) {
	    if (!CheckCollision(_model, node)) { // False : Collision-free
			RRTNode *old = near;
			near = (new RRTNode(node, old)); // parent node�� near��, configuration�� node
			c_tree->addNode(*near); // Tree�� node �߰�
			return REACHED;
		}
		else {
			count++;
			return TRAPPED; // Collision !!
		}
	}

	for (int i = 0; i< goal.size(); i++) {
		qnew.push_back(near->getConfiguration()[i] + ((node[i] - near->getConfiguration()[i]) / distance)*step_size);
	}
	bool check = false; 
	check = CheckCollision(_model, qnew);
	if (!check) {
		RRTNode *old = near;

		near = (new RRTNode(qnew, old));
		c_tree->addNode(*near);
		return ADVANCED; // 'near' extend 'qnew'
	}
	else {

		return TRAPPED;
	}
}
// int rrt::ConstrainedConnect(std::vector<double> &node) {
// 	//&node : random configuration
// 	std::vector<double> robot;
// 	RRTNode* near = c_tree->getNearest(robot, node, DOF_weights); // random config�� ���� ����� c_tree�� ��� ã��
// 	std::vector<double> s = this->ConstrainedExtend(node, near); // near��� ���� random config�� Extend
//     cout << "extend " << endl;
//     cout << s << endl;
// 	return s;
// }
std::vector<double> rrt::ConstrainedExtend(std::vector<double> &node, RRTNode* &near) {
	// q_near & q_rand
	std::vector<double> robot;
	std::vector<double> qs, qs_old;
    qs.clear();
    qs_old.clear();
	bool project_flag = false;
	int iteration_ = 0 ;

	for (int i = 0; i < dof; i++) {
		qs.push_back(near->getConfiguration()[i]);
		qs_old.push_back(near->getConfiguration()[i]);
	}

	while (true) {
            
		if (std_norm(node, qs, dof) < step_size) {
			return qs;
		}
		else if (std_norm(node, qs, dof) - std_norm(qs_old, node, dof) > 0.0) {
			return qs_old;
		}
		if(iteration_ == 200){
			return qs_old;
		}

		for (int i = 0; i< goal.size(); i++)
			qs[i] = (qs[i] + min(step_size, std_norm(node, qs, dof)) * (node[i]-qs[i]) / std_norm(node, qs, dof));

		// qs_old : near / qs : extend
		qs = ProjectConfig(_model, qs_old, qs); // project qs onto constraint manifold
        
      //  cout << qs.empty() << endl;

        if (!qs.empty() && !CheckCollision(_model, qs))
        {
            // for (int i=0;i<dof;i++)
            // cout << qs[i] << "\t"

            // <<endl;
            RRTNode *old = near;           //qs_old
            near = (new RRTNode(qs, old)); //
            c_tree->addNode(*near);
            qs_old = qs;
			iteration_ ++;
            // cout << "project ok" << endl;
            // return qs_old;
        }
        else {
			// if (!CheckCollision(_model, qs)) {
			// 	RRTNode *old = near; //qs_old
			// 	near = (new RRTNode(qs, old)); // 
			// 	c_tree->addNode(*near);
        	// 	qs_old = qs;
            //   cout << "project ok" << endl;
			// }
			// else {
				return qs_old;
			//}
		}
	}
}
std::vector<double> rrt::ProjectConfig(Robotmodel model, std::vector<double> qold, std::vector<double> &qs) {
	bool flag = false;
	model.q.resize(dof);

	// Tc
	Matrix4d T0_c, T0_obj, Tc_obj;
//	T0_c.topLeftCorner(3,3) = refer_rot;
	T0_c.setIdentity();
	T0_c.topRightCorner(3, 1) = refer_pos;

	MatrixXd J_temp(6, dof), J(6, dof), eye(6, 6);
	eye.setIdentity();
	VectorXd d_c(6), dx(6), q_error(dof);
	dx.setZero();
	Vector3d phi;
	Vector3d s[3], v[3], w[3];
	Matrix3d Rotd;
	Rotd = refer_rot;

	for (int i = 0; i < dof; i++) {// deg -> rad
		qs[i] = qs[i] * M_PI / 180.0;
		qold[i] = qold[i] * M_PI / 180.0;
	}

	while (true) {
		for (int i = 0; i < dof; i++)
			model.q(i) = qs[i];

		Matrix3d Rot_temp = model.Rot*Rot_arm(model.q);
		Vector3d pos_temp = CalcBodyToBaseCoordinates(*model.model, model.q, model.body_id[dof-1], model.com_id[dof-1] ); // get position and rotation in EE frame;

		T0_obj.setIdentity();
		T0_obj.topLeftCorner(3, 3) = Rot_temp;
		T0_obj.topRightCorner(3, 1) = pos_temp;
		Tc_obj = T0_c.inverse() * T0_obj;


		d_c.head(3) = Tc_obj.topRightCorner(3, 1);
		d_c(3) = atan2(-Tc_obj(1, 2), Tc_obj(2, 2));
		d_c(4) = asin(Tc_obj(0, 2));
		d_c(5) = atan2(-Tc_obj(0, 1), Tc_obj(0, 0));

		for (int i = 0; i < 3; i++) {
			if (d_c(i) > C(i, 0)) // max
				dx(i) = d_c(i) - C(i, 0);
			else if (d_c(i) < C(i, 1)) // min
				dx(i) = d_c(i) - C(i, 1);
			else
				dx(i) = 0.0;
		}

		for (int i = 0; i < 3; i++)
			if (constraint_axis[i] == false)
				dx(i) = 0.0;

		for (int i = 0; i < 3; i++) {
			v[i] = Rot_temp.block(0, i, 3, 1);
			w[i] = Rotd.block(0, i, 3, 1);
			s[i] = v[i].cross(w[i]);
		}
		phi = s[0] + s[1] + s[2];
		phi = -0.5* phi;
		dx.tail(3) = 1.0*phi;


		// Algorithm 4 - line 3
		if (dx.norm() < 0.001) {
            for (int i = 0; i < dof; i++)
            {
                qs[i] = qs[i] / M_PI * 180.0;
            }
            //		cout << Tc_obj << endl;
		//	cout << "pos" << pos_temp.transpose() << endl;
		//	cout << "rot" << Rot_temp << endl;
			//		getchar();

			// flag = true;
			// break;
            return qs;
		}

		// Algorithm 4 - line 4
		if (left) {
			CalcPointJacobian6D(*model.model, model.q, model.body_id[dof-1], model.com_id[dof-1] , J_temp, true);
			J.topLeftCorner(3, dof) = J_temp.bottomLeftCorner(3, dof);
			J.bottomLeftCorner(3, dof) = J_temp.topLeftCorner(3, dof);
		}
		else {
			CalcPointJacobian6D(*model.model, model.q, model.body_id[dof-1], model.com_id[dof-1] , J_temp, true);
			J.topLeftCorner(3, dof) = J_temp.bottomLeftCorner(3, dof);
			J.bottomLeftCorner(3, dof) = J_temp.topLeftCorner(3, dof);
		}

		// Algorithm 4 - line 5
		q_error = J.transpose() * (J*J.transpose()).inverse() * dx;

		// Algorithm 4 - line 6
		for (int i = 0; i < dof; i++)
			qs[i] -= q_error(i);

		// Algorithm 4 - line 7 : stuck here
		if (!OutsideJointLimit(qs)) {
            qs.clear();
         //   cout << "empty" << endl;
			return qs;
            // flag = false;
			// break;
		}

	}

	// rad -> deg
	// for (int i = 0; i < dof; i++) {
	// 	qs[i] = qs[i] / M_PI * 180.0;
	// }

}
double rrt::getDistance(std::vector<double> &a, std::vector<double> &b) {

	//Gets squared Euclidean Distance on C-Space between node and a given
	double accumulated = 0.0f;
	std::vector<double> diff = b;

	for (int i = 0; i< diff.size(); i++) {
		diff[i] -= a[i];
		//double dif = _configuration[i] - (config)[i];
		accumulated += (diff[i] * diff[i] * DOF_weights[i] * DOF_weights[i]);
	}
	return sqrt(accumulated);
}
bool rrt::SmoothPath(std::ostream& sout, std::istream& sinput)
{
	std::string fullinput;
	//Parse input
	while (sinput) {
		std::string input;
		sinput >> input;
		fullinput += input;
	}
	int nSmooth = 200; // atoi(fullinput.c_str());

	while (nSmooth) { // while(x) -> while( x != 0 )
		ShortcutSmoothing();
		nSmooth--;
	}
	
	for (int i = 0; i< path.size(); i++) {
		std::vector<double> node = path[i];
		for (int j = 0; j<node.size() - 1; j++) {
			sout << node[j] << ",";
		}
		sout << node[node.size() - 1] << "\n";
	}

	return true;
}
bool rrt::ShortcutSmoothing() {
	if (path.size() <= 2) return false;
	int i = ((double)rand() / RAND_MAX)*path.size();
	int j = ((double)rand() / RAND_MAX)*path.size();
	while (abs(j - i)<2) {
		i = ((double)rand() / RAND_MAX)*path.size();
		j = ((double)rand() / RAND_MAX)*path.size();
	}
	if (j < i) { //make sure i is the smaller number
		int a = j;
		j = i;
		i = a;
	}

	if (CheckTraj(path[i], path[j])) {
		path.erase(path.begin() + i + 1, path.begin() + j);
		return true;
	}
	return false;
}

bool rrt::CheckTraj(std::vector<double> &a, std::vector<double> &b) {
	std::vector<double> u = getUnitVector(a, b); // a->b
	std::vector<double> p = a;
	while (p != b) {
		if (CheckCollision(_model, p)) { return false;
		break;
		}
		if (getDistance(p, b) < step_size / 2) {
			p = b;
		}
		else {
			for (int i = 0; i < u.size(); i++) {
				p[i] += u[i] * step_size / 2;
			}
		}
	}
	return true;
}
std::vector<double> rrt::getUnitVector(std::vector<double> &a, std::vector<double> &b) {

	//Gets squared Euclidean Distance on C-Space between node and a given
	double accumulated = 0.0f;
	std::vector<double> diff = b;

	for (int i = 0; i< diff.size(); i++) {
		diff[i] -= a[i];//double dif = _configuration[i] - (config)[i];
		accumulated += (diff[i] * diff[i] * DOF_weights[i] * DOF_weights[i]);
	}
	accumulated = sqrt(accumulated);
	for (int i = 0; i < diff.size(); i++) {
		diff[i] /= accumulated;
	}
	return diff;
}
bool rrt::CheckCollision(Robotmodel model, std::vector<double> &config) {
	model.q.resize(dof);
	for (int i = 0; i < dof; i++)
		model.q(i) = config[i] * M_PI / 180.0;

	if (!left) {
		Matrix3d Rot_temp = model.Rot*Rot_arm(model.q);// end-effector
		Box2[0].vAxis[0] = Rot_temp.col(0);
		Box2[0].vAxis[1] = Rot_temp.col(1);
		Box2[0].vAxis[2] = Rot_temp.col(2);
		Box2[0].fAxis = Vector3d(0.03, 0.07, 0.1);
		Box2[0].vPos = CalcBodyToBaseCoordinates(*model.model, model.q, model.body_id[dof-1], model.com_id[dof-1] , true);

		Matrix3d Rot_temp2 = model.Rot*Rot_arm_link5(model.q); // link 5
		Box2[1].vAxis[0] = Rot_temp2.col(0);
		Box2[1].vAxis[1] = Rot_temp2.col(1);
		Box2[1].vAxis[2] = Rot_temp2.col(2);
		Box2[1].fAxis = Vector3d(0.05, 0.05, 0.10);
		Box2[1].vPos = CalcBodyToBaseCoordinates(*model.model, model.q, model.body_id[4], model.com_id[4], true);

		Matrix3d Rot_temp3 = model.Rot*Rot_arm_link2(model.q); // link 2
		Box2[2].vAxis[0] = Rot_temp3.col(0);
		Box2[2].vAxis[1] = Rot_temp3.col(1);
		Box2[2].vAxis[2] = Rot_temp3.col(2);
		Box2[2].fAxis = Vector3d(0.05, 0.05, 0.10);
		Box2[2].vPos = CalcBodyToBaseCoordinates(*model.model, model.q, model.body_id[1], model.com_id[1], true);

		// Box2[3].vAxis[0] = Rot_temp.col(0);
		// Box2[3].vAxis[1] = Rot_temp.col(1);
		// Box2[3].vAxis[2] = Rot_temp.col(2);
		// Box2[3].fAxis = Vector3d(0.03, 0.03, 0.05);
		// Box2[3].vPos = CalcBodyToBaseCoordinates(*model.model, model.q, model.body_id[6], model.com_id[dof], true);


	}
	else {
			Matrix3d Rot_temp = model.Rot*Rot_arm(model.q);// end-effector
		Box2[0].vAxis[0] = Rot_temp.col(0);
		Box2[0].vAxis[1] = Rot_temp.col(1);
		Box2[0].vAxis[2] = Rot_temp.col(2);
		Box2[0].fAxis = Vector3d(0.03, 0.07, 0.1);
		Box2[0].vPos = CalcBodyToBaseCoordinates(*model.model, model.q, model.body_id[dof-1], model.com_id[dof-1] , true);

		Matrix3d Rot_temp2 = model.Rot*Rot_arm_link5(model.q); // link 5
		Box2[1].vAxis[0] = Rot_temp2.col(0);
		Box2[1].vAxis[1] = Rot_temp2.col(1);
		Box2[1].vAxis[2] = Rot_temp2.col(2);
		Box2[1].fAxis = Vector3d(0.05, 0.05, 0.10);
		Box2[1].vPos = CalcBodyToBaseCoordinates(*model.model, model.q, model.body_id[4], model.com_id[4], true);

		Matrix3d Rot_temp3 = model.Rot*Rot_arm_link2(model.q); // link 2
		Box2[2].vAxis[0] = Rot_temp3.col(0);
		Box2[2].vAxis[1] = Rot_temp3.col(1);
		Box2[2].vAxis[2] = Rot_temp3.col(2);
		Box2[2].fAxis = Vector3d(0.05, 0.05, 0.10);
		Box2[2].vPos = CalcBodyToBaseCoordinates(*model.model, model.q, model.body_id[1], model.com_id[1], true);

		Box2[3].vAxis[0] = Rot_temp.col(0);
		Box2[3].vAxis[1] = Rot_temp.col(1);
		Box2[3].vAxis[2] = Rot_temp.col(2);
		Box2[3].fAxis = Vector3d(0.03, 0.03, 0.05);
		Box2[3].vPos = CalcBodyToBaseCoordinates(*model.model, model.q, model.body_id[6], model.com_id[dof], true);


	}
	bool chk;
	if (box_num == 0) {
		chk = false;
		return chk;
	}
	for (int i = 0; i < box_num; i++) {
		for (int j = 0;j < box_num2; j++) {
		chk = CheckOBBCollision(&Box1[i], &Box2[j]);
		if (chk) {
			return chk;
		}
		//cout << "collision" << i << "\t" << j << endl;
	}
	}
	return chk;
}
bool rrt::OutsideJointLimit(std::vector<double> q) { // radian
	for (int i = 0;i < q.size();i++) {
		if (q[i] > upper_limit(i)/180.0*M_PI) {
		//	cout << q[i] << "\t" << upper_limit(i)/180.0*M_PI << "\t" << i << endl;
			return false;
		}
		else if (q[i] < lower_limit(i)/180.0*M_PI) {
		//	cout << q[i] << "\t" << lower_limit(i)/180.0*M_PI  << "\t" << i << endl;

			return false;
		}
	}
	return true;

}
bool rrt::CheckOBBCollision(ST_OBB* box0, ST_OBB* box1) // Collision-free : False
{
	// compute difference of box centers,D=C1-C0
	Vector3d D = Vector3d(box1->vPos(0) - box0->vPos(0), box1->vPos(1) - box0->vPos(1), box1->vPos(2) - box0->vPos(2));

	float C[3][3];    //matrix C=A^T B,c_{ij}=Dot(A_i,B_j)
	float absC[3][3]; //|c_{ij}|
	float AD[3];      //Dot(A_i,D)
	float R0, R1, R;    //interval radii and distance between centers
	float R01;        //=R0+R1

					  //A0
	C[0][0] = FDotProduct(box0->vAxis[0], box1->vAxis[0]);// vAxis : direction // 3D Dot product
	C[0][1] = FDotProduct(box0->vAxis[0], box1->vAxis[1]);
	C[0][2] = FDotProduct(box0->vAxis[0], box1->vAxis[2]);
	AD[0] = FDotProduct(box0->vAxis[0], D);
	absC[0][0] = (float)fabsf(C[0][0]);
	absC[0][1] = (float)fabsf(C[0][1]);
	absC[0][2] = (float)fabsf(C[0][2]);
	R = (float)fabsf(AD[0]);
	R1 = box1->fAxis(0) * absC[0][0] + box1->fAxis(1) * absC[0][1] + box1->fAxis(2) * absC[0][2];
	R01 = box0->fAxis(0) + R1;
	if (R > R01)
		return 0;

	//A1
	C[1][0] = FDotProduct(box0->vAxis[1], box1->vAxis[0]);
	C[1][1] = FDotProduct(box0->vAxis[1], box1->vAxis[1]);
	C[1][2] = FDotProduct(box0->vAxis[1], box1->vAxis[2]);
	AD[1] = FDotProduct(box0->vAxis[1], D);
	absC[1][0] = (float)fabsf(C[1][0]);
	absC[1][1] = (float)fabsf(C[1][1]);
	absC[1][2] = (float)fabsf(C[1][2]);
	R = (float)fabsf(AD[1]);
	R1 = box1->fAxis(0) * absC[1][0] + box1->fAxis(1) * absC[1][1] + box1->fAxis(2) * absC[1][2];
	R01 = box0->fAxis(1) + R1;
	if (R > R01)
		return 0;

	//A2
	C[2][0] = FDotProduct(box0->vAxis[2], box1->vAxis[0]);
	C[2][1] = FDotProduct(box0->vAxis[2], box1->vAxis[1]);
	C[2][2] = FDotProduct(box0->vAxis[2], box1->vAxis[2]);
	AD[2] = FDotProduct(box0->vAxis[2], D);
	absC[2][0] = (float)fabsf(C[2][0]);
	absC[2][1] = (float)fabsf(C[2][1]);
	absC[2][2] = (float)fabsf(C[2][2]);
	R = (float)fabsf(AD[2]);
	R1 = box1->fAxis(0) * absC[2][0] + box1->fAxis(1) * absC[2][1] + box1->fAxis(2) * absC[2][2];
	R01 = box0->fAxis(2) + R1;
	if (R > R01)
		return 0;

	//B0
	R = (float)fabsf(FDotProduct(box1->vAxis[0], D));
	R0 = box0->fAxis(0) * absC[0][0] + box0->fAxis(1) * absC[1][0] + box0->fAxis(2) * absC[2][0];
	R01 = R0 + box1->fAxis(0);
	if (R > R01)
		return 0;

	//B1
	R = (float)fabsf(FDotProduct(box1->vAxis[1], D));
	R0 = box0->fAxis(0) * absC[0][1] + box0->fAxis(1) * absC[1][1] + box0->fAxis(2) * absC[2][1];
	R01 = R0 + box1->fAxis(1);
	if (R > R01)
		return 0;

	//B2
	R = (float)fabsf(FDotProduct(box1->vAxis[2], D));
	R0 = box0->fAxis(0) * absC[0][2] + box0->fAxis(1) * absC[1][2] + box0->fAxis(2) * absC[2][2];
	R01 = R0 + box1->fAxis(2);
	if (R > R01)
		return 0;

	//A0xB0
	R = (float)fabsf(AD[2] * C[1][0] - AD[1] * C[2][0]);
	R0 = box0->fAxis(1) * absC[2][0] + box0->fAxis(2) * absC[1][0];
	R1 = box1->fAxis(1) * absC[0][2] + box1->fAxis(2) * absC[0][1];
	R01 = R0 + R1;
	if (R > R01)
		return 0;

	//A0xB1
	R = (float)fabsf(AD[2] * C[1][1] - AD[1] * C[2][1]);
	R0 = box0->fAxis(1) * absC[2][1] + box0->fAxis(2) * absC[1][1];
	R1 = box1->fAxis(0) * absC[0][2] + box1->fAxis(2) * absC[0][0];
	R01 = R0 + R1;
	if (R > R01)
		return 0;

	//A0xB2
	R = (float)fabsf(AD[2] * C[1][2] - AD[1] * C[2][2]);
	R0 = box0->fAxis(1) * absC[2][2] + box0->fAxis(2) * absC[1][2];
	R1 = box1->fAxis(0) * absC[0][1] + box1->fAxis(1) * absC[0][0];
	R01 = R0 + R1;
	if (R > R01)
		return 0;

	//A1xB0
	R = (float)fabsf(AD[0] * C[2][0] - AD[2] * C[0][0]);
	R0 = box0->fAxis(0) * absC[2][0] + box0->fAxis(2) * absC[0][0];
	R1 = box1->fAxis(1) * absC[1][2] + box1->fAxis(2) * absC[1][1];
	R01 = R0 + R1;
	if (R > R01)
		return 0;

	//A1xB1
	R = (float)fabsf(AD[0] * C[2][1] - AD[2] * C[0][1]);
	R0 = box0->fAxis(0) * absC[2][1] + box0->fAxis(2) * absC[0][1];
	R1 = box1->fAxis(0) * absC[1][2] + box1->fAxis(2) * absC[1][0];
	R01 = R0 + R1;
	if (R > R01)
		return 0;

	//A1xB2
	R = (float)fabsf(AD[0] * C[2][2] - AD[2] * C[0][2]);
	R0 = box0->fAxis(0) * absC[2][2] + box0->fAxis(2) * absC[0][2];
	R1 = box1->fAxis(0) * absC[1][1] + box1->fAxis(1) * absC[1][0];
	R01 = R0 + R1;
	if (R > R01)
		return 0;

	//A2xB0
	R = (float)fabsf(AD[1] * C[0][0] - AD[0] * C[1][0]);
	R0 = box0->fAxis(0) * absC[1][0] + box0->fAxis(1) * absC[0][0];
	R1 = box1->fAxis(1) * absC[2][2] + box1->fAxis(2) * absC[2][1];
	R01 = R0 + R1;
	if (R > R01)
		return 0;

	//A2xB1
	R = (float)fabsf(AD[1] * C[0][1] - AD[0] * C[1][1]);
	R0 = box0->fAxis(0) * absC[1][1] + box0->fAxis(1) * absC[0][1];
	R1 = box1->fAxis(0) * absC[2][2] + box1->fAxis(2) * absC[2][0];
	R01 = R0 + R1;
	if (R > R01)
		return 0;

	//A2xB2
	R = (float)fabsf(AD[1] * C[0][2] - AD[0] * C[1][2]);
	R0 = box0->fAxis(0) * absC[1][2] + box0->fAxis(1) * absC[0][2];
	R1 = box1->fAxis(0) * absC[2][1] + box1->fAxis(1) * absC[2][0];
	R01 = R0 + R1;
	if (R > R01)
		return 0;

	return 1;
}
MatrixXd rrt::MergeRRTResults(MatrixXd joint_target1, MatrixXd joint_target2, int case_){
	// case 0 : RRT / RRT
	// case 1 : RRT / CBiRRT
	// case 2 : CBiRRT / RRT
	// case 3 : CBiRRT / CBiRRT 
	MatrixXd merged_joint_target;
	int row[2];
	int reminder[2];
	switch(case_ )
	{
		case 0 :
			row[0] = joint_target1.rows();
			row[1] = joint_target2.rows();
			merged_joint_target.resize(row[0] + row[1] -1, dof);
			merged_joint_target.topRows(row[0]) = joint_target1.topRows(row[0]);
			merged_joint_target.bottomRows(row[1]-1) = joint_target2.bottomRows(row[1]-1);
		break;
		case 1 :
			row[0] = joint_target1.rows();
			row[1] = joint_target2.rows() / 20;
			reminder[1] = joint_target2.rows() % 20;
			if (reminder[1] == 0)
			{
				merged_joint_target.resize(row[0] + row[1], dof);
				merged_joint_target.topRows(row[0]) = joint_target1.topRows(row[0]);
				for (int i = 0; i < row[1]; i++)
					merged_joint_target.row(row[0] + i) = joint_target2.row(20 * (i + 1));
			}
			else{
				merged_joint_target.resize(row[0] + row[1] + 1,dof);
				merged_joint_target.topRows(row[0]) = joint_target1.topRows(row[0]);
				for (int i = 0; i < row[1]; i++)
					merged_joint_target.row(row[0] + i) = joint_target2.row(20 * (i + 1));

				merged_joint_target.bottomRows(1) = joint_target2.bottomRows(1);
			}
			break; 
		case 2 :
			row[0] = joint_target1.rows() /20;
			row[1] = joint_target2.rows() ;
			reminder[0] = joint_target1.rows() % 20;
			if (reminder[0] == 0)
			{
				merged_joint_target.resize(row[0] + row[1],dof);
				for (int i = 0; i < row[0]; i++)
					merged_joint_target.row(i) = joint_target1.row(20 * (i));

				merged_joint_target.bottomRows(row[1]) = joint_target2.bottomRows(row[1]);
			}
			else{
				merged_joint_target.resize(row[0] + row[1] + 1,dof);
				for (int i = 0; i < row[0] + 1; i++)
					merged_joint_target.row(i) = joint_target1.row(20 * (i));

				merged_joint_target.bottomRows(row[1]) = joint_target2.bottomRows(row[1]);
			}
		break;
		case 3 :
			row[0] = joint_target1.rows() /20;
			row[1] = joint_target2.rows() /20;
			reminder[0] = joint_target1.rows() % 20;
			reminder[1] = joint_target2.rows() % 20;
			if (reminder[0] == 0){
				if (reminder[1] == 0){
					merged_joint_target.resize(row[0] + row[1] + 1,dof);
					for (int i = 0; i < row[0] + 1; i++)
						merged_joint_target.row(i) = joint_target1.row(20 * (i));

					for (int i = 0; i < row[1]; i++)
						merged_joint_target.row(row[0] + 1 + i) = joint_target2.row(20 * (i+1));
				}
				else{
					merged_joint_target.resize(row[0] + row[1] + 2,dof);
					for (int i = 0; i < row[0] + 1; i++)
						merged_joint_target.row(i) = joint_target1.row(20 * (i));

					for (int i = 0; i < row[1]; i++)
						merged_joint_target.row(row[0] + 1 + i) = joint_target2.row(20 * (i+1));

					merged_joint_target.bottomRows(1) = joint_target2.bottomRows(1);
				}
			}
			else{
				if (reminder[1] == 0){
					merged_joint_target.resize(row[0] + row[1] + 2,dof);
					for (int i = 0; i < row[0] + 1; i++)
						merged_joint_target.row(i) = joint_target1.row(20 * (i));

					merged_joint_target.row(row[0]+1) = joint_target1.bottomRows(1);

					for (int i = 0; i < row[1]; i++)
						merged_joint_target.row(row[0] + 2 + i) = joint_target2.row(20 * (i+1));
				}
				else{
					merged_joint_target.resize(row[0] + row[1] + 3,dof);
					for (int i = 0; i < row[0] + 1; i++)
						merged_joint_target.row(i) = joint_target1.row(20 * (i));

					merged_joint_target.row(row[0]+1) = joint_target1.bottomRows(1);

					for (int i = 0; i < row[1]; i++)
						merged_joint_target.row(row[0] + 2 + i) = joint_target2.row(20 * (i+1));

					merged_joint_target.bottomRows(1) = joint_target2.bottomRows(1);
				}
			}
		break;
	}


	return merged_joint_target;
}