#include <iostream>
#include <string>
#include "ROS_bridge.hpp"
#include "Controller.h"

#include <sensor_msgs/JointState.h>

// for vrep keyboard event
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

using namespace std;

bool _kbhit()
{
    termios term;
    tcgetattr(0, &term);

    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &term);

    return byteswaiting > 0;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mobile manipulator mobile planner");
	ros::NodeHandle nh("~");
	
	const double hz = 100;
	ROS_bridge rb(nh, hz);
	
	Controller ac(nh);
	nh.param("urdf_param", ac.urdf_param, std::string("/robot_description"));

	ac.initModel();
	ac.initVariable();
	
	sleep(1);
	rb.vrepStart();
	sleep(1);
	rb.vrepEnableSyncMode();
	sleep(1);

	bool isSimulationRun = false;
	bool exitFlag = false;	

	while (ros::ok() && !exitFlag)
	{
		rb.read_vrep();
		ac.readdata(rb.current_ql_, rb.current_qr_, rb.current_ql_dot_, rb.current_qr_dot_, rb.current_base_);
		
		if (_kbhit())
		{
			int key = getchar();
			switch (key)
			{
			case 'i':
				cout << "Initial Posture (Joint Space)" << endl;
				ac.setMode(Controller::INIT);
				break;
			/* Non-holonomic RRT */
			case 'm':
				cout << "Move mobile robot" << endl;
				ac.setMode(Controller::MOVE_MOBILE1);
				break;
			case 'n':
				cout << "Move mobile robot" << endl;
				ac.setMode(Controller::MOVE_MOBILE2);
				break;
			/* BiRRT */
			case 'e':
				cout << "Reaching cup" << endl;
				ac.setMode(Controller::REACH_CUP);
				break;
			case 'r':
				cout << "Moving Cup" << endl;
				ac.setMode(Controller::MOVE_CUP);
				break;
			case 's':
				cout << "Reaching Milk" << endl;
				ac.setMode(Controller::REACH_MILK);
				break;
			case 'd':
				cout << "Moving Milk" << endl;
				ac.setMode(Controller::MOVE_MILK);
				break;
			case 'g':
				cout << "Grasping Milk" << endl;
				rb.desired_grasping_l = 0.02;
				break;
			case '\t':
				if (isSimulationRun) {
					cout << "Simulation Pause" << endl;
					isSimulationRun = false;
				}
				else {
					cout << "Simulation Run" << endl;
					isSimulationRun = true;
					ac.setMode(Controller::DEFAULT);
				}
				break;
			case 'q':
				isSimulationRun = false;
				exitFlag = true;
				break;
			
			}
		}
		
		if (isSimulationRun) {
			ac.compute();
			ac.writedata(rb.desired_ql_, rb.desired_qr_, rb.desired_base_vel_);
			rb.write_vrep();
			rb.wait();
			
		}
	
	}
	
	return 0;
}