#include "ros/ros.h"
#include "movearm.h"
#include "gripper_openclose.h"
//#include "grasp.h"

using namespace socialrobot_motion;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motionplan_server");
  ros::NodeHandle n;

  //MoveArm moveArm;
  //ros::ServiceServer movearm_srv = n.advertiseService("/motion_plan/move_arm", &MoveArm::plan_motion, (MotionBase*) &moveArm);

  GripperOpenClose gripper;
  ros::ServiceServer gripper_srv = n.advertiseService("/motion_plan/gripper_open_close", &GripperOpenClose::plan_motion, (MotionBase*) &gripper);

  //GraspObject graspObject;
  //ros::ServiceServer grasp_srv = n.advertiseService("/motion_plan/grasp_obejct", &GraspObject::plan_motion, (MotionBase*) &GraspObject);
  
  
  //ros::spin();
  ros::AsyncSpinner spinner(4); // Use 4 threads, this can be modified.
  spinner.start();
  ros::waitForShutdown();

  return 0;
}