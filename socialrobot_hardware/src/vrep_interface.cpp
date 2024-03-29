#include "interface_controller.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveit_vrep_interface");
  ros::NodeHandle n;

  // action server for moveit execution button
  ArmController left_arm("left_arm_controller/follow_joint_trajectory",n);
  ArmController right_arm("right_arm_controller/follow_joint_trajectory",n);
  ArmController dual_arm("dual_arm_controller/follow_joint_trajectory",n);

  // action server for socialrobot system
  ArmController arm_controller("arm_controller/follow_joint_trajectory",n);

  // gripper server 
  GripperController gripper_controller("gripper_controller/follow_joint_trajectory",n);

  ros::Rate rate(5000);

  while(ros::ok()){
    ros::spinOnce();
    left_arm.compute();
    right_arm.compute();
    dual_arm.compute();
    arm_controller.compute();

    gripper_controller.compute();
    rate.sleep();
  }


  return 0;
}
