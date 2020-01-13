#include <ros_control_boilerplate/generic_hw_control_loop.h>
//#include <socialrobot_hardware/skkuhand_hw_interface.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "behavior");
    ros::NodeHandle nh;

    // NOTE: We run the ROS loop in a separate thread as external calls such
    // as service callbacks to load controllers can block the (main) control loop
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Create the hardware interface specific to your robot
    boost::shared_ptr<socialrobot_hw::SKKUHandHWInterface> skkuhand_hw_interface(new socialrobot_hw::SKKUHandHWInterface(nh));
    skkuhand_hw_interface->init();

    // Start the control loop
    ros_control_boilerplate::GenericHWControlLoop control_loop(nh, skkuhand_hw_interface);
    //control_loop.run(); // Blocks until shutdown signal recieved

    ros::waitForShutdown();

    return 0;
}