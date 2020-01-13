# socialrobot_actionlib

## Dependencies

- ROS Kinetic/Melodic
- [rdflib](https://github.com/RDFLib/rdflib)

## Install

1. Install the ROS. [Instructions for Ubuntu 16.04](http://wiki.ros.org/indigo/Installation/Ubuntu)
   
2. [Setup your ROS environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
   
3. Install [rdflib](https://github.com/RDFLib/rdflib) python library
    ```
    sudo pip install rdflib
    ```
4. make and launch 
   ```
   catkin_make
   source devel/setup.bash
   roslaunch socialrobot_actionlib actionlib.launch
