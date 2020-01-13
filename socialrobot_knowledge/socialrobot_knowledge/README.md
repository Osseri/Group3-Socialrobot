# socialrobot_knowledge
The meta-package for Socialrobot project.

- context_manager
- rosjava_custom_srv

## Dependencies

- ROS Kinetic/Melodic
- rosjava
- SWI-prolog


## Install

1. Install the ROS. [Instructions for Ubuntu 16.04](http://wiki.ros.org/indigo/Installation/Ubuntu)
   
2. [Setup your ROS environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
   

3. Install rosjava packages and dependencies
   ```
   sudo apt install openjdk-8-jdk openjfx
   sudo apt-get install ros-kinetic-rosjava
   ```
4. Install SWI-prolog
   
   Add the ppa ppa:swi-prolog/stable to your system’s software sources:
   ```
   sudo add-apt-repository ppa:swi-prolog/stable
   sudo apt-get update
   sudo apt-get install swi-prolog swi-prolog-java
   ```



5. Unzip 20190827_cmprolog.zip to specific folder
   

## Setup
1. export environment variables to `~/.bashrc`
   ```
   export SWI_HOME_DIR="/usr/lib/swi-prolog"
   export LD_PRELOAD="$LD_PRELOAD:$SWI_HOME_DIR/lib/x86_64-linux/libswipl.so"
   export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$SWI_HOME_DIR/bin/x86_64-linux/:$SWI_HOME_DIR/lib/x86_64-linux/"

   ```
2. Modify the cmProlog environment.
   
   Open `cmProlog/prolog/init.pl` file and modify line2 and line3 path to your cmProlog path

3. Modify the contextmanager environment.
   
   Open `context_manager/src/main/java/org/ros/rosjava_context_manager/ContextManager.java` file and modify line 346 and line 363 to your cmProlog path
   
4. Re-build
   ```
   catkin_make
   ```


## Issues
1. build error about `org.ros.rosjava_messages`: generate rosjava messages.

```
genjava_message_artifacts --verbose -p std_msgs geometry_msgs octomap_msgs moveit_msgs vision_msgs rosjava_custom_srv
```

   