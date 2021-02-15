sequence 

1. Turn on camera on robocare and external camera.
external : roslaunch vision_calibrator external_realsense.launch 
robocare : roslaunch realsens2_camera rs_camera.launch 
(For the test, Azure Kinect was used.)
(roslaunch azure_kinect_ros_driver driver.launch)

2. Launch the module from KIST
roslaunch darknet_ros yolo_v3_store_light_inv.launch 

3. Launch the calibratior nodes 
roslaunch vision_calibrator calibrator_external.launch
roslaunch vision_calibrator calibrator_robocare.launch 

Both nodes are uses same python file(external_calibrator.py).
I tried to merge these 2 launch file, but it works abnormal. 
(I'm guessing because these are using same python file for different node.)
If you know the merge these two files, then please modify.

These nodes runs with the start topic "/calibration/start".
I simply used this command to run nodes at once.
rostopic pub -1 /calibration/start std_msgs/Bool 'True'

To respond to change of cameras, I used rosparam in launch file.
Please look line 45-51 in external_calibrator.py.


4. Launch the object tracker 
rosrun multi_object_tracker object_tracking.py

It first waits the topics include the transformation matrix from both camera.
The name of topics are "/cam_e/calibration/result",  "/camera/calibration/result",
and these comes from the nodes described in number 3 in the above. 

After receves these topics, it publishes 3d bounding box of the deteccted features.