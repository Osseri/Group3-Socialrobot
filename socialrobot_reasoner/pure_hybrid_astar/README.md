# Pusher Node

The Hybrid-A* source from [here](https://github.com/ryul1206/2d-push-planner-py).

```sh
roslaunch social_robot_bringup base.launch
roslaunch social_robot_bringup arm.launch
	
roslaunch socialrobot_interface init.launch
roslaunch robocare_navigation social_robot_navigation.launch

roslaunch pusher_node pusher_node.launch
rosrun pusher_node example.py
```
