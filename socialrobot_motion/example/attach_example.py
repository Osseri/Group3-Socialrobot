#!/usr/bin/env python

import moveit_commander
import geometry_msgs
import rospy
import sys


def wait_for_state_update(box_name, scene, box_is_known=False, box_is_attached=False, timeout=4):
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
        attached_objects = scene.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0
        is_known = box_name in scene.get_known_object_names()
        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True
        rospy.sleep(0.1)
        seconds = rospy.get_time()
    return False

if __name__ == '__main__':
    rospy.init_node('attach_object_async', anonymous=True)
    #moveit_commander.roscpp_initialize(sys.argv)
    
    scene = moveit_commander.PlanningSceneInterface()
    # ROS answers suggested adding a sleep here to allow the PlanningSceneInterface
    # to initialize properly; we have noticed that collision objects aren't
    # always received without it.
    rospy.sleep(1.0)
    
    # # create the box we will be using to check for collision detection
    # box_name = 'box1'
    # box_pose = geometry_msgs.msg.PoseStamped()
    # box_pose.header.frame_id = "base_footprint"
    # box_pose.pose.orientation.w = 1.0
    # box_pose.pose.position.z = 0.2
    # box_pose.pose.position.x = 0.6
    # box_pose.pose.position.y = -0.6
    # scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.4))
    
    # box_name = 'box2'
    # box_pose = geometry_msgs.msg.PoseStamped()
    # box_pose.header.frame_id = "base_footprint"
    # box_pose.pose.orientation.w = 1.0
    # box_pose.pose.position.z = 0.2
    # box_pose.pose.position.x = 0.6
    # box_pose.pose.position.y = 0
    # scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.4))    

    # box_name = 'box3'
    # box_pose = geometry_msgs.msg.PoseStamped()
    # box_pose.header.frame_id = "base_footprint"
    # box_pose.pose.orientation.w = 1.0
    # box_pose.pose.position.z = 0.2
    # box_pose.pose.position.x = 0.6
    # box_pose.pose.position.y = 0.6
    # scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.4))
    
    # robot = moveit_commander.RobotCommander()
    # leftarm_group = moveit_commander.MoveGroupCommander('left_arm')
    # rightarm_group = moveit_commander.MoveGroupCommander('right_arm')
    # lefteef_link = leftarm_group.get_end_effector_link()
    # righteef_link = rightarm_group.get_end_effector_link()

    # #rospy.loginfo(wait_for_state_update(box_name, scene, box_is_known=True))

    # grasping_group = 'left_eef'
    # touch_links = robot.get_link_names(group=grasping_group)
    # touch_links.append(righteef_link)    

    # scene.attach_box(righteef_link, 'box3', touch_links=touch_links)
    # #scene.remove_attached_object(lefteef_link, 'obj_red_gotica')    
    
    print('------objects-------')
    print (scene.get_known_object_names())
    print('------attached objects-------')
    print(scene.get_attached_objects().keys())
    attached_objects = scene.get_attached_objects()
    scene_objects = scene.get_objects()
    print(attached_objects)

    for obj in scene_objects.keys():
        print(scene_objects[obj].id)
        print(scene_objects[obj].primitives[0].dimensions)
        print(scene_objects[obj].primitive_poses[0].position)
        print(scene_objects[obj].primitive_poses[0].orientation)

    for obj in attached_objects.keys():
        print(attached_objects[obj].object.id)
        print(attached_objects[obj].object.primitives[0].dimensions)
        print(attached_objects[obj].object.primitive_poses[0].position)
        print(attached_objects[obj].object.primitive_poses[0].orientation)