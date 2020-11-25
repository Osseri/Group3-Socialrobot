#!/usr/bin/env python
def move_goalpose_client(arm_name, hand_name, start_state, goal_pos, goal_ori, obj, planner_name, n_attempt, c_time, n_repeat):
    import rospy
    from arm_move.srv._arm_move_srv import *

    rospy.wait_for_service('move_goalpose_srv')
    try:
        f_check_srv = rospy.ServiceProxy('move_goalpose_srv', arm_move_srv)
        # Grasp the object if obj
        if obj:
            att_box_client(hand_name, obj)
        pub_msg = arm_move_srvRequest()
        pub_msg.arm_name.append(arm_name)
        pub_msg.goal_position.x = goal_pos[0]
        pub_msg.goal_position.y = goal_pos[1]
        pub_msg.goal_position.z = goal_pos[2]
        pub_msg.goal_orientation.x = goal_ori[0]
        pub_msg.goal_orientation.y = goal_ori[1]
        pub_msg.goal_orientation.z = goal_ori[2]
        pub_msg.goal_orientation.w = goal_ori[3]
        pub_msg.planner_name = planner_name
        pub_msg.n_attempt = n_attempt
        pub_msg.c_time = c_time
        pub_msg.n_repeat = n_repeat
        pub_msg.start_state = start_state
        resp_f_check = f_check_srv(pub_msg)
        if resp_f_check.feasibility == 1:
            if len(resp_f_check.r_trj.joint_trajectory.points) > 0:
                print "Plan steps:", len(resp_f_check.r_trj.joint_trajectory.points),
                return [1, resp_f_check.r_trj]
            else:
                print "No plan",
                return [0, resp_f_check.r_trj]
        else:
            print "No plan",
            return [0, resp_f_check.r_trj]
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def feasible_check_obj_joint_client(arm_name, hand_name, start_state, goal_pos, goal_ori, obj, planner_name, n_attempt, c_time, n_repeat):
    import rospy
    from arm_move.srv._arm_move_srv import *
    rospy.wait_for_service('feasibile_check_srv')
    try:
        f_check_srv = rospy.ServiceProxy('feasibile_check_srv', arm_move_srv)
        # Grasp the object if obj
        if obj:
            att_box_client(hand_name, obj)
        pub_msg = arm_move_srvRequest()
        pub_msg.arm_name.append(arm_name)
        pub_msg.goal_position.x = goal_pos[0]
        pub_msg.goal_position.y = goal_pos[1]
        pub_msg.goal_position.z = goal_pos[2]
        pub_msg.goal_orientation.x = goal_ori[0]
        pub_msg.goal_orientation.y = goal_ori[1]
        pub_msg.goal_orientation.z = goal_ori[2]
        pub_msg.goal_orientation.w = goal_ori[3]
        pub_msg.planner_name = planner_name
        pub_msg.n_attempt = n_attempt
        pub_msg.c_time = c_time
        pub_msg.n_repeat = n_repeat
        pub_msg.start_state = start_state
        resp_f_check = f_check_srv(pub_msg)
        # print "feasibility check", resp_f_check
        if resp_f_check.feasibility == 1:
            if len(resp_f_check.r_trj.joint_trajectory.points) > 0:
                print "Plan steps:", len(resp_f_check.r_trj.joint_trajectory.points),
                return [1, resp_f_check.r_trj]
            else:
                print "No plan",
                return [0, resp_f_check.r_trj]
        else:
            print "No plan",
            return [0, resp_f_check.r_trj]
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def add_box_client(box_name, box_xyz, box_xyzw, box_wdh, box_color):
    import rospy
    from arm_move.srv._box_info_srv import *
    rospy.wait_for_service('add_box_srv')
    try:
        add_box_srv = rospy.ServiceProxy('add_box_srv', box_info_srv)

        pub_msg = box_info_srvRequest()
        pub_msg.object_name.append(box_name)
        pub_msg.object_color.append(box_color)

        pub_msg.object_position.x = box_xyz[0]
        pub_msg.object_position.y = box_xyz[1]
        pub_msg.object_position.z = box_xyz[2]
        pub_msg.object_orientation.x = box_xyzw[0]
        pub_msg.object_orientation.y = box_xyzw[1]
        pub_msg.object_orientation.z = box_xyzw[2]
        pub_msg.object_orientation.w = box_xyzw[3]
        pub_msg.object_scale.x = box_wdh[0]
        pub_msg.object_scale.y = box_wdh[1]
        pub_msg.object_scale.z = box_wdh[2]

        resp1 = add_box_srv(pub_msg)

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def del_box_client(box_name):
    import rospy
    from arm_move.srv._box_info_srv import *

    rospy.wait_for_service('del_box_srv')
    try:
        del_box_srv = rospy.ServiceProxy('del_box_srv', box_info_srv)
        # pub_msg = box_info_msg()
        pub_msg = box_info_srvRequest()
        pub_msg.object_name.append(box_name)

        resp1 = del_box_srv(pub_msg)

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def att_box_client(hand_name, box_name):
    import rospy
    from arm_move.srv._att_hand_box_srv import *

    rospy.wait_for_service('att_box_srv')
    try:
        att_box_srv = rospy.ServiceProxy('att_box_srv', att_hand_box_srv)

        pub_msg = att_hand_box_srvRequest()
        pub_msg.object_name.append(box_name)
        pub_msg.hand_name.append(hand_name)

        resp1 = att_box_srv(pub_msg)

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def det_box_client(box_name, box_xyz=[0, 0, 0], box_xyzw=[0, 0, 0, 0], box_wdh=[0, 0, 0], box_color='red'):
    import rospy
    from arm_move.srv._box_info_srv import *

    rospy.wait_for_service('det_box_srv')
    try:
        det_box_srv = rospy.ServiceProxy('det_box_srv', box_info_srv)

        pub_msg = box_info_srvRequest()
        pub_msg.object_name.append(box_name)
        pub_msg.object_color.append(box_color)

        pub_msg.object_position.x = box_xyz[0]
        pub_msg.object_position.y = box_xyz[1]
        pub_msg.object_position.z = box_xyz[2]
        pub_msg.object_orientation.x = box_xyzw[0]
        pub_msg.object_orientation.y = box_xyzw[1]
        pub_msg.object_orientation.z = box_xyzw[2]
        pub_msg.object_orientation.w = box_xyzw[3]
        pub_msg.object_scale.x = box_wdh[0]
        pub_msg.object_scale.y = box_wdh[1]
        pub_msg.object_scale.z = box_wdh[2]

        resp1 = det_box_srv(pub_msg)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def draw_box_client(box_name, box_xyz, box_xyzw, box_wdh, box_color):
    import rospy
    from arm_move.srv._box_info_srv import *

    rospy.wait_for_service('draw_box_srv')
    try:
        draw_box_srv = rospy.ServiceProxy('draw_box_srv', box_info_srv)
        # pub_msg = box_info_msg()
        pub_msg = box_info_srvRequest()
        # pub_srv.header = 0
        pub_msg.object_name.append(box_name)
        pub_msg.object_color.append(box_color)

        pub_msg.object_position.x = box_xyz[0]
        pub_msg.object_position.y = box_xyz[1]
        pub_msg.object_position.z = box_xyz[2]
        pub_msg.object_orientation.x = box_xyzw[0]
        pub_msg.object_orientation.y = box_xyzw[1]
        pub_msg.object_orientation.z = box_xyzw[2]
        pub_msg.object_orientation.w = box_xyzw[3]
        pub_msg.object_scale.x = box_wdh[0]
        pub_msg.object_scale.y = box_wdh[1]
        pub_msg.object_scale.z = box_wdh[2]

        resp1 = draw_box_srv(pub_msg)

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def move_cartesian(arm_name, goal_position, goal_orientation):
    import rospy
    from arm_move.srv._arm_move_srv import *
    rospy.wait_for_service('arm_car_path_srv')
    try:
        m_joints_srv = rospy.ServiceProxy('arm_car_path_srv', arm_move_srv)

        pub_msg = arm_move_srvRequest()
        pub_msg.arm_name.append(arm_name)
        pub_msg.goal_position.x = goal_position[0]
        pub_msg.goal_position.y = goal_position[1]
        pub_msg.goal_position.z = goal_position[2]
        pub_msg.goal_orientation.x = goal_orientation[0]
        pub_msg.goal_orientation.y = goal_orientation[1]
        pub_msg.goal_orientation.z = goal_orientation[2]
        pub_msg.goal_orientation.w = goal_orientation[3]

        resp1 = m_joints_srv(pub_msg)

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def move_joints_client_deg(arm_name, jointGoal):
    import rospy
    import numpy as np
    from arm_move.srv._arm_goalJoint_srv import *

    rospy.wait_for_service('arm_goalJoint_srv')
    try:
        m_joints_srv = rospy.ServiceProxy('arm_goalJoint_srv', arm_goalJoint_srv)
        # pub_msg = box_info_msg()
        pub_msg = arm_goalJoint_srvRequest()
        pub_msg.goalPose.name = [arm_name]
        radGoal = np.deg2rad(jointGoal)
        pub_msg.goalPose.position = radGoal

        resp1 = m_joints_srv(pub_msg)

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def move_joints_client_rad(arm_name, jointGoal):
    import rospy
    from arm_move.srv._arm_goalJoint_srv import *

    rospy.wait_for_service('arm_goalJoint_srv')
    try:
        m_joints_srv = rospy.ServiceProxy('arm_goalJoint_srv', arm_goalJoint_srv)
        # pub_msg = box_info_msg()
        pub_msg = arm_goalJoint_srvRequest()
        pub_msg.arm_name = [arm_name]
        # radGoal = np.deg2rad(jointGoal)
        pub_msg.goalPose.position = jointGoal
        print"\tCL : go to", jointGoal,
        resp1 = m_joints_srv(pub_msg)
        if resp1.w_flag == 1:
            print "work done"
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def move_hand_joint_client(joint_set):
    import rospy
    from arm_move.srv._arm_goalJoint_srv import *
    rospy.wait_for_service('jaco_hand_goalJoint_srv')
    try:
        m_joints_srv = rospy.ServiceProxy('jaco_hand_goalJoint_srv', arm_goalJoint_srv)
        pub_msg = arm_goalJoint_srvRequest()
        pub_msg.arm_name.append('gripper')
        pub_msg.goalPose.position = joint_set

        # pub_msg.arm_name.append('r_arm')
        # pub_msg.goalPose.position = [0.6, -0.3, -0.05, -2.25, -1.59, 0.3, 0.01]

        print"\tCL : move hand to", joint_set,
        resp1 = m_joints_srv(pub_msg)

        # print "response", resp1
        if resp1.w_flag == 1:
            print "work done"
            return resp1.w_flag
        else:
            print "can not close more"
            return resp1.w_flag

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


if __name__ == "__main__":
    print "This file has list of custom made functions"