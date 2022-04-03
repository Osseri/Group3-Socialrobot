#!/usr/bin/env python
import rospy
import rospkg
import numpy as np
import irm
import copy
import rviz_utils
from data_shape import IDX  # , wIDX  # wIDX: Fwiped
from collision import CollisionBox
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import JointState
from ir_repositioning.srv import Repositioning, RepositioningResponse


class InverseReachabilitySolver:
    # def __init__(self, base_radius):
    def __init__(self):
        # self.config = config

        # Load IRM data
        pkg_dir = rospkg.RosPack().get_path("ir_repositioning")
        self.reposition_R = irm.InverseReachabilityMap("%s/config/robocare_right_irm.npy" % pkg_dir)
        self.reposition_L = irm.InverseReachabilityMap("%s/config/robocare_left_irm.npy" % pkg_dir)

        rospy.loginfo("IRM Loading (Right):\n%s", self.reposition_R)
        rospy.loginfo("IRM Loading (Left):\n%s", self.reposition_L)

        # Start a service
        rospy.Service("/ir_server/find_positions", Repositioning, self.callback_process)

    def solv_one_hand(self, req, is_right=None):
        # INPUT
        Pt = (req.Pt.x, req.Pt.y)
        Obs = [CollisionBox((box.center.x, box.center.y), box.center.theta, box.size_x, box.size_y) for box in req.Obs]

        # min, max (rad)
        Cr = (req.Cr.x, req.Cr.y)
        Ct = (req.Ct.x, req.Ct.y)
        # min_radius, max_radius, interval
        section_def = (
            req.section_definition.x,
            req.section_definition.y,
            req.section_definition.z,
        )

        if is_right is None:
            raise NotImplementedError()
        repos = self.reposition_R if is_right else self.reposition_L

        candidates = repos.calc(Pt, Obs, Cr, Ct, section_def, req.collision_offset, req.max_dist)
        return candidates

    def solv_dual_hand(self, req, verbose=False):
        data_interval = 0.05  # meter

        # Right hand
        # R_Cr = (0, 100)
        # L_Cr = (-100, 0)
        R_Fcut = self.reposition_R.get_Fcut(85, 95, req.max_dist, verbose)
        L_Fcut = self.reposition_L.get_Fcut(-95, -85, req.max_dist, verbose)

        obj_center = (req.Pt.x, req.Pt.y)
        self.reposition_R.target_center = np.array(obj_center)
        self.reposition_L.target_center = np.array(obj_center)

        # self.reposition_R.debug_F(R_Fcut, rviz_utils.t_BLUE, "R_Fcut_look_fw", -0.01, 0.03)
        # self.reposition_L.debug_F(L_Fcut, rviz_utils.t_RED, "L_Fcut_look_fw", -0.02, 0.02)
        # rospy.loginfo("================================")
        # a = input()


        # Change: hand_x_align -> base_x_align
        R_Fcut_look_fw = R_Fcut.copy()
        R_Fcut_look_fw[:, IDX["TCP_X"]] = -R_Fcut[:, IDX["TCP_Y"]]
        R_Fcut_look_fw[:, IDX["TCP_Y"]] = R_Fcut[:, IDX["TCP_X"]]
        L_Fcut_look_fw = L_Fcut.copy()
        L_Fcut_look_fw[:, IDX["TCP_X"]] = L_Fcut[:, IDX["TCP_Y"]]
        L_Fcut_look_fw[:, IDX["TCP_Y"]] = -L_Fcut[:, IDX["TCP_X"]]

        # self.reposition_R.debug_F(R_Fcut_look_fw, rviz_utils.t_BLUE, "R_Fcut_look_fw", -0.01, 0.03)
        # self.reposition_L.debug_F(L_Fcut_look_fw, rviz_utils.t_RED, "L_Fcut_look_fw", -0.02, 0.02)
        # rospy.loginfo("================================")
        # a = input()

        # object width
        _max_obj_width = 0.325
        obj_width = min(req.dual_hand_width, _max_obj_width)
        corrected_width = np.round(obj_width / data_interval) * data_interval
        center_to_hand = corrected_width / 2.0
        rospy.loginfo("obj_width: %f" % obj_width)
        rospy.loginfo("corrected_width: %f" % corrected_width)
        rospy.loginfo("center_to_hand: %f" % center_to_hand)

        R_Fcut_look_fw[:, IDX["TCP_Y"]] -= center_to_hand
        L_Fcut_look_fw[:, IDX["TCP_Y"]] += center_to_hand

        # self.reposition_R.debug_F(R_Fcut_look_fw, rviz_utils.t_BLUE, "R_Fcut_look_fw", -0.01, 0.03)
        # self.reposition_L.debug_F(L_Fcut_look_fw, rviz_utils.t_RED, "L_Fcut_look_fw", -0.02, 0.02)
        # rospy.loginfo("================================")
        # a = input()

        # collection
        def make_int_key(raw_data_point):
            """unit: meter"""
            salt = 0.01
            x = raw_data_point[IDX["TCP_X"]] + salt
            y = raw_data_point[IDX["TCP_Y"]] + salt
            to_meter = 1.0 / data_interval
            return (int(x * to_meter), int(y * to_meter))

        rospy.logwarn(R_Fcut_look_fw.shape)
        l_dict = {make_int_key(p): p for p in L_Fcut_look_fw}
        Fdual = []
        for r in R_Fcut_look_fw:
            xy = make_int_key(r)
            if xy in l_dict:
                l_manip = l_dict[xy][IDX["M"]]
                r_manip = r[IDX["M"]]
                dual_manip = min(l_manip, r_manip)
                dual = r.copy()
                dual[IDX["M"]] = dual_manip
                Fdual.append(dual)
        Fdual = np.array(Fdual)

        # Center filtering
        if (req.strict_dual is True) and (len(Fdual) > 0):
            _ts = data_interval * 0.8
            _yf = Fdual[:, IDX["TCP_Y"]]
            # print("_ts: ", _ts)
            # print("_yf: ", _yf)
            # print("(_yf <= _ts): ", (_yf <= _ts))
            # print("(_yf >= -0.001): ", (_yf >= -0.001))
            _cf = (_yf <= _ts) * (_yf >= -0.001)
            Fdual = Fdual[_cf]
            Fdual[:, IDX["TCP_Y"]] = 0.0

        self.reposition_R.debug_F(Fdual, rviz_utils.GREEN, "Fdual", -0.035, 0.01)

        # Wiped ###############################
        if req.strict_dual is True:
            object_dir_deg = np.degrees(req.Ct.x)
            Fwiped = self.reposition_R.get_F_interpolate_for_strict(
                object_dir_deg, Fdual, verbose, interval=0.01
            )
        else:
            minCt = np.degrees(req.Ct.x)
            maxCt = np.degrees(req.Ct.y)
            Fwiped = self.reposition_R.get_Fwiped(minCt, maxCt, Fdual, verbose, interval=0.01)
        #######################################

        rospy.logwarn("Fwiped shape: %s", Fwiped.shape)
        self.reposition_R.debug_Fwiped(Fwiped, rviz_utils.YELLOW, "Fwiped", -0.03, 0.01)

        # Clean
        Obs = [CollisionBox((box.center.x, box.center.y), box.center.theta, box.size_x, box.size_y) for box in req.Obs]
        Fclean = self.reposition_R.get_Fclean(Obs, Fwiped, req.collision_offset, verbose)
        rospy.logwarn("Fclean shape: %s", Fclean.shape)

        candidates = self.reposition_R.get_candidates(Fclean, num=-1, is_dual=True)
        return candidates

    def callback_process(self, req):
        if req.hand_type == req.DUAL_HAND:
            candidates = self.solv_dual_hand(req)
        elif req.hand_type == req.RIGHT_HAND:
            candidates = self.solv_one_hand(req, is_right=True)
        elif req.hand_type == req.LEFT_HAND:
            candidates = self.solv_one_hand(req, is_right=False)
        else:
            raise ValueError

        rospy.loginfo("num_candidates: %s", len(candidates))
        """
        Candidates:
        | Column Index | Name                     | Unit       | Remark           |
        | ------------ | ------------------------ | ---------- | ---------------- |
        | 0            | Ct                       | **DEGREE** |                  |
        | 1            | Cr                       | **DEGREE** | EE Yaw           |
        | 2            | Mobile Base x            | meter      | For query output |
        | 3            | Mobile Base y            | meter      | For query output |
        | 4            | Manipulability           | -          |                  |
        | 5            | Target Object z (height) | meter      | For query input  |
        | 6            | EEP x                    | meter      | Based on base_footprint |
        | 7            | EEP y                    | meter      | Based on base_footprint |
        | 8            | EEP z                    | meter      | Based on base_footprint |
        | 9            | EE Roll                  | **DEGREE** | Based on base_footprint |
        | 10           | EE Pitch                 | **DEGREE** | Based on base_footprint |
        | 11           | EE Yaw                   | **DEGREE** | Cr, Based on base_footprint |
        | 12           | Joint_0 value            | radian     |                  |
        | 13           | Joint_1 value            | radian     |                  |
        | >=14         | Joint_2... values        | radian     |                  |
        """

        candis = []  # geometry_msgs/Pose2D[]
        appros = []  # float64[]
        manips = []  # float64[]
        joint_angles = []  # sensor_msgs/JointState[]
        for data in candidates:
            Ct, Cr, x, y, m = data[:5]
            joints = data[12:]

            pose = Pose2D()
            pose.x = x
            pose.y = y
            pose.theta = np.radians((Ct - Cr) if req.hand_type != req.DUAL_HAND else Ct)
            candis.append(pose)
            appros.append(Cr)
            manips.append(m)
            joint_angles.append(copy.copy(joints))

        # print(type(candidates))
        if len(candidates):
            rospy.logwarn(
                "IR_Solver Result Sample:\n\tCt: %s\n\tCr: %s\n\t=> theta: %s",
                candidates[0, 0],
                candidates[0, 1],
                candis[0].theta,
            )

        resp = RepositioningResponse()
        resp.num_candidates = len(candidates)
        resp.candidates = candis
        resp.approach_angles = appros
        resp.manipulabilities = manips
        for js in joint_angles:
            jmsg = JointState()
            jmsg.position = js
            resp.joint_angles.append(jmsg)
        return resp


def run_service():
    rospy.init_node("ir_server")
    InverseReachabilitySolver()
    #     float(rospy.get_param("/ir_server/base_radius")),
    #     rospy.get_param("/ir_server/config"),
    # )
    rospy.spin()


if __name__ == "__main__":
    run_service()
