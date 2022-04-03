import numpy as np
from data_shape import IDX, wIDX
import jupyter_utils as jutils
"""
raw:
    [[Cr x y manip],
     [Cr x y manip], ...]
layers:
    [deg]: [[x, ...],
            [y, ...],
            [manip, ...]]
"""

# For Rviz Visualization #########
import rospy
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
import rviz_utils

_id = {
    "R_Fcut": 1100,
    "R_Fcut_rot": 1101,
    "R_Fcut_look_fw": 1102,
    "L_Fcut_look_fw": 1103,
    "Fdual": 1104,
    # points
    "Fcut": 1000,
    "Fwiped": 1001,
    "Fclean": 1002,
    "candidates": 1003,
    # box
    "obs_real": 2000,
    "obs_offset": 3000,
    # sphere
    "best": 4000,
}
##################################


class InverseReachabilityMap:
    def __init__(self, file_path):
        self.file_path = file_path

        # In this file, EE roll, pitch, yaw(Cr) are degree.
        self.irm_raw = np.load(self.file_path)

        #######
        # TODO: no layer conversion
        # manip_layers = futils.raw_to_layers(manipulability_raw)
        # feasi_layers = self.manipulability_to_feasibility(manip_layers)
        # self.irm_raw = futils.layers_to_raw(feasi_layers)
        #######
        # self.free_raw = None

        self.target_center = np.array([0, 0])
        self.rviz_clearing_idx_max = 5
        self.rviz_debug = rospy.Publisher("/ir_server/debug_markers", Marker, queue_size=1, latch=True)
        self.rviz_best = rospy.Publisher("/ir_server/best_base", Marker, queue_size=1, latch=True)

    def __repr__(self):
        repr = "< Inverse Reachability Solver >\n"
        repr += "Configuration:\n"
        repr += "    IRM data: %s\n" % self.file_path
        repr += "       Shape: %s\n" % str(self.irm_raw.shape)
        return repr

    def xy_correction(self, xys):
        """For ROS
        xys: np.array [[x, y], ...]
        """
        corrected = xys + self.target_center
        return corrected

    def get_Fcut(self, minCr, maxCr, max_dist, verbose):
        """min/max Cr is in degree."""
        Crs = self.irm_raw[:, IDX["Y"]]

        filter_arr = (Crs >= minCr) * (Crs <= maxCr)
        Fcut = self.irm_raw[filter_arr].copy()

        # max dist
        def sq_dist(x, y):
            return x**2 + y**2

        in_dist = np.array([sq_dist(x, y) <= (max_dist**2) for x, y in Fcut[:, IDX["TCP_X"]:IDX["TCP_Z"]]])
        Fcut = Fcut[in_dist]
        return Fcut

    def get_Fmax(self, Fcut, section_def, verbose):
        # meter to cm
        min_radius = int(section_def[0] * 100)
        max_radius = int(section_def[1] * 100)
        interval = int(section_def[2] * 100)
        sections = np.array(range(min_radius, max_radius, interval))
        # cm to meter
        sections = sections / 100.0
        sq_sections = np.square(sections)

        num_sections = len(sq_sections) - 1
        Fmax = np.zeros((num_sections, self.irm_raw.shape[1]), dtype=np.float32)

        def find_idx(ascending, value):
            # Check range
            if ascending[0] > value or ascending[-1] < value:
                return None
            # Binary search
            Li = 0
            Ri = len(ascending) - 1
            while (Li + 1) < Ri:
                mid = int((Li + Ri) / 2)
                if value < ascending[mid]:
                    Ri = mid
                else:
                    Li = mid
            return Li

        for point in Fcut:
            x, y = point[IDX["TCP_X"]:IDX["TCP_Z"]]
            sq_radius = x * x + y * y
            idx = find_idx(sq_sections, sq_radius)
            if idx is not None:
                prev_manip = Fmax[idx, IDX["M"]]
                new_manip = point[IDX["M"]]
                if prev_manip < new_manip:
                    Fmax[idx] = point.copy()

        # Trim the zero-sections
        trimmed = Fmax[:, IDX["M"]] > 0
        Fmax = Fmax[trimmed]
        # Fmax = Fcut

        # PLOT
        # ROS
        _xys = self.xy_correction(Fcut[:, IDX["TCP_X"]:IDX["TCP_Z"]])
        _points = [(_x, _y, 0.0) for _x, _y in _xys]
        _ms = Fcut[:, IDX["M"]]
        if _ms.size:
            # _colors = jutils.get_colors(_ms)
            _colors = [rviz_utils.t_DARK for _ in _xys]
            self.rviz_debug.publish(rviz_utils.create_points(
                _id["Fcut"],
                _points,
                _colors,
            ))
        return Fmax

    def get_Fwiped(self, minCt, maxCt, Fmax, verbose, interval=0.03):
        """min/max Ct is in degree."""
        """
        Fwiped
        [
            Ct,
            Cr,
            base_x,
            base_y,
            m,
            target z,
            eep x, y, z, r, p, y,
            joint 1, 2, 3...
        ]
        """
        # interval = 0.03  # m
        two_pi = 2.0 * np.pi

        circle_points = []
        for point in Fmax:
            Cr_ = point[IDX["Y"]]
            x = point[IDX["TCP_X"]]
            y = point[IDX["TCP_Y"]]
            m_ = point[IDX["M"]]
            target_z_ = point[IDX["TCP_Z"]]
            eep_ = point[IDX["EEP_X"]:IDX["M"]]
            joints_ = point[IDX["Joint"]:]

            radius = np.sqrt(x * x + y * y)
            d_rad = interval / radius

            num_points = int(two_pi / d_rad)
            half = int(num_points / 2)
            # Circular Mapping in the range of Ct
            # # Now, it is hardcoded in -180 ~ 180. (X)
            for idx in range(-half, num_points - half):
                rad = d_rad * idx
                Ct = np.degrees(rad)

                # Ct
                if (Ct > minCt) and (Ct < maxCt):
                    c, s = np.cos(rad), np.sin(rad)
                    nx = c * x - s * y
                    ny = s * x + c * y

                    dummy = np.concatenate(([Ct, Cr_, nx, ny, m_, target_z_], eep_, joints_), axis=0)
                    circle_points.append(dummy)
        Fwiped = np.array(circle_points)

        # ROS
        # rospy.loginfo(Fwiped.shape)
        # _xy_ = Fwiped[:, wIDX["Bx"]:wIDX["M"]]
        # rospy.loginfo(_xy_.shape)

        _xys = self.xy_correction(Fwiped[:, wIDX["Bx"]:wIDX["M"]])
        _points = [(_x, _y, -0.01) for _x, _y in _xys]
        _colors = [rviz_utils.t_BLUE for _ in _xys]
        self.rviz_debug.publish(rviz_utils.create_points(
            _id["Fwiped"],
            _points,
            _colors,
            size=0.005,
        ))
        return Fwiped

    def get_F_interpolate_for_strict(self, object_dir_deg, Fdual, verbose, interval=0.03):
        """Modified from get_Fwiped()"""
        """
        Fwiped
        [
            Ct,
            Cr,
            base_x,
            base_y,
            m,
            target z,
            eep x, y, z, r, p, y,
            joint 1, 2, 3...
        ]
        """
        # Make Z layers
        z_layers = {}  # {z: {x: point}...}
        for point in Fdual:
            z_key = int(1000.0 * point[IDX["TCP_Z"]])
            if z_key not in z_layers:
                z_layers[z_key] = {}

            x_key = int(1000.0 * point[IDX["TCP_X"]])
            if x_key not in z_layers[z_key]:
                z_layers[z_key][x_key] = point
            elif z_layers[z_key][x_key][IDX["M"]] > point[IDX["M"]]:
                z_layers[z_key][x_key] = point

        # per layer
        for line in z_layers.values():
            # X-axis Sort (ascending)
            x_keys = sorted(line.keys())
            # Linear interpolation
            prev_x = None
            for x_key in x_keys:
                if prev_x is None:
                    prev_x = x_key
                    continue

                prev_point = line[prev_x]
                curr_point = line[x_key]

                dx = curr_point[IDX["TCP_X"]] - prev_point[IDX["TCP_X"]]
                dex = curr_point[IDX["EEP_X"]] - prev_point[IDX["EEP_X"]]
                dm = curr_point[IDX["M"]] - prev_point[IDX["M"]]

                ratio = [1. / 4., 2. / 4., 3. / 4.]
                for r in ratio:
                    new_point = prev_point.copy()
                    new_x_key = new_point[IDX["TCP_X"]] + r * dx
                    new_point[IDX["TCP_X"]] = new_x_key
                    new_point[IDX["EEP_X"]] += r * dex
                    new_point[IDX["M"]] += r * dm
                    line[new_x_key] = new_point
                    # print(len(line))
                prev_x = x_key

        # Convert to np.array
        Ct = object_dir_deg
        rad = np.radians(Ct)
        linear_points = []
        for line in z_layers.values():
            for point in line.values():
                Cr_ = point[IDX["Y"]]
                x = point[IDX["TCP_X"]]
                y = point[IDX["TCP_Y"]]
                m_ = point[IDX["M"]]
                target_z_ = point[IDX["TCP_Z"]]
                eep_ = point[IDX["EEP_X"]:IDX["M"]]
                joints_ = point[IDX["Joint"]:]

                c, s = np.cos(rad), np.sin(rad)
                nx = c * x - s * y
                ny = s * x + c * y
                dummy = np.concatenate(([Ct, Cr_, nx, ny, m_, target_z_], eep_, joints_), axis=0)
                linear_points.append(dummy)
        Finterpolated = np.array(linear_points)

        ##########

        # circle_points = []
        # for point in Fmax:

        #     radius = np.sqrt(x * x + y * y)
        #     d_rad = interval / radius

        #     num_points = int(two_pi / d_rad)
        #     half = int(num_points / 2)
        #     # Circular Mapping in the range of Ct
        #     # # Now, it is hardcoded in -180 ~ 180. (X)
        #     for idx in range(-half, num_points - half):
        #         rad = d_rad * idx
        #         Ct = np.degrees(rad)

        #         # Ct
        #         if (Ct > minCt) and (Ct < maxCt):

        # Fwiped = np.array(circle_points)
        #########3
        # Fwiped = np.array(circle_points)

        # ROS
        # rospy.loginfo(Fwiped.shape)
        # _xy_ = Fwiped[:, wIDX["Bx"]:wIDX["M"]]
        # rospy.loginfo(_xy_.shape)

        _xys = self.xy_correction(Finterpolated[:, wIDX["Bx"]:wIDX["M"]])
        _points = [(_x, _y, -0.01) for _x, _y in _xys]
        _colors = [rviz_utils.t_BLUE for _ in _xys]
        self.rviz_debug.publish(rviz_utils.create_points(
            _id["Fwiped"],
            _points,
            _colors,
            size=0.005,
        ))
        return Finterpolated

    def debug_F(self, F, color_rgba, name, z, size):
        """Fraw only. Fwiped has different idx."""
        # _xys = self.xy_correction(F[:, wIDX["Bx"]:wIDX["M"]])
        _xys = self.xy_correction(F[:, IDX["TCP_X"]:IDX["TCP_Z"]])
        _points = [(x, y, z) for x, y in _xys]
        _colors = [color_rgba for _ in _xys]
        self.rviz_debug.publish(rviz_utils.create_points(
            _id[name],
            _points,
            _colors,
            size=size,
        ))

    def debug_Fwiped(self, Fwiped, color_rgba, name, z, size):
        """Fraw only. Fwiped has different idx."""
        # _xys = self.xy_correction(F[:, wIDX["Bx"]:wIDX["M"]])
        _xys = self.xy_correction(Fwiped[:, wIDX["Bx"]:wIDX["M"]])
        _points = [(x, y, z) for x, y in _xys]
        _colors = [color_rgba for _ in _xys]
        self.rviz_debug.publish(rviz_utils.create_points(
            _id[name],
            _points,
            _colors,
            size=size,
        ))

    def get_Fclean(self, Obs, Fwiped, collision_offset, verbose):
        """target_center must be setted."""
        """
        Fwiped
        [Ct Cr x y m]
        """
        # Pt(self.target_center), Obs
        # Fclean = Fwiped.copy()
        Fclean = Fwiped
        for collision in Obs:
            collision.set_offset(collision_offset)
            filter_arr = np.array([not collision.check(p[wIDX["Bx"]:wIDX["M"]] + self.target_center) for p in Fclean])
            Fclean = Fclean[filter_arr]

        # free_raw = Fclean.copy()
        free_raw = Fclean

        # ROS
        def _get_line_strip(_id, vertices, color):
            """vertices: [[x, y], ...]"""
            closed = np.append(vertices, [vertices[0]], axis=0)
            _points = [(x, y, 0.0) for x, y in closed]
            _colors = [color for _ in closed]
            return rviz_utils.create_line(_id, _points, _colors)

        # obstacles
        idx = 0
        for collision in Obs:
            _vxys = np.transpose(collision.vertices)
            _oxys = np.transpose(collision.offsets)
            self.rviz_debug.publish(_get_line_strip(_id["obs_real"] + idx, _vxys, rviz_utils.WHITE))
            self.rviz_debug.publish(_get_line_strip(_id["obs_offset"] + idx, _oxys, rviz_utils.RED))
            idx += 1
        empty_box = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
        for _i in xrange(idx, self.rviz_clearing_idx_max):
            self.rviz_debug.publish(_get_line_strip(_id["obs_real"] + idx, empty_box, rviz_utils.t_NO_COLOR))
            self.rviz_debug.publish(_get_line_strip(_id["obs_offset"] + idx, empty_box, rviz_utils.t_NO_COLOR))

        # ROS
        _xys = self.xy_correction(Fclean[:, wIDX["Bx"]:wIDX["M"]])
        _points = [(x, y, 0.0) for x, y in _xys]
        _ms = Fclean[:, wIDX["M"]]
        if _ms.size:
            _colors = jutils.get_colors(_ms)
            self.rviz_debug.publish(rviz_utils.create_points(
                _id["Fclean"],
                _points,
                _colors,
            ))
        return free_raw

    def get_candidates(self, free_raw, num=-1, is_dual=False, verbose=False):
        """
        free_raw (descending order) relative to target coordinates
        [Ct Cr x y m]
        if num < 0, collect all max(M) points.
        """
        # When no answer
        if not free_raw.size:
            return []
        # Sorting in descending order
        ms = -np.transpose(free_raw)[wIDX["M"]]
        s = ms.argsort()
        Fsort = free_raw[s]
        if num >= 0:
            # Collect top N points
            candidates = Fsort[:num].copy()
        else:
            # Collect all points of maximum manipulability.
            idx = 0
            w = wIDX["M"]
            prev_M = Fsort[0][w]
            for p in Fsort:
                if prev_M > p[w]:
                    break
                idx += 1
            candidates = Fsort[:idx].copy()

        # candidates
        _xys = self.xy_correction(candidates[:, wIDX["Bx"]:wIDX["M"]])
        candidates[:, wIDX["Bx"]:wIDX["M"]] = _xys

        # ROS
        _points = [(xy[0], xy[1], 0.0) for xy in _xys]
        _colors = [rviz_utils.t_PURPLE for _ in _xys]
        self.rviz_debug.publish(rviz_utils.create_points(
            _id["candidates"],
            _points,
            _colors,
            size=0.015,
        ))
        # best_point
        Ct, Cr = candidates[0, :wIDX["Bx"]]
        theta = np.radians((Ct - Cr) if not is_dual else Ct)
        length = 0.2
        width = 0.03
        height = 0.03
        best_point = rviz_utils.create_marker(
            _id["best"],
            _points[0],
            quaternion_from_euler(theta, 0, 0, axes="rzxy"),
            (length, width, height),
            rviz_utils.PURPLE,
            Marker.ARROW,
        )
        self.rviz_best.publish(best_point)
        if candidates is None:
            candidates = []
        return candidates

    @staticmethod
    def candidate_inspector(candidate_point):
        Ct = candidate_point[wIDX["Ct"]]  # deg
        Cr = candidate_point[wIDX["Cr"]]  # deg
        Bx = candidate_point[wIDX["Bx"]]
        By = candidate_point[wIDX["By"]]
        M = candidate_point[wIDX["M"]]
        Tz = candidate_point[wIDX["Tz"]]
        # m, m, m, deg, deg, deg
        EEPx, EEPy, EEPz, EEProll, EEPpitch, EEPyaw = candidate_point[wIDX["EEPx"]:wIDX["J"]]
        J = candidate_point[wIDX["J"]:]  # rad
        heading = Ct - Cr  # deg

        print("Candidate:")
        print("\t   Base Pose2D: (x: %.3f m, y: %.3f m, theta: %.3f deg)" % (Bx, By, heading))
        print("\t            Cr: %.3f deg" % Cr)
        print("\t            Ct: %.3f deg" % Ct)
        print("\t      Target Z: %.3f m" % Tz)
        print("\tManipulability: %.5f" % M)
        print("\t  End-effecotr: (x: %.3f m, y: %.3f m, z: %.3f m)" % (EEPx, EEPy, EEPz))
        print("\t                (r: %.3f m, p: %.3f m, y: %.3f m)" % (EEProll, EEPpitch, EEPyaw))
        print("\t                based on [base_footprint]")
        j_string = ["%.1f" % deg for deg in np.degrees(J)]
        print("\t   Joint (deg): ", j_string)

    def calc(self, Pt, Obs, Cr, Ct, section_def, collision_offset, max_dist, verbose=False):
        """
        * Distance unit: meters
        * Angle unit: radians

        Pt: Position of the target object (in global coordinates)
            - format: (x, y)
        Obs: Area list of ground obstacles (in global coordinates)
            - format: [CollisionModel, ...]
        Cr: Constraints on the approach angle (relative to the robot heading)
            - format: (min, max)
            - range: -pi ~ pi
        Ct: Constraints on the approach angle (in global coordinates)
            - format: (min, max)
            - range: -pi ~ pi
        section_def: The definition of ROI (Region of interest)
            - format: (min_radius, max_radius, interval)
        """
        # self.free_raw = None
        self.target_center = np.array(Pt)

        ###################################
        # 1. Fcut
        ###################################
        minCr = np.degrees(Cr[0])
        maxCr = np.degrees(Cr[1])
        Fcut = self.get_Fcut(minCr, maxCr, max_dist, verbose)
        rospy.logwarn("Fcut: %s", Fcut.shape)

        ###################################
        # 2. Fmax (skip)
        ###################################
        # Fmax = self.get_Fmax(Fcut, section_def, verbose)
        Fmax = Fcut

        ###################################
        # 3. Fwiped
        ###################################
        minCt = np.degrees(Ct[0])
        maxCt = np.degrees(Ct[1])
        Fwiped = self.get_Fwiped(minCt, maxCt, Fmax, verbose)
        rospy.logwarn("Fwiped: %s", Fwiped.shape)

        ###################################
        # 4. Fclean
        ###################################
        Fclean = self.get_Fclean(Obs, Fwiped, collision_offset, verbose)
        rospy.logwarn("Fclean: %s", Fclean.shape)

        # candidates
        candidates = self.get_candidates(Fclean, num=-1)
        return candidates
