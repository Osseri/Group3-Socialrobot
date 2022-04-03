import numpy as np
import matplotlib.pyplot as plt
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
try:
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
except ModuleNotFoundError:
    pass
##################################


class InverseReachabilityMap:
    def __init__(self, file_path, robot_radius, is_jupyter=False):
        self.robot_radius = robot_radius
        self.file_path = file_path
        self._is_jupyter = is_jupyter

        # In this file, EE roll, pitch, yaw(Cr) are degree.
        self.irm_raw = np.load(self.file_path)

        max_x = max(abs(self.irm_raw[:, IDX["TCP_X"]])) * 1.1
        max_y = max(abs(self.irm_raw[:, IDX["TCP_Y"]])) * 1.1
        boundary = max(max_x, max_y)
        self.xyMinMax = [-boundary, boundary, -boundary, boundary]

        #######
        # TODO: no layer conversion
        # manip_layers = futils.raw_to_layers(manipulability_raw)
        # feasi_layers = self.manipulability_to_feasibility(manip_layers)
        # self.irm_raw = futils.layers_to_raw(feasi_layers)
        #######
        # self.free_raw = None

        self.target_center = np.array([0, 0])
        self.rviz_clearing_idx_max = 5
        if not self._is_jupyter:  # ROS
            self.rviz_debug = rospy.Publisher("/ir_server/debug_markers", Marker, queue_size=1, latch=True)
            self.rviz_best = rospy.Publisher("/ir_server/best_base", Marker, queue_size=1, latch=True)

    def __repr__(self):
        repr = "< Inverse Reachability Solver >\n"
        repr += "Configuration:\n"
        repr += "    Base radius: %f m\n" % self.robot_radius
        repr += "    IRM data: %s\n" % self.file_path
        repr += "       Shape: %s\n" % str(self.irm_raw.shape)
        return repr

    def xy_correction(self, xys):
        """For ROS
        xys: np.array [[x, y], ...]
        """
        corrected = xys + self.target_center
        return corrected

    def get_Fcut(self, minCr, maxCr, verbose):
        """min/max Cr is in degree."""
        Crs = self.irm_raw[:, IDX["Y"]]

        if self._is_jupyter and verbose:
            print("######################")
            print("# 1. Fcut")
            print("######################")
            print("Cut the range of `Cr` from `Fraw` and set it to `Fcut`.")
            print("Lower constraints on Cr: %f deg", minCr)
            print("Upper constraints on Cr: %f deg", maxCr)
            print("Input Cr range in degree: ", sorted(set(Crs)))

        filter_arr = (Crs >= minCr) * (Crs <= maxCr)
        Fcut = self.irm_raw[filter_arr].copy()

        # PLOT
        if self._is_jupyter and verbose:
            fig = plt.figure()
            ax1 = fig.add_subplot(121, projection="3d")
            ax2 = fig.add_subplot(122, projection="3d")
            jutils.scatter_3d(ax1, jutils.filtering(self.irm_raw), self.xyMinMax, "Raw IRM")
            jutils.scatter_3d(ax2, jutils.filtering(Fcut), self.xyMinMax, "Fcut")
            plt.show()
        return Fcut

    def get_Fmax(self, Fcut, section_def, verbose):
        if self._is_jupyter and verbose:
            print("######################")
            print("# 2. Fmax")
            print("######################")
            print("And extract only the maximum as a `Fmax`.")

        # meter to cm
        min_radius = int(section_def[0] * 100)
        max_radius = int(section_def[1] * 100)
        interval = int(section_def[2] * 100)
        sections = np.array(range(min_radius, max_radius, interval))
        # cm to meter
        sections = sections / 100.0
        sq_sections = np.square(sections)
        if self._is_jupyter and verbose:
            print("sq_sections.shape: ", sq_sections.shape)

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
        if self._is_jupyter:
            if verbose:
                print("Fmax.shape: ", Fmax.shape)
                # print("Fmax:\n", Fmax)
                print("When Ct == 0.0 deg")

                fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)
                jutils.scatter_2d(ax1, jutils.filtering(Fcut), self.xyMinMax, "Fcut")
                jutils.scatter_2d(ax2, jutils.filtering(Fmax), self.xyMinMax, "Fmax")
                for r in sections:
                    ax1.add_artist(plt.Circle((0, 0), radius=r, color="gray", alpha=0.4, fill=False))
                    ax2.add_artist(plt.Circle((0, 0), radius=r, color="gray", alpha=0.4, fill=False))
                fig.tight_layout()
                plt.show()
        else:
            # ROS
            _xys = self.xy_correction(Fcut[:, IDX["TCP_X"]:IDX["TCP_Z"]])
            _points = [(x, y, 0.0) for x, y in _xys]
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
        if self._is_jupyter and verbose:
            print("######################")
            print("# 3. Fwiped")
            print("######################")
            print("Wipe the `Fmax` in the range of `Ct`. => `Fwiped`")
            print("min Ct: ", minCt)
            print("max Ct: ", maxCt)
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

        if self._is_jupyter:
            if verbose:
                print("Fwiped.shape: ", Fwiped.shape)
                print("Interval is ", interval, " [m].")
                fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)
                wiped_plot = Fwiped[:, wIDX["Cr"]:wIDX["Tz"]]
                jutils.scatter_2d(ax1, jutils.filtering(Fmax), self.xyMinMax, "Fmax")
                jutils.scatter_2d(ax2, wiped_plot, self.xyMinMax, "Fwiped")

                max_manip = np.max(Fmax[:, IDX["M"]])
                min_manip = np.min(Fmax[:, IDX["M"]])
                grad = jutils.ColorGradient()

                for point in Fmax:
                    cr = point[IDX["Y"]]
                    x = point[IDX["TCP_X"]]
                    y = point[IDX["TCP_Y"]]
                    m = point[IDX["M"]]
                    ax1.add_artist(
                        plt.Circle(
                            (0, 0),
                            radius=np.sqrt(x * x + y * y),
                            color=grad.get(jutils.scale_remapping(m, max_manip, min_manip, 1, 0)),
                            fill=False,
                            linewidth=0.2,
                        ))
                    ax1.add_artist(plt.Circle((x, y), radius=0.2 / 100.0, color="red", fill=False))
                fig.tight_layout()
                plt.show()
        else:
            # ROS
            # rospy.loginfo(Fwiped.shape)
            # _xy_ = Fwiped[:, wIDX["Bx"]:wIDX["M"]]
            # rospy.loginfo(_xy_.shape)

            _xys = self.xy_correction(Fwiped[:, wIDX["Bx"]:wIDX["M"]])
            _points = [(x, y, -0.01) for x, y in _xys]
            _colors = [rviz_utils.t_BLUE for _ in _xys]
            self.rviz_debug.publish(rviz_utils.create_points(
                _id["Fwiped"],
                _points,
                _colors,
                size=0.005,
            ))
        return Fwiped

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

    def get_Fclean(self, Obs, Fwiped, verbose):
        """target_center must be setted."""
        if self._is_jupyter and verbose:
            print("######################")
            print("# 4. Fclean")
            print("######################")
            print("Remove all obstacle areas from `Fwiped` with the offset of `Rsize`. => `Fclean`")
        """
        Fwiped
        [Ct Cr x y m]
        """
        # Pt(self.target_center), Obs
        # Fclean = Fwiped.copy()
        Fclean = Fwiped
        if self._is_jupyter and verbose:
            print("Fclean.shape: ", Fclean.shape)
        for collision in Obs:
            collision.set_offset(self.robot_radius)
            filter_arr = [not collision.check(p[wIDX["Bx"]:wIDX["M"]] + self.target_center) for p in Fclean]
            Fclean = Fclean[filter_arr]
            if self._is_jupyter and verbose:
                print("Fclean.shape: ", Fclean.shape)

        # free_raw = Fclean.copy()
        free_raw = Fclean

        # if self._is_jupyter:
        #     if verbose:
        #         # PLOT
        #         clean_plot = Fclean[:, wIDX["Cr"]:wIDX["Tz"]]
        #         fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)

        #         # jutils.scatter_2d(ax1, wiped_plot, self.xyMinMax, "Fwiped")
        #         jutils.scatter_2d(ax1, wiped_plot, [-1, 1, -1, 1], "Fwiped")
        #         if clean_plot.shape[0]:
        #             jutils.scatter_2d(ax2, clean_plot, self.xyMinMax, "Fclean")
        #             # jutils.scatter_2d(
        #             #     ax2,
        #             #     clean_plot,
        #             #     [7, 9, 7, 9],
        #             #     "Fclean",
        #             # )
        #         for p in Fmax:
        #             cr = p[IDX["Y"]]
        #             x = p[IDX["TCP_X"]]
        #             y = p[IDX["TCP_Y"]]
        #             m = p[IDX["M"]]
        #             ax1.add_artist(plt.Circle((x, y), radius=0.2 / 100.0, color="red", fill=False))
        #             # # RED DOTS
        #             # if clean_plot.shape[0]:
        #             #     ax2.add_artist(
        #             #         plt.Circle((x, y), radius=0.2 / 100.0, color="red", fill=False)
        #             #     )

        #         max_manip = np.max(Fclean[:, wIDX["M"]])
        #         mask = np.equal(Fclean[:, wIDX["M"]], max_manip)
        #         candidates = Fclean[mask]
        #         for p in candidates:
        #             ct, cr, x, y, m = p[:wIDX["Tz"]]
        #             ax2.add_artist(plt.Circle(
        #                 (x, y),
        #                 linewidth=2,
        #                 radius=1.0 / 100.0,
        #                 color="red",
        #                 fill=True,
        #             ))
        #         for collision in Obs:
        #             xs, ys = collision.vertices
        #             xs = np.append(xs, xs[0]) - self.target_center[0]
        #             ys = np.append(ys, ys[0]) - self.target_center[1]
        #             ax1.plot(xs, ys)
        #             # ax2.plot(xs, ys)
        #             xs, ys = collision.offsets
        #             xs = np.append(xs, xs[0]) - self.target_center[0]
        #             ys = np.append(ys, ys[0]) - self.target_center[1]
        #             ax1.plot(xs, ys)
        #             # ax2.plot(xs, ys)
        #         fig.tight_layout()
        #         plt.show()
        # else:
        if not self._is_jupyter:
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

        if self._is_jupyter and verbose:
            print("num: ", len(candidates))
            # print("Fclean[:%d]:\n" % num, free_raw[:num])
            # print("Fclean[:%d] (sorted):\n" % num, Fsort[:num])
            # PLOT
            clean_plot = free_raw[:, wIDX["Cr"]:wIDX["Tz"]]
            fig, (ax1, ax2) = plt.subplots(nrows=1, ncols=2)
            jutils.scatter_2d(ax1, clean_plot, self.xyMinMax, "All candidates")
            for p in candidates:
                ct, cr, x, y, m = p[:wIDX["Tz"]]
                ax1.add_artist(plt.Circle(
                    (x, y),
                    linewidth=2,
                    radius=0.5 / 100.0,
                    color="red",
                    fill=False,
                ))
            fig.tight_layout()
            plt.show()

        # candidates
        _xys = self.xy_correction(candidates[:, wIDX["Bx"]:wIDX["M"]])
        candidates[:, wIDX["Bx"]:wIDX["M"]] = _xys

        if not self._is_jupyter:
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
        print("\t                (r: %.3f m, p: %.3f m, y: %.3f m) based on [base_footprint]" %
              (EEProll, EEPpitch, EEPyaw))
        j_string = ["%.1f" % deg for deg in np.degrees(J)]
        print("\t   Joint (deg): ", j_string)

    def calc(self, Pt, Obs, Cr, Ct, section_def, is_right_hand, verbose=False):
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
        if is_right_hand:
            minCr = np.degrees(Cr[0])
            maxCr = np.degrees(Cr[1])
            Fcut = self.get_Fcut(minCr, maxCr, verbose)
        else:
            # mirror to right
            minCr = -np.degrees(Cr[1])
            maxCr = -np.degrees(Cr[0])
            Fcut = self.get_Fcut(minCr, maxCr, verbose)
            # back to left
            Fcut[:, IDX["TCP_Y"]] *= -1.0

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

        ###################################
        # 4. Fclean
        ###################################
        Fclean = self.get_Fclean(Obs, Fwiped, verbose)

        # candidates
        candidates = self.get_candidates(Fclean, num=-1)
        return candidates
