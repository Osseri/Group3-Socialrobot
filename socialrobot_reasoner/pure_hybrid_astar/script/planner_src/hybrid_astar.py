import heapq
import time
import numpy as np
from scipy.spatial import cKDTree
import rospy
from pure_hybrid_astar.srv import PureHybridAstarRequest as PHR


class Collision:
    def __init__(self, radius, width):
        self.radius = radius
        self.width = width


class Node:
    def __init__(self, real_xyt, grid_index):
        """
        real_xyt: m, m, rad
        """
        self._real_xyt = real_xyt
        self._grid_index = grid_index

        # came from
        self.parent_index = None
        self.steering_type = None  # +:ccw, -:cw

        self._g_score = np.inf
        self._h_score = 0.0
        self._f_score = self._g_score + self._h_score

    @property
    def xyt(self):
        return self._real_xyt

    @property
    def xy(self):
        return (self._real_xyt[0], self._real_xyt[1])

    @property
    def rad(self):
        return self._real_xyt[2]

    @property
    def index(self):
        return self._grid_index

    @property
    def g(self):
        return self._g_score

    @property
    def h(self):
        return self._h_score

    @property
    def f(self):
        return self._f_score

    def set_xyt(self, xyt):
        self._real_xyt = xyt

    def set_score(self, g, h):
        self._g_score = g
        self._h_score = h
        self._f_score = self._g_score + self._h_score

    def __repr__(self):
        x, y, rad = self._real_xyt
        deg = np.degrees(rad)
        return "\n<Node({:.2f},{:.2f},{:.0f}) parent[{}] f{:.4f}=g{:.4f}+h{:.4f}>".format(x, y, deg, self.index, self.f, self.g, self.h)

    # # Customs for GridMap
    # def __hash__(self):
    #     return self._grid_xyt

    # Customs for OpenList
    # (<, <=, >, >=, == and !=) can be overloaded by providing definition
    #  to __lt__, __le__, __gt__, __ge__, __eq__ and __ne__ magic methods.
    def __lt__(self, other):
        return self.f < other.f


class Grid:
    def __init__(self, dxy_meter, dtheta_radian):
        self.dxy_meter_float = dxy_meter
        self.dtheta_deg_int = int(np.rint(np.degrees(dtheta_radian)))
        if 360 % self.dtheta_deg_int:
            raise ValueError("360 % np.degrees(dtheta) must be 0.")
        self._data = {}

    def find_grid_index(self, real_xyt):
        x_index = int(np.rint(real_xyt[0] / self.dxy_meter_float))
        y_index = int(np.rint(real_xyt[1] / self.dxy_meter_float))
        real_deg = np.degrees(real_xyt[2])
        t_index = int(np.rint(real_deg % 360) / self.dtheta_deg_int)
        return (x_index, y_index, t_index)

    def find_real_xyt(self, grid_index):
        x = grid_index[0] * self.dxy_meter_float
        y = grid_index[1] * self.dxy_meter_float
        t = np.radians(grid_index[2] * self.dtheta_deg_int)
        return (x, y, t)

    def get_node_from_index(self, index):
        return self._data[index]

    def get_node(self, xyt):
        index = self.find_grid_index(xyt)
        if index in self._data:
            return self._data[index]
        else:
            n = Node(xyt, index)
            self._data[index] = n
            return n


class HybridAstar:
    def __init__(self, dxy_meter, dtheta_radian):
        self.grid_xy_size = dxy_meter
        self.min_drad = dtheta_radian
        # print("=== dxy_meter, drad = {:.2f}, {:.1f}".format(dxy_meter, dtheta_radian))
        self.grid = Grid(self.grid_xy_size, self.min_drad)
        self.min_succesor_dist = dxy_meter * np.sqrt(2)

        # obstacle
        self.kdtree = None
        # successor [(type, successor)]
        self.successors = []
        # collision
        self.workspace = None
        self.object_collision = None
        self.robot_collision = None

        self.conditions = {
            "circle_obstacles": False,
            "successor": False,
            "collision_model": False,
        }

    def set_obstacles(self, point_list):
        obstacles = np.array(point_list if point_list else [(-100, -100)])
        # TODO: Filter with grid index
        self.kdtree = cKDTree(obstacles)
        self.conditions["circle_obstacles"] = True

    def get_closest_distance(self, query_xy):
        distance, index = self.kdtree.query(query_xy)
        return distance

    def reset_successors(self, motion_model, backward):
        self.successors = []

        # self.min_drad is 12 deg
        preset = {
            # (name, max_heading_change, tilting)
            PHR.HOLONOMIC: ("HOLONOMIC", np.radians(85.), np.radians(85.)),
            PHR.MONOCYCLE: ("MONOCYCLE", np.radians(170.), np.radians(0.)),
            PHR.BICYCLE: ("BICYCLE", np.radians(37.), np.radians(0.)),
        }
        if motion_model not in preset:
            rospy.logerr("Undefiend motion model!")
            raise NotImplementedError
        
        model = preset[motion_model]
        rospy.logwarn("Detected model is: %s" % model[0])
        self.set_circle_successors(model[1], model[2], backward)
        self.conditions["successor"] = True

    def set_circle_successors(self, max_heading_change, tilting, backward):
        # diagonal_length: 15*root(2) = 21.2 cm /2 = 10.6
        _min_succesor_dist = self.min_succesor_dist
        _min_drad = self.min_drad  # 12 deg

        def calc_csucc(d_heading_rad):
            if d_heading_rad != 0.0:
                _r = _min_succesor_dist / 2.0
                dx = _r * (1.0 + np.cos(d_heading_rad))
                dy = _r * np.sin(d_heading_rad)
                dtheta = d_heading_rad
                icp_x = 0.0
                icp_y = _r / np.arctan(-d_heading_rad / 2.0)
                cost = abs(icp_y * d_heading_rad)
            else:
                dx = _min_succesor_dist
                dy = 0.0
                dtheta = 0.0
                cost = _min_succesor_dist
                icp_x = 0.0
                icp_y = np.inf
            return ((dx, dy), dtheta, cost, (icp_x, icp_y))

        # (dxy, dtheta, cost, icp_xy) [meter, rad]
        # backward
        if backward:
            dxy, dtheta, cost, icp_xy = calc_csucc(0.0)
            dxy = (-dxy[0], dxy[1])
            fw = (0, (dxy, dtheta, cost, icp_xy))
            self.successors.append(fw)

        drad_list = [(0, 0)]  # type, rad
        # type(+), rad(+): ccw
        _rad = _min_drad
        stype = 1
        while _rad < max_heading_change:
            drad_list.append((stype, _rad))
            drad_list.append((-stype, -_rad))
            stype += 1
            _rad += _min_drad

        for steering_type, rad in drad_list:
            dxy, dtheta, cost, icp_xy = calc_csucc(rad)
            fw = (steering_type, (dxy, dtheta, cost, icp_xy))
            self.successors.append(fw)

            _rad = _min_drad
            tilt_type = 1
            x, y = dxy
            ix, iy = icp_xy
            while _rad < tilting:
                c, s = np.cos(_rad), np.sin(_rad)
                tdxy = (c*x -s*y, s*x + c*y)
                ticp_xy = (c*ix -s*iy, s*ix + c*iy)
                fw = (steering_type + tilt_type, (tdxy, dtheta, cost, ticp_xy))
                self.successors.append(fw)

                c, s = np.cos(-_rad), np.sin(-_rad)
                tdxy = (c*x -s*y, s*x + c*y)
                fw = (steering_type + tilt_type, (tdxy, dtheta, cost, ticp_xy))
                self.successors.append(fw)

                tilt_type += 1
                _rad += _min_drad

    def get_successors(self, current_xy, current_rad):
        _c = np.cos(current_rad)
        _s = np.sin(current_rad)
        R = np.array([[_c, -_s], [_s, _c]])

        def transformation(local_xy):
            new_xy = np.dot(R, np.array(local_xy)) + np.array(current_xy)
            return new_xy

        def for_each(one_successor):
            dxy, dtheta, cost, icp_xy = one_successor
            new_xy = transformation(dxy)
            new_theta = current_rad + dtheta
            new_icp_xy = transformation(icp_xy)
            return (new_xy, new_theta, cost, new_icp_xy)

        successors = [(stype, for_each(s)) for stype, s in self.successors]
        return successors

    def set_collision_model(self, safety_radius, robot_radius, robot_width, ws_limit):        
        self.robot_collision = Collision(safety_radius + robot_radius, robot_width)
        self.workspace = ws_limit
        self.conditions["collision_model"] = True

    def is_not_in_workspace(self, xy):
        x, y = xy
        xmin, xmax, ymin, ymax = self.workspace
        return (x < xmin) or (x > xmax) or (y < ymin) or (y > ymax)

    def check_collision(self, robot_xyt):
        rb_xy = (robot_xyt[0], robot_xyt[1])
        robot_collide = self.get_closest_distance(rb_xy) < self.robot_collision.radius
        out_of_workspace = self.is_not_in_workspace(rb_xy)
        return (robot_collide or out_of_workspace)

    def compute_path(self, robot_start, robot_goal, ignore_goal_orientation, timeout):
        """
        robot_start: pose2d (x, y, heading) [m, m, rad]
        robot_goal: pose2d (x, y, heading) [m, m, rad]
        ignore_goal_orientation: bool
        ---
        path_length: int
        robot_path: (non-holonomic / holonomic)
        ============
        1. ignore_goal_orientation option
        2. minimum steering heuristic
        """
        if False in self.conditions.values():
            raise ValueError("Some conditions are not prepared.")

        self.grid = Grid(self.grid_xy_size, self.min_drad)

        def length_of_bezier(a, b, c, d):

            def bezier_point(t):
                f1 = a + t * (b - a)
                f2 = b + t * (c - b)
                f3 = c + t * (d - c)
                s1 = f1 + t * (f2 - f1)
                s2 = f2 + t * (f3 - f2)
                xy = s1 + t * (s2 - s1)
                return xy

            L = 0
            prev_xy = a
            for t in np.linspace(0, 1, 5):
                xy = bezier_point(t)
                L += np.linalg.norm(xy - prev_xy)
                prev_xy = xy
            return L

        def heuristic(query_xyt):
            qx, qy, qrad = query_xyt
            gx, gy, grad = robot_goal
            qxy = np.array((qx, qy))
            gxy = np.array((gx, gy))
            dist = np.linalg.norm(gxy - qxy)
            if ignore_goal_orientation:
                h_cost = dist
            else:
                rate = 0.2
                offset = rate * dist
                q2 = qxy + offset * np.array((np.cos(qrad), np.sin(qrad)))
                g2 = gxy - offset * np.array((np.cos(qrad), np.sin(qrad)))
                modified_dist = length_of_bezier(qxy, q2, g2, gxy)
                # IMPORTANT
                min_rotation_radius = 0.3
                drad = abs(qrad - grad)
                arc_length = min_rotation_radius * drad
                h_cost = modified_dist + arc_length
            return h_cost

        def steering_cost(prev_stype, successor_stype):
            if prev_stype is not None:
                dstype = abs(successor_stype - prev_stype)
                drad = dstype * self.min_drad
                pseudo_dy = drad * self.min_succesor_dist
                return pseudo_dy
            return 0.0

        # MAIN ALGORITHM
        start = self.grid.get_node(robot_start)
        start.set_score(0.0, heuristic(robot_start))
        open_heap = []
        heapq.heappush(open_heap, start)
        is_not_sorted = False

        goal_index = self.grid.find_grid_index(robot_goal)
        begin_time = time.time()
        print("computing...")
        while open_heap:
            elapsed = time.time() - begin_time
            if elapsed > timeout:
                break
            if is_not_sorted:
                heapq.heapify(open_heap)
                is_not_sorted = False
            current = heapq.heappop(open_heap)

            # print("\ncur:{}\top:{}".format(current, open_heap))

            if (current.index[0] == goal_index[0]) and (current.index[1] == goal_index[1]):
                if ignore_goal_orientation:
                    return self.reconstruct_path(current)
                elif (current.index[2] == goal_index[2]):
                    return self.reconstruct_path(current)

            # [new(xy, theta, cost, icp_xy), ...] [meter, rad]
            successors = self.get_successors(current.xy, current.rad)
            for stype, succ in successors:
                xy, rad, edge_weight, icp = succ
                xyt = (xy[0], xy[1], rad)

                # Collision check
                is_safe = not self.check_collision(xyt)
                if is_safe:
                    neighbor = self.grid.get_node(xyt)

                    steering_weight = steering_cost(current.steering_type, stype)
                    # print(">>{}".format(steering_weight))
                    tentative_g = current.g + edge_weight + steering_weight
                    # print("xyt:{:.3f}, {:.3f}, {:.3f}deg {}".format(xy[0], xy[1], np.degrees(rad), rad))
                    # print("ne-:{}, ten-g:{}".format(neighbor, tentative_g))

                    if tentative_g < neighbor.g:
                        neighbor.set_xyt(xyt)
                        neighbor.parent_index = current.index
                        neighbor.steering_type = stype
                        neighbor.set_score(tentative_g, heuristic(xyt))
                        if neighbor not in open_heap:
                            heapq.heappush(open_heap, neighbor)
                        else:
                            is_not_sorted = True
                        # print("ne-:{}, ten-g:{}".format(neighbor, tentative_g))
                        # print("op:{}".format(open_heap))
        # path_length, object[xyt,...], robot[xyt,...]
        return (0, [], [])

    # def calc_robot_xyt(self, object_xyt):
    #     offset = (self.object_collision.width + self.robot_collision.width) / 2.0
    #     rt = object_xyt[2]
    #     rx = object_xyt[0] - offset * np.cos(rt)
    #     ry = object_xyt[1] - offset * np.sin(rt)
    #     return (rx, ry, rt)

    def reconstruct_path(self, last_node):
        reverse_path = [last_node]
        node = last_node
        while node.parent_index is not None:
            node = self.grid.get_node_from_index(node.parent_index)
            reverse_path.append(node)
        reverse_path.reverse()
        path = reverse_path

        path_length = len(path)
        robot_path = [n.xyt for n in path]

        return (path_length, robot_path)


if __name__ == "__main__":
    pass
