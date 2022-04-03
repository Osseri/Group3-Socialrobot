#!/usr/bin/env python
import rospy
import rospkg
from tf import transformations
import numpy as np
from scipy.spatial import ConvexHull
from geometry_msgs import msg as geo_msg
from nav_msgs import msg as nav_msg
from pure_hybrid_astar.srv import PureHybridAstar, PureHybridAstarResponse
from planner_src import hybrid_astar


def XYTs_to_navPath(xyrad_list, frame_id):
    nav_path = nav_msg.Path()
    nav_path.header.frame_id = frame_id
    for x, y, rad in xyrad_list:
        ps = geo_msg.PoseStamped()
        ps.header.frame_id = frame_id
        ps.pose.position.x = x
        ps.pose.position.y = y
        quat = transformations.quaternion_about_axis(rad, (0, 0, 1))
        ps.pose.orientation.x = quat[0]
        ps.pose.orientation.y = quat[1]
        ps.pose.orientation.z = quat[2]
        ps.pose.orientation.w = quat[3]
        nav_path.poses.append(ps)
    return nav_path


def bb3d_to_2d_convexhull(bb3d):
    # 2d projection of 8 vertices
    T = transformations.quaternion_matrix((
        bb3d.center.orientation.x,
        bb3d.center.orientation.y,
        bb3d.center.orientation.z,
        bb3d.center.orientation.w,
    ))
    T[:3, 3] = (
        bb3d.center.position.x,
        bb3d.center.position.y,
        bb3d.center.position.z,
    )
    # 8 points    
    dx = bb3d.size.x / 2.0
    dy = bb3d.size.y / 2.0
    dz = bb3d.size.z / 2.0
    vertices = np.array([
        [ dx,  dy,  dz, 1.0],
        [ dx,  dy, -dz, 1.0],
        [ dx, -dy,  dz, 1.0],
        [ dx, -dy, -dz, 1.0],
        [-dx,  dy,  dz, 1.0],
        [-dx,  dy, -dz, 1.0],
        [-dx, -dy,  dz, 1.0],
        [-dx, -dy, -dz, 1.0],
    ]).transpose()
    vertices = np.dot(T, vertices)
    projection = vertices[:2].transpose()  # x, y
    # naive convexhull
    return ConvexHull(projection)


def get_points_of_2d_convexhull(convexhull, min_interval=0.05):
    points = [v for v in convexhull.points[convexhull.vertices]]
    for sidx, gidx in convexhull.simplices:
        spoint = convexhull.points[sidx]
        gpoint = convexhull.points[gidx]
        v = gpoint - spoint
        v_length = np.linalg.norm(v)
        interval_vector = v * (min_interval / v_length)
        while v_length > min_interval:
            spoint += interval_vector
            points.append(np.array(spoint))
            v_length = np.linalg.norm(gpoint - spoint)
    return points


class MainProcess:
    def __init__(self):
        # Grid configuration
        grid_dxy_meter = 0.15
        grid_dtheta_deg = 12

        self.solver = hybrid_astar.HybridAstar(
            grid_dxy_meter,
            np.radians(grid_dtheta_deg),
        )
        # Service
        rospy.Service("/pure_hybrid_astar/find_path", PureHybridAstar, self.cb_process)

    def cb_process(self, req):
        # Successor
        self.solver.reset_successors(req.motion_model, req.backward_motion)
        rospy.logwarn("successor: %d" % len(self.solver.successors))

        # Constraints
        r_width = req.robot_width_meter
        r_radius = r_width / np.sqrt(2)
        ws_limit = (req.ws_xmin, req.ws_xmax, req.ws_ymin, req.ws_ymax)
        self.solver.set_collision_model(req.safety_radius, r_radius, r_width, ws_limit)

        # vision_msgs/BoundingBox3D req.obstacles
        # TODO: nav_msgs/OccupancyGrid req.occupancy_map
        point_list = []
        for bb3d in req.obstacles:
            chull = bb3d_to_2d_convexhull(bb3d)
            point_list += get_points_of_2d_convexhull(chull)
        self.solver.set_obstacles(point_list)

        path_length, r_xyrs = self.solver.compute_path(
            (0.0, 0.0, 0.0),                            # r_start
            (req.goal.x, req.goal.y, req.goal.theta),   # r_goal
            req.ignore_goal_orientation,                # ig
            req.timeout_in_sec,                         # computation limit
        )
        if path_length:
            rospy.loginfo("Hybrid A* Done! path length = %d" % path_length)
            #print(r_xyrs)
        else:
            rospy.logerr("Something wrong! path_length = 0")

        resp = PureHybridAstarResponse()
        resp.path_length = path_length
        resp.robot_path = XYTs_to_navPath(r_xyrs, "base_footprint")
        return resp


if __name__ == "__main__":
    rospy.init_node("pusher_node")
    p = MainProcess()
    rospy.spin()
