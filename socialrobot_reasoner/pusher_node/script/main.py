#!/usr/bin/env python
import rospy
import rospkg
from tf import transformations
import numpy as np
from geometry_msgs import msg as geo_msg
from nav_msgs import msg as nav_msg
from pusher_node.srv import PlanePushing, PlanePushingRequest, PlanePushingResponse
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


def motion_model_name(motion_model):
    names = {
        PlanePushingRequest.WEDGE: 'wedge',
        PlanePushingRequest.FULL_CIRCLE: 'full_circle',
        PlanePushingRequest.SM_CIRCLE: 'sm_circle',
        PlanePushingRequest.OMNI: 'omni',
    }
    return names[motion_model]


class MainProcess:
    def __init__(self):
        # Grid configuration
        # grid_dxy_meter = 0.15
        # grid_dtheta_deg = 12
        grid_dxy_meter = 0.10
        grid_dtheta_deg = 15

        self.solver = hybrid_astar.HybridAstar(
            grid_dxy_meter,
            np.radians(grid_dtheta_deg),
            (-4.0, 4.0, -4.0, 4.0),
        )
        self.solver.set_successors(
            enable_wedge=True,
            enable_full_circle=False,
            enable_sm_circle=False,
            enable_omni=True,
        )  # Default is wedge

        # Service
        rospy.Service("/pusher_node/find_path", PlanePushing, self.cb_process)

    def cb_process(self, req):
        # --------------------------------------------------
        new_model_name = motion_model_name(req.motion_model)
        if self.solver.last_model_name != new_model_name:
            rospy.loginfo(">> Reset successors: {} -> {}".format(
                self.solver.last_model_name, new_model_name))
            self.solver.reset_successors(motion_model_name=new_model_name)
            rospy.loginfo("Successor size: {}".format(len(self.solver.successors)))
        else:
            rospy.loginfo(">> No change in motion model: {}".format(new_model_name))
        # --------------------------------------------------

        o_width = req.object_width_meter
        r_width = req.robot_width_meter
        o_radius = o_width / np.sqrt(2)
        r_radius = r_width / np.sqrt(2)
        self.solver.set_collision_model(o_radius, r_radius, o_width, r_width)

        # geometry_msgs/Polygon
        self.solver.set_obstacles([(p.x, p.y) for p in req.obstacles.points])

        o_start = (req.object_start.x, req.object_start.y, req.object_start.theta)
        o_goal = (req.object_goal.x, req.object_goal.y, req.object_goal.theta)
        ig = req.ignore_goal_orientation
        path_length, o_xyrs, r_xyrs = self.solver.compute_path(o_start, o_goal, ig)
        if path_length:
            rospy.loginfo("Hybrid A* Done! path length = %d" % path_length)
        else:
            rospy.logerr("Something wrong! path_length = 0")

        resp = PlanePushingResponse()
        resp.path_length = path_length
        resp.object_path = XYTs_to_navPath(o_xyrs, "base_footprint")
        # print(r_xyrs)
        resp.robot_path = XYTs_to_navPath(r_xyrs, "base_footprint")
        return resp


if __name__ == "__main__":
    rospy.init_node("pusher_node")
    p = MainProcess()
    rospy.spin()
