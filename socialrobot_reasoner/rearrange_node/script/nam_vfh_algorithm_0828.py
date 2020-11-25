# Author: Changjoo Nam (cjnam@kist.re.kr)
# Date: 07/22/2019
# Description: Planning for removing obstacles to grasp an unknown target object in clutter (A certain target in a known and static environment)

# Input:
# robot_height (double): the height of robot view point (i.e., camera's z coordinate)
# robot_pose (double list): the 2D coordinates of the robot (and the camera)
# target_id (int): ID of the target object (-9999 if unknown)

# N (int): the number of all objects including the target
# R (double list): the radii of objects in meters (e.g., R = [0.025, 0.027, ...])
# H (double list): the heights of objects in meters (e.g., H = [0.07, 0.08, ...])
# X (double list): the x coordinates of objects in meters (e.g., X = [0.10, 0.51, ...])
# Y (double list): the y coordinates of objects in meters (e.g., Y = [0.21, 0.12, ...])

# x_min (double): the minimum x coordinate of the environment
# x_max (double): the maximum x coordinate of the environment
# y_min (double): the minimum y coordinate of the environment
# y_max (double): the maximum y coordinate of the environment

# Output:
# accessibility (int): the accessibility of the target object (-1=unaccessible, 0=undetected, 1=accessible)
# relocate_id (int): the ID of the object to be relocated (e.g., 2)
# relocate_coordinates (double list): the (x, y) coordinates of the object to be relocated (e.g., [0.02, 0.05])


"""
# Example input
robot_height = 0.075
robot_pose = [0.32299999999999995, -0.06575]
target_id = 8

N = 10
R = [0.025, 0.026, 0.03, 0.028, 0.026, 0.025, 0.03, 0.03, 0.029, 0.025, 0.03]
H = [0.073, 0.071, 0.068, 0.07, 0.071, 0.069, 0.07, 0.073, 0.066, 0.066, 0.075]
X = [0.475, 0.306, 0.475, 0.36405000000000004, 0.306, 0.17099999999999999, 0.43005000000000004, 0.20405, 0.20900000000000002, 0.17099999999999999]
Y = [0.19475, 0.14075000000000001, 0.36975, 0.26625, 0.40975000000000006, 0.16075, 0.27425, 0.28725, 0.04225, 0.38375000000000004]

x_min =0.06299999999999999
x_max =0.583
y_min =-0.06575
y_max =0.51775

# Example run
import relocate_planner as rp
[accessibility, relocate_id, relocate_coordinates] = rp.relocate_planner(robot_height, robot_pose, target_id, N, R, H, X, Y, x_min, x_max, y_min, y_max)
print('Target accessibility (-1=unaccessible, 0=undetected, 1=accessible): %d' % accessibility)
print('Relocate Object %d at (%f, %f)' % (relocate_id, relocate_coordinates[0], relocate_coordinates[1]))
"""


def relocate_planner(robot_height, robot_pose, target_id, N, R, H, X, Y, x_min, x_max, y_min, y_max):
    import sys
    import numpy as np
    import random as rn
    import networkx as nx
    import VFHplus_mobile as vfh
    import matplotlib
    import matplotlib.pyplot as plt

    # import VFHplus_change_radius as vfh
    # distance(x, y): compute the Euclidean distance between 2D points x and y
    def distance(point_one, point_two):
        return ((point_one[0] - point_two[0]) ** 2 +
                (point_one[1] - point_two[1]) ** 2) ** 0.5

    # unique_list(sequence): remove duplicated elements in the input list 'sequence'
    def unique_list(seq):
        seen = set()
        seen_add = seen.add
        return [x for x in seq if not (x in seen or seen_add(x))]

    def invisible_volume(object_pose, object_height, camera_pose, camera_height, radius, mean_r):
        d = distance(object_pose, camera_pose)
        l = (object_height * (d + 2 * radius)) / (camera_height - object_height)
        x = (2 * radius * (d + 2 * radius + l)) / (d + mean_r)
        y = (2 * radius * (d + 2 * radius)) / d
        z = (((x - y) ** 2) / 2 + l ** 2) ** 0.5
        return object_height * (x * z - y * z + l * y) / 2

    edges_add = []
    edges_all = []
    objects = []
    walls = []

    min_len = 10000
    min_weight = min_len * 10

    mean_r = np.mean(R)
    robot_radius = max(R)
    R.append(robot_radius)

    x_min = x_min-4*mean_r
    y_min = y_min-4*mean_r
    x_max = x_max+4*mean_r
    y_max = y_max+4*mean_r

    # Generating bounding box (Wall)
    X_t = list(np.linspace(x_min, x_max, int(np.ceil((x_max - x_min)/(4*mean_r)))))
    Y_t = list(np.linspace(y_min, y_max, int(np.ceil((y_max - y_min)/(4*mean_r)))))

    X_w = X_t + [x_max]*len(Y_t)
    X_w = X_w + X_t
    Y_w = [y_max]*len(X_t) + Y_t
    Y_w = Y_w + [y_min]*len(X_t)

    M = len(X_w)
    R_wall = [2*np.sqrt(mean_r**2)] * M
    for i in range(0, M):
        walls.append([X_w[i], Y_w[i]])

    for i in range(0, N):
        objects.append([X[i], Y[i]])
    objects.append(robot_pose)

    # Create an empty graph
    G = nx.Graph()
    # N+1 because the robot base pose is also a node
    all_nodes = list(range(0, N+1))
    G.add_nodes_from(all_nodes)

    # Connect edges of the graph using VFH+
    #  Description: for each pair of node i and node j, check if an edge (i, j) can be connected between them (i is not equal to j)
    #               Note that (i, j) and (j, i) are different (directed edges a.k.a. "arrows")
    #               (i ,j) connected?: the end-effector can move any object from Object i's pose to Object j's pose without collision (if Objects i and j are removed)
    nodes_done = []
    for i in all_nodes:
        # Exclude myself (node i) and previously checked nodes
        nodes_wo_me = list(set(all_nodes) - set([i]) - set(nodes_done))
        nodes_done.append(i)
        for j in nodes_wo_me:
            # end_pose (the end-effector pose):
            # the end-effector is located at node j's pose (picking from node j's location-> go to i's location)
            end_pose = objects[j]
            edge = (i, j) #A path i to j: any object can move from i to j and j to i
            # VFH+ checks within d_max (between Object i and the end-effector)
            d_max = distance(objects[i], end_pose)
            # d_max = distance(objects[i], end_pose)+0.09
            # Except i and j, other objects are regarded as obstacles
            obstacles_sub = list(set(all_nodes) - set([i, j]) - set([N]))
            # radius (to compute the augmented radius): choose the largest radius among all objects because any object in the scene should be able to move between i and j without collision
            radius = max(R)
            # Run VFH+ (in VFHplus_mobile.py): the wall is included as obstacles
            if i == target_id and j == N:
                _, _, _, collision_free = vfh.influence(len(all_nodes) - 2 + M - 1, objects[i], [objects[k] for k in obstacles_sub] + walls, end_pose, d_max, [R[k] for k in obstacles_sub] + R_wall, radius, 1)
                # if collision_free == 1:
                #     edges_add.append(edge)
                if collision_free == 1:
                   _, _, _, collision_free_r = vfh.influence(len(all_nodes) - 2 + M - 1, objects[j], [objects[k] for k in obstacles_sub] + walls, objects[i], d_max, [R[k] for k in obstacles_sub] + R_wall, radius, 1)
                   if collision_free_r == 1:
                       edges_add.append(edge)
            else:
                _, _, _, collision_free = vfh.influence(len(all_nodes) - 2 + M - 1, objects[i], [objects[k] for k in obstacles_sub] + walls, end_pose, d_max, [R[k] for k in obstacles_sub] + R_wall, radius, 0)
                # if collision_free == 1:
                #     edges_add.append(edge)
                if collision_free == 1:
                   _, _, _, collision_free_r = vfh.influence(len(all_nodes) - 2 + M - 1, objects[j], [objects[k] for k in obstacles_sub] + walls, objects[i], d_max, [R[k] for k in obstacles_sub] + R_wall, radius, 0)
                   if collision_free_r == 1:
                       edges_add.append(edge)
    G.add_edges_from(edges_add)

    # Find accessibile objects
    #  Description: the nodes connected to the robot node can be accessed by the robot since there are paths between the robot node and its neighbors
    all_sources = [N]#robot node
    accessible_nodes = list(G.neighbors(N))
    accessible_nodes.sort()

    # print 'Accessible: ', accessible_nodes
    # print 'Target: ', target_id

    if target_id < 0:
        uncovered_volume = []
        for j in range(0, len(accessible_nodes)):
            uncovered_volume.append(
                invisible_volume(objects[accessible_nodes[j]], H[accessible_nodes[j]], robot_pose, robot_height, R[accessible_nodes[j]], mean_r))
        node_next = accessible_nodes[uncovered_volume.index(max(uncovered_volume))]
        path = [node_next]
        accessibility = 0
    else:
        # Find the min-hop path bewteen each pair of a visible (accessible) object and the target
        for source in all_sources:
            if nx.has_path(G, source, target_id):
                # Find all min-hop paths (all ties)
                paths = list(nx.all_shortest_paths(G, source, target_id, weight=None))
                path_weights = [0] * len(paths)
                # Compute the Euclidean distance of each path for tie breaking
                for i in range(0, len(paths)):
                    for j in range(0, len(paths[i]) - 1):
                        path_weights[i] = path_weights[i] + distance(objects[paths[i][j]], objects[paths[i][j + 1]])

                # Choose the path with the minimum Euclidean distance if there are multiple min-hop paths
                idx = path_weights.index(min(path_weights))
                if len(paths[idx]) < min_len:
                    path = paths[idx]
                    min_len = len(path)
                    min_weight = min(path_weights)
                elif len(paths[idx]) == min_len and min(path_weights) < min_weight:
                    path = paths[idx]
                    min_len = len(path)
                    min_weight = min(path_weights)
        # If there is a single path found (min_len remains 10000 if no path found), add it to the path explored until now
        if min_len < 10000:
            path.pop(0)  # remove the robot node from the path
        else:
            path = [-1]
            # sys.exit('No path found to the target')

        if path[0] == target_id:
            accessibility = 1
        else:
            accessibility = -1

    relocate_id = path[0]
    relocate_coordinates = objects[relocate_id]

    # Print
    #print('Target accessibility (0=unaccessible, 1=accessible): %d' % accessibility)
    #print('Relocate Object %d at (%f, %f)' % (relocate_id, relocate_coordinates[0], relocate_coordinates[1]))
    #
    # if 0:
    #     # Plot the final graph
    #     plt.figure(2)
    #
    #     # Plot the configuration (Fig. 8a in the paper)
    #     plt.subplot(121)
    #
    #     plt.gcf()
    #     ax = plt.gca()
    #
    #     all_nodes = list(range(0, N))
    #     all_obstacles = list(set(all_nodes) - set([target_id]))
    #     for i in all_obstacles:
    #         obstacle_plot = plt.Circle((objects[i][0], objects[i][1]), R[i], color='r', clip_on=False)
    #         ax.add_artist(obstacle_plot)
    #
    #     #source_plot = plt.Circle((objects[path[0]][0], objects[path[0]][1]), R[path[0]], color='gray', clip_on=False)
    #     #ax.add_artist(source_plot)
    #
    #     target_plot = plt.Circle((objects[target_id][0], objects[target_id][1]), R[target_id], color='lime', clip_on=False)
    #     ax.add_artist(target_plot)
    #
    #     for i in range(0, M):
    #         wall_plot = matplotlib.patches.Rectangle((walls[i][0] - 0.04, walls[i][1] - 0.04), 0.08, 0.08, color='sienna', clip_on=False)
    #         ax.add_patch(wall_plot)
    #
    #     plt.plot()
    #     plt.axis('scaled')
    #
    #     plt.xlim(min(X_w) - max(R), max(X_w) + max(R))
    #     plt.ylim(min(Y_w) - max(R), max(Y_w) + max(R))
    #
    #     # Plot the graph (Fig. 8b in the paper)
    #     plt.subplot(122)
    #
    #     pos = {}
    #     for i in all_nodes:
    #         pos[all_nodes[i]] = objects[i]
    #
    #     labels = {}
    #     for node_name in all_nodes:
    #         labels[node_name] = str(node_name)
    #
    #     nodes = nx.draw_networkx_nodes(G, pos, all_nodes, node_color='r', node_size=500)
    #     nodes_t = nx.draw_networkx_nodes(G, pos, nodelist=[target_id], node_color='lime', node_size=500)
    #     #if path:
    #     #    nodes_s = nx.draw_networkx_nodes(G, pos, nodelist=[path[0]], node_color='gray', node_size=500)
    #     #    nodes_s.set_edgecolor('black')
    #     nodes.set_edgecolor('black')
    #     nodes_t.set_edgecolor('black')
    #     nx.draw_networkx_labels(G, pos, labels, font_size=20)
    #
    #     path_edges = []
    #     for i in range(0, len(path) - 1):
    #         path_edges.append((path[i], path[i + 1]))
    #     nx.draw_networkx_edges(G, pos, edgelist=edges_add, width=0.5, arrowsize=20)
    #     nx.draw_networkx_edges(G, pos, edgelist=path_edges, width=2, edge_color='red', arrowsize=20)
    #
    #     plt.xlim(min(X) - 2 * max(R), max(X) + 2 * max(R))
    #     plt.ylim(min(Y) - 2 * max(R), max(Y) + 2 * max(R))
    #     plt.axis('off')
    #     plt.axis('scaled')
    #     plt.plot()
    #
    #     while True:
    #         try:
    #             plt.show()
    #         except UnicodeDecodeError:
    #             continue
    #         break

    return [accessibility, relocate_id, relocate_coordinates], path


def g_ore(in_tar_pos, in_obs_pos, in_tar_r, in_obs_r, in_rob_pos, in_ws_zero, in_ws_wd):
    import copy
    import timeit

    # print "start"
    # start = timeit.default_timer()
    robot_height = 0.075
    robot_pose = in_rob_pos
    target_id = len(in_tar_pos) + len(in_obs_pos) - 1 - 1
    N = len(in_tar_pos)+len(in_obs_pos) - 1
    # print "\n\nN", N
    R = copy.deepcopy(in_obs_r)
    R.append(in_tar_r)

    H = []
    for i in range(N):
        H.append(0.075)

    X = []
    Y = []

    for i in range(len(in_obs_pos)):
        X.append(in_obs_pos[i][0])
        Y.append(in_obs_pos[i][1])

    X.append(in_tar_pos[0])
    Y.append(in_tar_pos[1])

    x_min = in_ws_zero[0]
    x_max = in_ws_zero[0] + in_ws_wd[0]
    y_min = in_ws_zero[1]
    y_max = in_ws_zero[1] + in_ws_wd[1]
    #
    # print "\nrobot_pose", robot_pose
    # print "\ntarget_id", target_id
    # print "\nN, R, H", N, R, H
    # print "\nX, Y", X, Y
    # print "ws zero", in_ws_zero, "width, depth", in_ws_wd
    # print "\nx_min, x_max, y_min, y_max", x_min, x_max, y_min, y_max

    _, g_order = relocate_planner(robot_height, robot_pose, target_id, N, R, H, X, Y, x_min, x_max, y_min, y_max)
    g_order = list(g_order)
    if g_order[0] == -1:
        # stop = timeit.default_timer()
        # print "time", stop - start
        # print "no path"
        return g_order
    g_order.pop()
    g_order.append('T')
    # print "find order:", g_order

    # stop = timeit.default_timer()
    # print "time", stop - start
    return g_order
#
# if __name__ == '__main__':
#     # Example input
#     robot_height = 0.075
#     robot_pose = [-0.1, 0.2]#[0.32299999999999995, -0.06575]
#     target_id = 0
#
#     N = 10
#     R = [0.025, 0.026, 0.03, 0.028, 0.026, 0.025, 0.03, 0.03, 0.029, 0.025]
#     H = [0.073, 0.071, 0.068, 0.07, 0.071, 0.069, 0.07, 0.073, 0.066, 0.066]
#     X = [0.475, 0.306, 0.475, 0.36405000000000004, 0.306, 0.17099999999999999, 0.43005000000000004, 0.20405, 0.20900000000000002, 0.17099999999999999]
#     Y = [0.19475, 0.14075000000000001, 0.36975, 0.26625, 0.40975000000000006, 0.16075, 0.27425, 0.28725, 0.04225, 0.38375000000000004]
#
#     x_min = 0.06299999999999999
#     x_max = 0.583
#     y_min = -0.06575
#     y_max = 0.51775
#
#     # Example run
#     # import relocate_planner as rp
#     [accessibility, relocate_id, relocate_coordinates], ret_path = relocate_planner(robot_height, robot_pose, target_id, N, R, H, X, Y, x_min, x_max, y_min, y_max)
#     print "path:", ret_path
#
#     # self.d_max = 2.0
#     # tm_tar_pos = copy.deepcopy(self.tar_pos)
#     # tm_tar_ori = [0.0, 0.0, 0.0]
#     # tm_obs_pos = copy.deepcopy(self.obs_pos)
#     # tm_obs_pos.extend(self.obs_wall)
#     # ob = len(tm_obs_pos)
#     # tm_obs_ori = []
#     # for i in range(ob):
#     #     tm_obs_ori.append([0.0, 0.0, 0.0])
#
#     # ore_order = TM_noplot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, self.rob_pos, self.rob_pos, self.d_max)
#     # ore_order = G_ore(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, self.rob_pos, self.rob_pos, self.d_max)
#
#
#
#
#
#
#
#
#
#     import matplotlib.pyplot as plt
#
#     plt.figure()
#     plt.scatter(X, Y, c='red', s=200, alpha=0.5, edgecolors='none')
#     plt.scatter(X[target_id], Y[target_id], c='blue', s=200, alpha=0.5, edgecolors='none')
#     plt.scatter(robot_pose[0], robot_pose[1], c='gray', s=200, alpha=0.5, edgecolors='none')
#
#     plt.show()
#     # print('Target accessibility (-1=unaccessible, 0=undetected, 1=accessible): %d' % accessibility)
#     # print('Relocate Object %d at (%f, %f)' % (relocate_id, relocate_coordinates[0], relocate_coordinates[1]))