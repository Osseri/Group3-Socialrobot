#!/usr/bin/env python
def relocate_planner(data):
    import sys
    import numpy as np
    import random as rn
    import networkx as nx
    import VFHplus_mobile as vfh

    robot_height = data.robot_height
    robot_pose = list(data.robot_pose)
    target_id = data.target_id
    N = data.N
    R = list(data.R)
    H = list(data.H)
    X = list(data.X)
    Y = list(data.Y)
    x_min = data.x_min
    x_max = data.x_max
    y_min = data.y_min
    y_max = data.y_max

    # robot_height, robot_pose, target_id, N, R, H, X, Y, x_min, x_max, y_min, y_max
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

    # Generating bounding box (Wall)
    X_t = list(np.linspace(x_min, x_max, int(np.ceil((x_max - x_min)/(2*mean_r)))))
    Y_t = list(np.linspace(y_min, y_max, int(np.ceil((y_max - y_min)/(2*mean_r)))))

    X_w = [x_min]*len(Y_t) + X_t
    X_w = X_w + [x_max]*len(Y_t)
    Y_w = Y_t + [y_max]*len(X_t)
    Y_w = Y_w + Y_t

    M = len(X_w)
    R_wall = [np.sqrt(mean_r**2 + mean_r**2)] * M
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
            # Except i and j, other objects are regarded as obstacles
            obstacles_sub = list(set(all_nodes) - set([i, j]) - set([N]))
            # radius (to compute the augmented radius): choose the largest radius among all objects because any object in the scene should be able to move between i and j without collision
            radius = max(R)
            # Run VFH+ (in VFHplus_mobile.py): the wall is included as obstacles
            if i == target_id and j == N:
                _, _, _, collision_free = vfh.influence(len(all_nodes) - 2 + M - 1, objects[i], [objects[k] for k in obstacles_sub] + walls, end_pose, d_max, [R[k] for k in obstacles_sub] + R_wall, radius, 1)
                if collision_free == 1:
                    _, _, _, collision_free_r = vfh.influence(len(all_nodes) - 2 + M - 1, objects[j], [objects[k] for k in obstacles_sub] + walls, objects[i], d_max, [R[k] for k in obstacles_sub] + R_wall, radius, 1)
                    if collision_free_r == 1:
                        edges_add.append(edge)
            else:
                _, _, _, collision_free = vfh.influence(len(all_nodes) - 2 + M - 1, objects[i], [objects[k] for k in obstacles_sub] + walls, end_pose, d_max, [R[k] for k in obstacles_sub] + R_wall, radius, 0)
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
            sys.exit('No path found to the target')

        if path[0] == target_id:
            accessibility = 1
        else:
            accessibility = -1

    relocate_id = path[0]
    relocate_coordinates = objects[relocate_id]

    # Print
    #print('Target accessibility (0=unaccessible, 1=accessible): %d' % accessibility)
    #print('Relocate Object %d at (%f, %f)' % (relocate_id, relocate_coordinates[0], relocate_coordinates[1]))
    return [accessibility, relocate_id, relocate_coordinates]

def listener():
    rospy.Service('relocation_srv', relocate_env_srv, relocate_planner)

    rospy.spin()
if __name__== '__main__':
    import rospy
    from relocation_node.srv._relocate_env_srv import *

    print "============================"
    print "===relocation node starts==="
    print "============================"
    rospy.init_node('relocate_planner_node', anonymous=True)

    listener()