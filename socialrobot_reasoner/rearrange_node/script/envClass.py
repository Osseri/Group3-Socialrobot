import custom_function as CUF
from VFHplus_change_radius import influence
# from VFHplus_mobile import influence
# from tree_making_no_plot import tree_making as TM_noplot
# from tree_making_no_plot2 import tree_making as TM_noplot
# from tree_making_plot import tree_making as TM_plot
from nam_vfh_algorithm_0828 import g_ore as NG_ore
import client_function as CLF

import numpy as np
import copy
import sensor_msgs
import std_msgs
import moveit_msgs
import math
import time
from tf.transformations import quaternion_from_euler
import matplotlib.pyplot as plt


GRID_SIZE = 0.01

class EnvInfo:

    def __init__(self, rob_pos, ws_width, ws_depth, ws_cen, grid_size, wall_r):
        self.rob_pos = rob_pos
        self.GRID_SIZE = grid_size
        self.ws_w = ws_width
        self.ws_d = ws_depth
        self.ws_cen = ws_cen
        self.ws_zero = [round(self.ws_cen[0] - ws_width * self.GRID_SIZE * 0.5, 2), round(self.ws_cen[1] - ws_depth * self.GRID_SIZE * 0.5, 2)]
        self.obs_wall = self.get_obs_wall(OBJ_R=wall_r)

        self.jaco_home_pos = [0.020, -0.0046, 0.673]
        self.jaco_home_ori = [-0.1869, 0.125044, 0.69344, 0.6845144]

        # goal_pose = [-0.010, -0.005, 0.967]
        # goal_orientation = [-0.1869, 0.125044, 0.69344, 0.6845144]
        self.order_error_flag = 1
        self.d_max = 2.0
        self.eta = 30
        self.l = 0.02
        self.n_path_try_acc = 3
        self.n_path_try_bt = 3
        self.n_path_try_picknplace = 3

        grid_act = np.zeros([ws_width, ws_depth])
        self.grid_act = CUF.mark_edge_grid(grid_act)

    def set_env(self, obstacle_name, obstacle_info, target_name, target_info):
        self.obs_r = []
        self.obs_pos = []
        self.obs_ori = []
        self.obs_grid = []
        self.tar_pos = [round(target_info[0][0][0], 2), round(target_info[0][0][1], 2)]
        # self.tar_pos = [round(target_info[0][0][2], 2)+0.01, round(-target_info[0][0][1], 2)-0.01]
        # self.tar_pos = [round(target_info[0][0][2], 2) + 0.05, round(-target_info[0][0][1], 2) - 0.03]

        self.tar_grid = [int((target_info[0][0][0] - self.ws_zero[0]) * 100), int((target_info[0][0][1] - self.ws_zero[1]) * 100)]
        # self.tar_grid = [int(round((target_info[0][0][2] - self.ws_zero[0]) * 100)), int(round((-target_info[0][0][1] - self.ws_zero[1]) * 100))]
        self.tar_r = round(target_info[0][2][0] * 0.5, 2)
        # print "number of obstacles", len(obstacle_info)
        for i in range(len(obstacle_info)):
            self.obs_pos.append([round(obstacle_info[i][0][0], 2), round(obstacle_info[i][0][1], 2)])
            self.obs_grid.append([int((obstacle_info[i][0][0] - self.ws_zero[0])*100), int((obstacle_info[i][0][1] - self.ws_zero[1])*100)])
            # self.obs_grid.append([int(round((obstacle_info[i][0][2] - self.ws_zero[0])*100)), int(round((-obstacle_info[i][0][1] - self.ws_zero[1])*100))])
            self.obs_r.append(round(obstacle_info[i][2][0] * 0.5, 2))
            self.obs_ori.append([0.0, 0.0, 0.0])

        self.object_z = obstacle_info[0][0][0]
        # print "obstacles", self.obs_pos
        # print "target_vfh", self.tar_pos
        # print "target_rviz", target_info[0]
        # print "robot pos", self.rob_pos

        tm_tar_pos = copy.deepcopy(self.tar_pos)
        tm_obs_pos = copy.deepcopy(self.obs_pos)

        self.ore_order = NG_ore(tm_tar_pos, tm_obs_pos, self.tar_r, self.obs_r, self.rob_pos, self.ws_zero, [self.ws_w * self.GRID_SIZE, self.ws_d * self.GRID_SIZE])

        self.ore_grid = []
        self.ore_pos = []
        self.ore_r = []

        self.obs_re_grid = copy.deepcopy(self.obs_grid)
        self.obs_re_pos = copy.deepcopy(self.obs_pos)
        self.obs_re_r = copy.deepcopy(self.obs_r)

        for i in self.ore_order:
            if i != 'T':
                self.ore_grid.append(self.obs_re_grid[i])
                self.ore_pos.append(self.obs_re_pos[i])
                self.ore_r.append(self.obs_re_r[i])
        for i in self.ore_order:
            if i != 'T':
                self.obs_re_grid.remove(self.obs_grid[i])
                self.obs_re_pos.remove(self.obs_pos[i])
                self.obs_re_r.remove(self.obs_r[i])

        self.grid_ori = copy.deepcopy(self.grid_act)
        for i in range(len(self.obs_r)):
            self.grid_ori = CUF.obstacle_circle(self.grid_ori, [round(self.obs_grid[i][0], 2), round(self.obs_grid[i][1], 2), self.obs_r[i]], 2)
        self.grid_ori = CUF.obstacle_circle(self.grid_ori, [self.tar_grid[0], self.tar_grid[1], self.tar_r], 4)  # target

        self.grid_del = copy.deepcopy(self.grid_act)
        for i in range(len(self.obs_re_r)):
            self.grid_del = CUF.obstacle_circle(self.grid_del, [round(self.obs_re_grid[i][0], 2), round(self.obs_re_grid[i][1], 2), self.obs_re_r[i]], 2)
        self.grid_del = CUF.obstacle_circle(self.grid_del, [self.tar_grid[0], self.tar_grid[1], self.tar_r], 4)  # target
        self.ore_order.pop()

    def get_env(self, obs_r, tar_r, min_ore):
        while 1:
            self.obs_grid = []
            grid_tmp = copy.deepcopy(self.grid_act)
            self.obs_r = obs_r
            self.tar_r = tar_r
            for ri in self.obs_r:
                grid_tmp, obs_center_tmp = CUF.place_circle_object_ig(grid_tmp, ri, 2)
                self.obs_grid.append(obs_center_tmp)
            grid_tmp, tar_tmp = CUF.place_circle_object_ig(grid_tmp, self.tar_r, 4)
            self.tar_grid = copy.deepcopy(tar_tmp)

            self.obs_pos = []
            for i in self.obs_grid:
                xi, yi = i
                self.obs_pos.append([round(xi * self.GRID_SIZE + self.ws_zero[0], 2), round(yi * self.GRID_SIZE + self.ws_zero[1], 2)])
            self.tar_pos = [round(self.tar_grid[0] * self.GRID_SIZE + self.ws_zero[0], 2), round(self.tar_grid[1] * self.GRID_SIZE + self.ws_zero[1], 2)]  # target object!

            self.d_max = 2.0
            tm_tar_pos = copy.deepcopy(self.tar_pos)
            tm_tar_ori = [0.0, 0.0, 0.0]
            tm_obs_pos = copy.deepcopy(self.obs_pos)
            # tm_obs_pos.extend(self.obs_wall)
            ob = len(tm_obs_pos)
            tm_obs_ori = []
            for i in range(ob):
                tm_obs_ori.append([0.0, 0.0, 0.0])

            ore_order = NG_ore(tm_tar_pos, tm_obs_pos, self.tar_r, self.obs_r, self.rob_pos, self.ws_zero, [self.ws_w * self.GRID_SIZE, self.ws_d * self.GRID_SIZE])
            # ore_order = TM_noplot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, self.rob_pos, self.rob_pos, self.d_max)
            # ore_order = TM_noplot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, self.obs_wall, self.rob_pos, self.rob_pos, self.d_max)
            # TM_noplot()
            if len(ore_order) > min_ore:
                # print"before rearrangemet: ", ore_order
                tm_tar_pos = copy.deepcopy(self.tar_pos)
                tm_obs_pos = copy.deepcopy(self.obs_pos)
                # tm_obs_pos.extend(self.obs_wall)
                # ob = len(tm_obs_pos)
                self.ore_order = ore_order
                # self.ore_order = TM_plot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, self.rob_pos, self.rob_pos, self.d_max)

                # check if the obstacles are rearranged then target is reachable
                tm_tar_pos = copy.deepcopy(self.tar_pos)
                tm_obs_pos = copy.deepcopy(self.obs_pos)
                tm_ore_pos = []
                for i in self.ore_order:
                    if i != 'T':
                        tm_obs_pos.remove(self.obs_pos[i])
                # tm_obs_pos.extend(self.obs_wall)
                ob = len(tm_obs_pos)
                tmp_order = NG_ore(tm_tar_pos, tm_obs_pos, self.tar_r, self.obs_r, self.rob_pos, self.ws_zero, [self.ws_w * self.GRID_SIZE, self.ws_d * self.GRID_SIZE])
                # tmp_order = TM_noplot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, self.obs_wall,self.rob_pos, self.rob_pos, self.d_max)
                # print "after removing:", tmp_order
                if tmp_order[0] == 'T':
                    # print"environment setting OK"
                    break
            # else:
                # print "SHORT...min:", min_ore, "ours:", len(ore_order)
        '''
        with out additional rearrangement
        '''
        if len(self.ore_order) > min_ore:
            self.ore_grid = []
            self.ore_pos = []
            self.ore_r = []

            self.obs_re_grid = copy.deepcopy(self.obs_grid)
            self.obs_re_pos = copy.deepcopy(self.obs_pos)
            self.obs_re_r = copy.deepcopy(self.obs_r)

            for i in self.ore_order:
                if i != 'T':
                    self.ore_grid.append(self.obs_re_grid[i])
                    self.ore_pos.append(self.obs_re_pos[i])
                    self.ore_r.append(self.obs_re_r[i])
            for i in self.ore_order:
                if i != 'T':
                    self.obs_re_grid.remove(self.obs_grid[i])
                    self.obs_re_pos.remove(self.obs_pos[i])
                    self.obs_re_r.remove(self.obs_r[i])

            self.grid_ori = copy.deepcopy(self.grid_act)
            for i in range(len(self.obs_r)):
                self.grid_ori = CUF.obstacle_circle(self.grid_ori, [round(self.obs_grid[i][0], 2), round(self.obs_grid[i][1], 2), self.obs_r[i]], 2)
            self.grid_ori = CUF.obstacle_circle(self.grid_ori, [self.tar_grid[0], self.tar_grid[1], self.tar_r], 4)  # target

            self.grid_del = copy.deepcopy(self.grid_act)
            for i in range(len(self.obs_re_r)):
                self.grid_del = CUF.obstacle_circle(self.grid_del, [round(self.obs_re_grid[i][0], 2), round(self.obs_re_grid[i][1], 2), self.obs_re_r[i]], 2)
            self.grid_del = CUF.obstacle_circle(self.grid_del, [self.tar_grid[0], self.tar_grid[1], self.tar_r], 4)  # target
            self.ore_order.pop()

    def update_env(self, in_obs_pos, in_obs_grid):
        self.d_max = 2.0
        tm_tar_pos = copy.deepcopy(self.tar_pos)
        tm_tar_ori = [0.0, 0.0, 0.0]
        tm_obs_pos = copy.deepcopy(in_obs_pos)
        # tm_obs_pos.extend(self.obs_wall)
        ob = len(tm_obs_pos)
        tm_obs_ori = []
        for i in range(ob):
            tm_obs_ori.append([0.0, 0.0, 0.0])

        self.ore_order = NG_ore(tm_tar_pos, tm_obs_pos, self.tar_r, self.obs_r, self.rob_pos, self.ws_zero, [self.ws_w * self.GRID_SIZE, self.ws_d * self.GRID_SIZE])
        while 1:
            tm_tar_pos = copy.deepcopy(self.tar_pos)
            tm_obs_pos = copy.deepcopy(self.obs_pos)
            tm_ore_pos = []
            for i in self.ore_order:
                if i != 'T':
                    tm_obs_pos[i] = [5.0, 0.0]
                    # tm_obs_pos.remove(self.obs_pos[i])
            # tm_obs_pos.extend(self.obs_wall)

            tmp_order = NG_ore(tm_tar_pos, tm_obs_pos, self.tar_r, self.obs_r, self.rob_pos, self.ws_zero, [self.ws_w * self.GRID_SIZE, self.ws_d * self.GRID_SIZE])
            # tmp_order = TM_noplot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, self.obs_wall,self.rob_pos, self.rob_pos, self.d_max)
            # print "after removing:", tmp_order
            if tmp_order[0] != 'T':
                # print "tricky environment so extend", self.ore_order
                self.ore_order.pop()
                # print "delete target", self.ore_order
                self.ore_order.extend(tmp_order)
                # print "to", self.ore_order
            else:
                if tmp_order[0] == 'T':
                    # print "ok", self.ore_order
                    break

        self.ore_grid = []
        self.ore_pos = []
        self.ore_r = []

        self.obs_re_grid = copy.deepcopy(in_obs_grid)
        self.obs_re_pos = copy.deepcopy(in_obs_pos)
        self.obs_re_r = copy.deepcopy(self.obs_r)

        for i in self.ore_order:
            if i != 'T':
                self.ore_grid.append(self.obs_re_grid[i])
                self.ore_pos.append(self.obs_re_pos[i])
                self.ore_r.append(self.obs_re_r[i])
        for i in self.ore_order:
            if i != 'T':
                self.obs_re_grid.remove(self.obs_grid[i])
                self.obs_re_pos.remove(self.obs_pos[i])
                self.obs_re_r.remove(self.obs_r[i])

        self.grid_ori = copy.deepcopy(self.grid_act)
        for i in range(len(self.obs_r)):
            self.grid_ori = CUF.obstacle_circle(self.grid_ori, [round(self.obs_grid[i][0], 2), round(self.obs_grid[i][1], 2), self.obs_r[i]], 2)
        self.grid_ori = CUF.obstacle_circle(self.grid_ori, [self.tar_grid[0], self.tar_grid[1], self.tar_r], 4)  # target

        self.grid_del = copy.deepcopy(self.grid_act)
        for i in range(len(self.obs_re_r)):
            self.grid_del = CUF.obstacle_circle(self.grid_del, [round(self.obs_re_grid[i][0], 2), round(self.obs_re_grid[i][1], 2), self.obs_re_r[i]], 2)
        self.grid_del = CUF.obstacle_circle(self.grid_del, [self.tar_grid[0], self.tar_grid[1], self.tar_r], 4)  # target
        self.ore_order.pop()
        # else:
        #     self.order_error_flag = 0

    def get_max_can(self, input_grid, bt_num, trial_num, obstacle_diameter, padding):
        bt_circle = []
        can_grid = []
        # circle_r = max(self.ore_r)+0.02
        circle_r = obstacle_diameter*0.5 + padding
        # circle_r = max(self.ore_r)+0.055
        # circle_r = max(self.ore_r)+0.045
        for bt in range(bt_num):
            grid_can = copy.deepcopy(input_grid)  # get original scene from the grid_set
            empt_grid, occu_grid = CUF.getEmpOcc(grid_can)
            for i in range(trial_num):
                pick_cen = np.random.randint(0, len(empt_grid))
                check_sum = 0
                for oc in range(len(occu_grid)):
                    d_w = empt_grid[pick_cen][0] - occu_grid[oc][0]
                    d_d = empt_grid[pick_cen][1] - occu_grid[oc][1]
                    d_c = (d_w * d_w + d_d * d_d) ** 0.5 * self.GRID_SIZE
                    if d_c <= circle_r:
                        check_sum = 1

                if check_sum == 0:
                    can_grid.append(empt_grid[pick_cen])
                    for em in range(len(empt_grid)):
                        d_w = empt_grid[pick_cen][0] - empt_grid[em][0]
                        d_d = empt_grid[pick_cen][1] - empt_grid[em][1]
                        d_c = (d_w * d_w + d_d * d_d) ** 0.5 * self.GRID_SIZE
                        if d_c <= circle_r:
                            grid_can[empt_grid[em][0]][empt_grid[em][1]] = 3
                            grid_can[empt_grid[pick_cen][0]][empt_grid[pick_cen][1]] = 3
                            occu_grid.append([empt_grid[em][0], empt_grid[em][1]])
            bt_circle.append([can_grid, grid_can])

        max_cir_num = []
        for i in range(len(bt_circle)):
            max_cir_num.append([len(bt_circle[i][0])])

        # print(max_cir_num.index(max(max_cir_num)))
        max_trial = max_cir_num.index(max(max_cir_num))

        self.grid_max_can = copy.deepcopy(bt_circle[max_trial][1])
        self.can_grid = bt_circle[max_trial][0]
        self.can_pos = []
        for i in self.can_grid:
            xi, yi = i
            self.can_pos.append([self.ws_zero[0] + xi * self.GRID_SIZE, self.ws_zero[1] + yi * self.GRID_SIZE])

    def get_obs_wall(self, OBJ_R):
        ws_side = []
        ws_side.append(
            [self.ws_cen[0] - self.ws_w * self.GRID_SIZE * 0.5, self.ws_cen[1] - self.ws_d * self.GRID_SIZE * 0.5 - OBJ_R])  # left low point
        ws_side.append([self.ws_cen[0] + self.ws_w * self.GRID_SIZE * 0.5 + OBJ_R,
                        self.ws_cen[1] - self.ws_d * self.GRID_SIZE * 0.5 - OBJ_R])  # right low point
        ws_side.append([self.ws_cen[0] + self.ws_w * self.GRID_SIZE * 0.5 + OBJ_R,
                        self.ws_cen[1] + self.ws_d * self.GRID_SIZE * 0.5 + OBJ_R])  # right high point
        ws_side.append(
            [self.ws_cen[0] - self.ws_w * self.GRID_SIZE * 0.5, self.ws_cen[1] + self.ws_d * self.GRID_SIZE * 0.5 + OBJ_R])  # left high point

        obs_wall = []
        # obs_wall.extend(CUF.linspace2D(ws_side[0], ws_side[1], round(self.ws_w * self.GRID_SIZE / OBJ_R)))
        # obs_wall.extend(CUF.linspace2D(ws_side[1], ws_side[2], round(self.ws_d * self.GRID_SIZE / OBJ_R)))
        # obs_wall.extend(CUF.linspace2D(ws_side[2], ws_side[3], round(self.ws_w * self.GRID_SIZE / OBJ_R)))
        return obs_wall

    def get_can_info(self, in_can_info, in_obs_pos, in_obs_re_pos, in_ore_order, in_tar_pos):
        tmp_can_info = []
        for i in range(len(in_ore_order)):
            tmp_can_info.append(copy.deepcopy(in_can_info))
        tmp_obs_pos = copy.deepcopy(in_obs_pos)
        tmp_obs_re_pos = copy.deepcopy(in_obs_re_pos)
        tmp_ore_order = copy.deepcopy(in_ore_order)
        tmp_tar_pos = copy.deepcopy(in_tar_pos)
        # print("\nCheck if candidate blocks the target")
        for step_i in range(len(tmp_ore_order)):
            for i in range(len(tmp_can_info[step_i])):
                vfh_obs_pos = copy.deepcopy(tmp_obs_re_pos)
                vfh_obs_pos.append(tmp_can_info[step_i][i].pos)
                vfh_obs_pos.extend(self.obs_wall)
                vfh_tar_pos = copy.deepcopy(tmp_tar_pos)
                ob = len(vfh_obs_pos)
                vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta, obstacle_r=self.obs_r)
                # vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.obs_r, self.tar_r)
                if vfh[3] == 0:
                    tmp_can_info[step_i][i].BT = 1  # BT == 1 : The candidate blocks the target.
                else:                       # BT == 0 : The candidate does not block the target.
                    tmp_can_info[step_i][i].BT = 0

            # print("\nCheck if the candidate is accessible.")
            for i in range(len(tmp_can_info[step_i])):
                vfh_tar_pos = copy.deepcopy(tmp_can_info[step_i][i].pos)
                vfh_obs_pos = copy.deepcopy(tmp_obs_pos)
                for si in range(step_i+1):
                    # print "\nstep", si, "\nbefore", vfh_obs_pos
                    vfh_obs_pos.remove(tmp_obs_pos[tmp_ore_order[si]])
                    # print "after", vfh_obs_pos
                vfh_obs_pos.append(tmp_tar_pos)
                vfh_obs_pos.extend(self.obs_wall)
                ob = len(vfh_obs_pos)
                vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta, obstacle_r=self.obs_r)
                if vfh[3] == 0:                      # A == 1 : The candidate is accessible.
                    tmp_can_info[step_i][i].A = 0    # A == 0 : The candidate is not accessible.
                else:                   #
                    tmp_can_info[step_i][i].A = 1

            # print("\nCheck the candidate ORC.")
            for i in range(len(tmp_can_info[step_i])):
                vfh_tar_pos = copy.deepcopy(tmp_can_info[step_i][i].pos)
                vfh_obs_pos = copy.deepcopy(tmp_obs_pos)
                vfh_obs_pos.append(tmp_tar_pos)
                vfh_obs_pos.extend(self.obs_wall)
                ob = len(vfh_obs_pos)
                vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta, obstacle_r=self.obs_r)
                if vfh[3] == 0:              # A == 1 : The candidate is accessible.
                    tmp_can_info[step_i][i].A = 0    # A == 0 : The candidate is not accessible.
                    tm_tar_pos = copy.deepcopy(vfh_tar_pos)
                    tm_tar_ori = [0.0, 0.0, 0.0]
                    tm_obs_pos = copy.deepcopy(tmp_obs_pos)
                    # tm_obs_pos.extend(self.obs_wall)
                    ob = len(tm_obs_pos)
                    tm_obs_ori = []
                    for obs_ori_i in range(ob):
                        tm_obs_ori.append([0.0, 0.0, 0.0])

                    ore_order = NG_ore(tm_tar_pos, tm_obs_pos, self.tar_r, self.obs_r, self.rob_pos, self.ws_zero, [self.ws_w * self.GRID_SIZE, self.ws_d * self.GRID_SIZE])
                    # ore_order = TM_noplot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, self.rob_pos, self.rob_pos, self.d_max)
                    ore_order.pop()  # The last order is always the target so we need to pop the last element.
                    tmp_can_info[step_i][i].ORC = ore_order
                else:                   #
                    tmp_can_info[step_i][i].A = 1

        return tmp_can_info

    def get_can_A(self, in_can_info, in_obs_pos, in_tar_pos, obj_diameter):
        tmp_can_info = copy.deepcopy(in_can_info)
        # print("\nCheck if the candidate is accessible.")
        for ci in range(len(tmp_can_info)):
            vfh_tar_pos = copy.deepcopy(tmp_can_info[ci].pos)
            vfh_obs_pos = copy.deepcopy(in_obs_pos)
            vfh_obs_pos.append(copy.deepcopy(in_tar_pos))
            vfh_obs_pos.extend(self.obs_wall)
            ob = len(vfh_obs_pos)
            vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta, obstacle_r=self.obs_r)
            if vfh[3] == 0:  # A == 1 : The candidate is accessible.
                # print "\tF : can", ci, "A = 0 (not accessible)"
                tmp_can_info[ci].A = 0  # A == 0 : The candidate is not accessible.
            else:  #
                # print "\tF : can", ci, "A = 1 (accessible)"
                tmp_can_info[ci].A = 1  # A == 0 : The candidate is not accessible.
                # print "\nangle:", vfh[-1]

                # only in MP version
                # xi, yi = tmp_can_info[ci].pos[0], tmp_can_info[ci].pos[1]
                # CLF.add_box_client('can_check', [self.object_z, -yi, xi], [-0.707, 0.0, -0.707, 0.0], [obj_diameter, obj_diameter, 0.12], 'pink')
                #
                # planner_name = 'RRTConnect'
                # # planner_name = 'BiTRRT'
                # n_attempt = 10
                # c_time = 0.1
                # n_repeat = 1
                # start_state = moveit_msgs.msg.RobotState()
                # joint_state = sensor_msgs.msg.JointState()
                # joint_state.header = std_msgs.msg.Header()
                # joint_state.name = ['j2n6s300_joint_1', 'j2n6s300_joint_2', 'j2n6s300_joint_3', 'j2n6s300_joint_4', 'j2n6s300_joint_5', 'j2n6s300_joint_6']
                # joint_state.position = [3.1415927410125732, 4.537856101989746, 5.93411922454834, -0.6108652353286743, 1.7453292608261108, -0.5235987901687622]
                # start_state.joint_state = joint_state
                # # goal_pose:
                # # goal_orientation:
                # # goal_pose = [self.object_z, -yi, xi - 0.05]
                # # goal_orientation = [-0.00145713772037, -0.998970756926, 0.0364956710831, 0.0268955302573]
                # z = self.object_z
                # # goal_pose = [z + 0.7, -yi, xi]
                # # Set the grasp pose: substract 17cm from the z value of the object centroid
                # goal_pitches = []
                # goal_pitch = np.deg2rad(vfh[-1])
                # # goal_pitch = vfh[-1] + math.pi/2
                # goal_pitches.append(goal_pitch)  # approaching_angle: vfh[-1] from the input
                # for i in range(self.n_path_try_acc):
                #     goal_pitches.append(goal_pitch + (i + 1) * (math.pi / 36))
                #     goal_pitches.append(goal_pitch - (i + 1) * (math.pi / 36))
                # # Get the grasp orientation (currently the front direction)
                # goal_orientations = []
                # for i in goal_pitches:
                #     goal_orientations.append(quaternion_from_euler(-i, 0, math.radians(90.0), axes='rxyz'))
                # l = self.l # 2020.0212
                # # l = 0.001
                # goal_poses = []
                # for i in goal_pitches:
                #     dx = math.sin(i - math.pi) * l
                #     dy = math.cos(i - math.pi) * l
                #     goal_poses.append([z, -yi + dx, xi + dy])
                #     # goal_poses.append([z + 0.07, goal_pose[1] + dx, goal_pose[2] - dy])
                #
                # # plt.figure()
                # # for i in range(len(in_obs_pos)):
                # #     plt.scatter(in_obs_pos[i][0], in_obs_pos[i][1], s=200, c='red')
                # # plt.scatter(xi, yi, s=200, c='pink')
                # # plt.scatter(xi + dy, yi + dx, s=200, c='blue')
                # #
                # # plt.figure()
                # # for i in range(len(in_obs_pos)):
                # #     plt.scatter(in_obs_pos[i][0], in_obs_pos[i][1], s=200, c='red')
                # # plt.scatter(xi, yi, s=200, c='pink')
                # # plt.scatter(xi + dy, yi - dx, s=200, c='blue')
                # #
                # # plt.show()
                #
                # feasibility1 = 0
                # i = 0
                # while not feasibility1 and i < len(goal_pitches):
                #     # CLF.add_box_client('can_bottom_x', [goal_poses[i][0]-0.15, goal_poses[i][1], goal_poses[i][2]], goal_orientations[i], [0.1, 0.005, 0.005], 'red')
                #     # CLF.add_box_client('can_bottom_y', [goal_poses[i][0]-0.15, goal_poses[i][1], goal_poses[i][2]], goal_orientations[i], [0.005, 0.1, 0.005], 'blue')
                #     # CLF.add_box_client('can_bottom_z', [goal_poses[i][0]-0.15, goal_poses[i][1], goal_poses[i][2]], goal_orientations[i], [0.005, 0.005, 0.1], 'green')
                #     [feasibility1, trajectory1] = CLF.feasible_check_obj_joint_client('arm', 'gripper', start_state, goal_poses[i], goal_orientations[i], [], planner_name, n_attempt, c_time, n_repeat)
                #     i = i + 1
                #     # time.sleep(1)
                #     # CLF.del_box_client('can_bottom_x')
                #     # CLF.del_box_client('can_bottom_y')
                #     # CLF.del_box_client('can_bottom_z')
                # if feasibility1 == 0:  # A == 1 : The candidate is accessible.
                #     print "=> A = 0 (MP failed)"
                #     tmp_can_info[ci].A = 0  # A == 0 : The candidate is not accessible.
                # else:  #
                #     print "=> A = 1 (MP successed)"
                #     tmp_can_info[ci].A = 1
                # CLF.del_box_client('can_check')
                # # tmp_can_info[ci].A = 1
        return tmp_can_info

    def init_BT(self, in_can_info):
        for ci in in_can_info:
            ci.BT = 0
        return in_can_info

    def get_can_BT(self, in_can_info, in_obs_pos, in_tar_pos, obj_diameter):
        tmp_can_info = copy.deepcopy(in_can_info)
        # print("\nCheck if candidate blocks the target")
        for ci in range(len(tmp_can_info)):
            if tmp_can_info[ci].A == 1:
                if tmp_can_info[ci].BT == 0:
                    vfh_obs_pos = copy.deepcopy(in_obs_pos)
                    vfh_obs_pos.append(tmp_can_info[ci].pos)
                    vfh_obs_pos.extend(self.obs_wall)
                    vfh_tar_pos = copy.deepcopy(in_tar_pos)
                    ob = len(vfh_obs_pos)
                    vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta, obstacle_r=self.obs_r)
                    if vfh[3] == 0:
                        # print "\tF : can", ci, "BT = 1 (vfh not accessible)"
                        tmp_can_info[ci].BT = 1  # BT == 1 : The candidate blocks the target.
                    else:                        # BT == 0 : The candidate does not block the target.
                        # print "\tF : can", ci, "BT = 0 (vfh accessible)"
                        tmp_can_info[ci].BT = 0

                        # only in MP version
                        # print "\nangle:", vfh[-1]
            #             xi, yi = tmp_can_info[ci].pos[0], tmp_can_info[ci].pos[1]
            #             CLF.add_box_client('can_check', [self.object_z, -yi, xi], [-0.707, 0.0, -0.707, 0.0], [obj_diameter, obj_diameter, 0.12], 'pink')
            #
            #             planner_name = 'RRTConnect'
            #             # planner_name = 'BiTRRT'
            #             n_attempt = 10
            #             c_time = 0.1
            #             n_repeat = 1
            #             start_state = moveit_msgs.msg.RobotState()
            #             joint_state = sensor_msgs.msg.JointState()
            #             joint_state.header = std_msgs.msg.Header()
            #             joint_state.name = ['j2n6s300_joint_1', 'j2n6s300_joint_2', 'j2n6s300_joint_3', 'j2n6s300_joint_4', 'j2n6s300_joint_5', 'j2n6s300_joint_6']
            #             joint_state.position = [3.1415927410125732, 4.537856101989746, 5.93411922454834, -0.6108652353286743, 1.7453292608261108, -0.5235987901687622]
            #             start_state.joint_state = joint_state
            #             # goal_pose:
            #             # goal_orientation:
            #             # goal_pose = [self.object_z, -yi, xi - 0.05]
            #             # goal_orientation = [-0.00145713772037, -0.998970756926, 0.0364956710831, 0.0268955302573]
            #             z = self.object_z
            #
            #             # Set the grasp pose: substract 17cm from the z value of the object centroid
            #             goal_pitches = []
            #             goal_pitch = np.deg2rad(vfh[-1])
            #             # goal_pitch = vfh[-1] + math.pi/2
            #             goal_pitches.append(goal_pitch)  # approaching_angle: vfh[-1] from the input
            #             for i in range(self.n_path_try_bt):
            #                 goal_pitches.append(goal_pitch + (i + 1) * (math.pi / 36))
            #                 goal_pitches.append(goal_pitch - (i + 1) * (math.pi / 36))
            #             # Get the grasp orientation (currently the front direction)
            #             goal_orientations = []
            #             for i in goal_pitches:
            #                 goal_orientations.append(quaternion_from_euler(-i, 0, math.radians(90.0), axes='rxyz'))
            #             l = self.l # 2020.0212
            #             # l = 0.001
            #             goal_poses = []
            #             for i in goal_pitches:
            #                 dx = math.sin(i - math.pi) * l
            #                 dy = math.cos(i - math.pi) * l
            #                 goal_poses.append([z, -in_tar_pos[1] + dx, in_tar_pos[0] + dy])
            #                 # goal_poses.append([z + 0.07, goal_pose[1] + dx, goal_pose[2] - dy])
            #
            #             # plt.figure()
            #             # for i in range(len(in_obs_pos)):
            #             #     plt.scatter(in_obs_pos[i][0], in_obs_pos[i][1], s=200, c='red')
            #             # plt.scatter(xi, yi, s=200, c='pink')
            #             # plt.scatter(xi + dy, yi + dx, s=200, c='blue')
            #             #
            #             # plt.figure()
            #             # for i in range(len(in_obs_pos)):
            #             #     plt.scatter(in_obs_pos[i][0], in_obs_pos[i][1], s=200, c='red')
            #             # plt.scatter(xi, yi, s=200, c='pink')
            #             # plt.scatter(xi + dy, yi - dx, s=200, c='blue')
            #             #
            #             # plt.show()
            #
            #             feasibility1 = 0
            #             i = 0
            #             while not feasibility1 and i < len(goal_pitches):
            #                 # CLF.add_box_client('can_bottom_x', [goal_poses[i][0]-0.15, goal_poses[i][1], goal_poses[i][2]], goal_orientations[i], [0.1, 0.005, 0.005], 'red')
            #                 # CLF.add_box_client('can_bottom_y', [goal_poses[i][0]-0.15, goal_poses[i][1], goal_poses[i][2]], goal_orientations[i], [0.005, 0.1, 0.005], 'blue')
            #                 # CLF.add_box_client('can_bottom_z', [goal_poses[i][0]-0.15, goal_poses[i][1], goal_poses[i][2]], goal_orientations[i], [0.005, 0.005, 0.1], 'green')
            #                 [feasibility1, trajectory1] = CLF.feasible_check_obj_joint_client('arm', 'gripper', start_state, goal_poses[i], goal_orientations[i], [], planner_name, n_attempt, c_time, n_repeat)
            #                 i = i + 1
            #                 # time.sleep(1)
            #                 # CLF.del_box_client('can_bottom_x')
            #                 # CLF.del_box_client('can_bottom_y')
            #                 # CLF.del_box_client('can_bottom_z')
            #             if feasibility1 == 0:  # A == 1 : The candidate is accessible.
            #                 print "=> BT = 1 (MP failed)"
            #                 tmp_can_info[ci].BT = 1  # A == 0 : The candidate is not accessible.
            #             else:  #
            #                 print "=> BT = 0 (MP successed)"
            #                 tmp_can_info[ci].BT = 0
            #             CLF.del_box_client('can_check')
            # else:
            #     # print tmp_can_info[ci].A
            #     print "\tF : can", ci, "A = 0 => BT = no matter"

        return tmp_can_info

    def get_cf(self, in_can_info):
        tmp_cf = []
        tmp_cf_index = []
        tmp_can_info = copy.deepcopy(in_can_info)

        # print("\nCheck the candidate ORC.")
        for ci in range(len(tmp_can_info)):
            # print "\ncan ", ci, "th has A, BT :", tmp_can_info[ci].A, tmp_can_info[ci].BT
            if tmp_can_info[ci].A == 1 and tmp_can_info[ci].BT == 0:
                tmp_cf.append(tmp_can_info[ci])
                tmp_cf_index.append(ci)
        return tmp_cf, tmp_cf_index

    def get_cf_b(self, in_cf, in_obs_pos):
        tmp_cf = copy.deepcopy(in_cf)
        tmp_obs_pos = copy.deepcopy(in_obs_pos)
        tmp_b = []
        for cb in range(len(tmp_cf)):  # cb: The candidate that will check the b value
            b = 0
            for ci in range(len(tmp_cf)):  # ci: Other candidates for checking the b value
                if cb != ci:
                    # print "\ntar", tmp_cf_pos[ci]
                    # print "obs", tmp_obs_pos
                    vfh_tar_pos = copy.deepcopy(tmp_cf[ci].pos)
                    vfh_obs_pos = copy.deepcopy(tmp_obs_pos)
                    vfh_obs_pos.append(tmp_cf[cb].pos)
                    vfh_obs_pos.extend(self.obs_wall)
                    ob = len(vfh_obs_pos)
                    vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta, obstacle_r=self.obs_r)
                    if vfh[3] == 0:
                        b = b + 1
            tmp_b.append(b)
        return tmp_b

    def get_cp(self, in_can_info):
        tmp_cp = []
        tmp_cp_index = []
        tmp_can_info = copy.deepcopy(in_can_info)

        # print("\nCheck the candidate ORC.")
        for ci in range(len(tmp_can_info)):
            # print "\ncan ", ci, "th has A, BT :", tmp_can_info[ci].A, tmp_can_info[ci].BT
            if tmp_can_info[ci].A == 0:
                tmp_cp.append(tmp_can_info[ci])
                tmp_cp_index.append(ci)
        return tmp_cp, tmp_cp_index

    # def get_can_A(self, in_can_info, in_obs_pos, in_tar_pos):
    #     tmp_can_info = copy.deepcopy(in_can_info)
    #     # print("\nCheck if the candidate is accessible.")
    #     for ci in range(len(tmp_can_info)):
    #         vfh_tar_pos = copy.deepcopy(tmp_can_info[ci].pos)
    #         vfh_obs_pos = copy.deepcopy(in_obs_pos)
    #         vfh_obs_pos.append(copy.deepcopy(in_tar_pos))
    #         vfh_obs_pos.extend(self.obs_wall)
    #         ob = len(vfh_obs_pos)
    #         vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta)
    #         if vfh[3] == 0:  # A == 1 : The candidate is accessible.
    #             tmp_can_info[ci].A = 0  # A == 0 : The candidate is not accessible.
    #             # print "c:", ci, ".A = 0"
    #         else:  #
    #             tmp_can_info[ci].A = 1
    #             # print "c:", ci, ".A = 1"
    #     return tmp_can_info
    #
    # def get_can_BT(self, in_can_info, in_obs_pos, in_tar_pos):
    #     tmp_can_info = copy.deepcopy(in_can_info)
    #     # print("\nCheck if candidate blocks the target")
    #     for ci in range(len(tmp_can_info)):
    #         vfh_obs_pos = copy.deepcopy(in_obs_pos)
    #         vfh_obs_pos.append(tmp_can_info[ci].pos)
    #         vfh_obs_pos.extend(self.obs_wall)
    #         vfh_tar_pos = copy.deepcopy(in_tar_pos)
    #         ob = len(vfh_obs_pos)
    #         vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta)
    #         if vfh[3] == 0:
    #             tmp_can_info[ci].BT = 1  # BT == 1 : The candidate blocks the target.
    #             # print "c:", ci, ".BT = 1"
    #         # else:                        # BT == 0 : The candidate does not block the target.
    #         #     tmp_can_info[ci].BT = 0
    #         #     print "c:", ci, ".BT = 0"
    #
    #     return tmp_can_info

    def get_c_ore(self, in_can_info):
        # print "input", in_can_info
        # print in_can_info[0].pos
        t_c_order = []
        for ci in range(len(in_can_info)):
            tm_tar_pos = in_can_info[ci].pos
            tm_tar_ori = [0.0, 0.0, 0.0]
            tm_obs_pos = copy.deepcopy(self.obs_pos)
            tm_obs_pos.append(self.tar_pos)
            # tm_obs_pos.extend(self.obs_wall)
            tm_ob = len(tm_obs_pos)
            tm_obs_ori = []
            for i in range(tm_ob):
                tm_obs_ori.append([0.0, 0.0, 0.0])

            ore_order = NG_ore(tm_tar_pos, tm_obs_pos, self.tar_r, self.obs_r, self.rob_pos, self.ws_zero, [self.ws_w * self.GRID_SIZE, self.ws_d * self.GRID_SIZE])
            while 1:
                tm_tar_pos = in_can_info[ci].pos
                tm_obs_pos = copy.deepcopy(self.obs_pos)
                tm_obs_pos.append(self.tar_pos)
                obs_r = []
                for i in tm_obs_pos:
                    obs_r.append(0.035)
                for i in ore_order:
                    if i != 'T':
                        tm_obs_pos[i] = [4.0, 0.0]

                tmp_order = NG_ore(tm_tar_pos, tm_obs_pos, self.tar_r, obs_r, self.rob_pos, self.ws_zero, [self.ws_w * self.GRID_SIZE, self.ws_d * self.GRID_SIZE])
                # tmp_order = TM_noplot(ob, tm_tar_pos, tm_tar_ori, tm_obs_pos, tm_obs_ori, self.obs_wall,self.rob_pos, self.rob_pos, self.d_max)
                # print "after removing:", tmp_order
                if tmp_order[0] != 'T':
                    # print "not ok"
                    t_c_order.append([])
                    break
                    # if tmp_order[0] == -1:
                    #     print "no path"
                    #     t_c_order.append([])
                    #     break
                    # if len(ore_order) > len(self.obs_pos):
                    #     print "no path"
                    #     t_c_order.append([])
                    #     break
                    # print "tricky environment for c_ore so extend", ore_order
                    # ore_order.pop()
                    # print "delete target", ore_order
                    # ore_order.extend(tmp_order)
                    # print "to", ore_order
                else:
                    if tmp_order[0] == 'T':
                        # print "ok", ore_order
                        ore_order.pop()

                        if len(tm_obs_pos) in ore_order:
                            # print "\n\nThere is target!! warning!!!\n\n"
                            t_c_order.append([])
                        else:
                            t_c_order.append(ore_order)
                        break
        return t_c_order

    def pick(self, in_obs_pos, pick_pose, obs_to_pick):
        print "pick at:", pick_pose

        vfh_obs_pos = copy.deepcopy(in_obs_pos)
        try:
            vfh_obs_pos.remove(obs_to_pick)
        except:
            print
        vfh_obs_pos.extend(self.obs_wall)
        vfh_tar_pos = copy.deepcopy(pick_pose)
        ob = len(vfh_obs_pos)
        vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta)
        if vfh[3] == 1:
            print "\nangle:", vfh[-1]
            xi, yi = pick_pose[0], pick_pose[1]

            planner_name = 'RRTConnect'
            # planner_name = 'BiTRRT'
            n_attempt = 500
            c_time = 0.5
            n_repeat = 5
            start_state = moveit_msgs.msg.RobotState()
            joint_state = sensor_msgs.msg.JointState()
            joint_state.header = std_msgs.msg.Header()
            joint_state.name = ['j2n6s300_joint_1', 'j2n6s300_joint_2', 'j2n6s300_joint_3', 'j2n6s300_joint_4', 'j2n6s300_joint_5', 'j2n6s300_joint_6']
            # joint_state.position = current_joints
            joint_state.position = [3.1415927410125732, 4.537856101989746, 5.93411922454834, -0.6108652353286743, 1.7453292608261108, -0.5235987901687622]

            # joint_state.position = [3.1415927410125732, 4.537856101989746, 5.93411922454834, -0.6108652353286743, 1.7453292608261108, -0.5235987901687622]
            start_state.joint_state = joint_state
            # goal_pose:
            # goal_orientation:
            # goal_pose = [self.object_z, -yi, xi - 0.05]
            # goal_orientation = [-0.00145713772037, -0.998970756926, 0.0364956710831, 0.0268955302573]
            z = self.object_z
            print "\tF : object pick height", z
            # goal_pose = [z + 0.02, -yi, xi]
            # Set the grasp pose: substract 17cm from the z value of the object centroid
            goal_pitches = []
            goal_pitch = np.deg2rad(vfh[-1])
            # goal_pitch = vfh[-1] + math.pi/2
            goal_pitches.append(goal_pitch)  # approaching_angle: vfh[-1] from the input
            for i in range(self.n_path_try_picknplace):
                goal_pitches.append(goal_pitch + (i + 1) * (math.pi / 36))
                goal_pitches.append(goal_pitch - (i + 1) * (math.pi / 36))
            # Get the grasp orientation (currently the front direction)
            goal_orientations = []
            for i in goal_pitches:
                goal_orientations.append(quaternion_from_euler(-i, 0, math.radians(90.0), axes='rxyz'))
            l = self.l #= 0.035 # 2020.0212
            # l = 0.03 # 2020.0212
            # l = 0.015 # 2020.02.10
            # l = 0 # original
            # l = 0.17
            goal_poses = []
            for i in goal_pitches:
                dx = math.sin(i - math.pi) * l
                dy = math.cos(i - math.pi) * l
                goal_poses.append([z, -pick_pose[1] + dx, pick_pose[0] + dy])

            feasibility1 = 0
            i = 0
            while not feasibility1 and i < len(goal_pitches):
                # CLF.add_box_client('can_bottom_x', [goal_poses[i][0] - 0.15, goal_poses[i][1], goal_poses[i][2]], goal_orientations[i], [0.1, 0.005, 0.005], 'red')
                # CLF.add_box_client('can_bottom_y', [goal_poses[i][0] - 0.15, goal_poses[i][1], goal_poses[i][2]], goal_orientations[i], [0.005, 0.1, 0.005], 'blue')
                # CLF.add_box_client('can_bottom_z', [goal_poses[i][0] - 0.15, goal_poses[i][1], goal_poses[i][2]], goal_orientations[i], [0.005, 0.005, 0.1], 'green')
                [feasibility1, trajectory1] = CLF.move_goalpose_client('arm', 'gripper', start_state, goal_poses[i], goal_orientations[i], [], planner_name, n_attempt, c_time, n_repeat)
                i = i + 1
                # time.sleep(1)
                # CLF.del_box_client('can_bottom_x')
                # CLF.del_box_client('can_bottom_y')
                # CLF.del_box_client('can_bottom_z')
        return [goal_poses[i-1], goal_orientations[i-1]]

    def place(self, in_obs_pos, place_pose, current_joints):
        print "place at:", place_pose
        vfh_obs_pos = copy.deepcopy(in_obs_pos)
        vfh_obs_pos.remove(place_pose)
        vfh_obs_pos.extend(self.obs_wall)
        vfh_tar_pos = copy.deepcopy(place_pose)
        ob = len(vfh_obs_pos)
        vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta)
        if vfh[3] == 1:
            print "angle:", vfh[-1]
            xi, yi = place_pose[0], place_pose[1]

            planner_name = 'RRTConnect'
            # planner_name = 'BiTRRT'
            n_attempt = 500
            c_time = 0.5
            n_repeat = 10
            start_state = moveit_msgs.msg.RobotState()
            joint_state = sensor_msgs.msg.JointState()
            joint_state.header = std_msgs.msg.Header()
            joint_state.name = ['j2n6s300_joint_1', 'j2n6s300_joint_2', 'j2n6s300_joint_3', 'j2n6s300_joint_4', 'j2n6s300_joint_5', 'j2n6s300_joint_6']
            # joint_state.position = current_joints
            # joint_state.position = [3.1415927410125732, 4.537856101989746, 5.93411922454834, -0.6108652353286743, 1.7453292608261108, -0.5235987901687622]
            start_state.joint_state = joint_state
            # goal_pose:
            # goal_orientation:
            # goal_pose = [self.object_z, -yi, xi - 0.05]
            # goal_orientation = [-0.00145713772037, -0.998970756926, 0.0364956710831, 0.0268955302573]
            z = self.object_z
            # goal_pose = [z + 0.02, -yi, xi]
            # Set the grasp pose: substract 17cm from the z value of the object centroid
            goal_pitches = []
            goal_pitch = np.deg2rad(vfh[-1])
            # goal_pitch = vfh[-1] + math.pi/2
            goal_pitches.append(goal_pitch)  # approaching_angle: vfh[-1] from the input
            for i in range(self.n_path_try_picknplace):
                goal_pitches.append(goal_pitch + (i + 1) * (math.pi / 36))
                goal_pitches.append(goal_pitch - (i + 1) * (math.pi / 36))
            # Get the grasp orientation (currently the front direction)
            goal_orientations = []
            for i in goal_pitches:
                goal_orientations.append(quaternion_from_euler(-i, 0, math.radians(90.0), axes='rxyz'))
                # goal_orientations.append(quaternion_from_euler(-i, math.radians(-5.0), math.radians(90.0), axes='rxyz'))

            l = self.l# = -0.00
            # l = 0.17
            goal_poses = []
            for i in goal_pitches:
                dx = math.sin(i - math.pi) * l
                dy = math.cos(i - math.pi) * l
                goal_poses.append([z + 0.05, -place_pose[1] + dx, place_pose[0] + dy])

            feasibility1 = 0
            i = 0
            while not feasibility1 and i < len(goal_pitches):
                # CLF.add_box_client('can_bottom_x', [goal_poses[i][0] - 0.15, goal_poses[i][1], goal_poses[i][2]], goal_orientations[i], [0.1, 0.005, 0.005], 'red')
                # CLF.add_box_client('can_bottom_y', [goal_poses[i][0] - 0.15, goal_poses[i][1], goal_poses[i][2]], goal_orientations[i], [0.005, 0.1, 0.005], 'blue')
                # CLF.add_box_client('can_bottom_z', [goal_poses[i][0] - 0.15, goal_poses[i][1], goal_poses[i][2]], goal_orientations[i], [0.005, 0.005, 0.1], 'green')
                [feasibility1, trajectory1] = CLF.move_goalpose_client('arm', 'gripper', start_state, goal_poses[i], goal_orientations[i], [], planner_name, n_attempt, c_time, n_repeat)
                i = i + 1
                # time.sleep(1)
                # CLF.del_box_client('can_bottom_x')
                # CLF.del_box_client('can_bottom_y')
                # CLF.del_box_client('can_bottom_z')
        return [goal_poses[i-1], goal_orientations[i-1]]

    def pre_pick(self, in_obs_pos, pick_pose, obs_to_pick):
        print "pick at:", pick_pose

        vfh_obs_pos = copy.deepcopy(in_obs_pos)
        try:
            vfh_obs_pos.remove(obs_to_pick)
        except:
            print
        vfh_obs_pos.extend(self.obs_wall)
        vfh_obs_pos.extend(self.obs_wall)
        vfh_tar_pos = copy.deepcopy(pick_pose)
        ob = len(vfh_obs_pos)
        vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta)
        if vfh[3] == 1:
            print "\nangle:", vfh[-1]
            xi, yi = pick_pose[0], pick_pose[1]

            planner_name = 'RRTConnect'
            # planner_name = 'BiTRRT'
            n_attempt = 500
            c_time = 0.5
            n_repeat = 10
            start_state = moveit_msgs.msg.RobotState()
            joint_state = sensor_msgs.msg.JointState()
            joint_state.header = std_msgs.msg.Header()
            joint_state.name = ['j2n6s300_joint_1', 'j2n6s300_joint_2', 'j2n6s300_joint_3', 'j2n6s300_joint_4', 'j2n6s300_joint_5', 'j2n6s300_joint_6']
            # joint_state.position = current_joints
            joint_state.position = [3.1415927410125732, 4.537856101989746, 5.93411922454834, -0.6108652353286743, 1.7453292608261108, -0.5235987901687622]

            # joint_state.position = [3.1415927410125732, 4.537856101989746, 5.93411922454834, -0.6108652353286743, 1.7453292608261108, -0.5235987901687622]
            start_state.joint_state = joint_state
            # goal_pose:
            # goal_orientation:
            # goal_pose = [self.object_z, -yi, xi - 0.05]
            # goal_orientation = [-0.00145713772037, -0.998970756926, 0.0364956710831, 0.0268955302573]
            z = self.object_z
            # goal_pose = [z + 0.02, -yi, xi]
            # Set the grasp pose: substract 17cm from the z value of the object centroid
            goal_pitches = []
            goal_pitch = np.deg2rad(vfh[-1])
            # goal_pitch = vfh[-1] + math.pi/2
            goal_pitches.append(goal_pitch)  # approaching_angle: vfh[-1] from the input
            for i in range(12):
                goal_pitches.append(goal_pitch + (i + 1) * (math.pi / 36))
                goal_pitches.append(goal_pitch - (i + 1) * (math.pi / 36))
            # Get the grasp orientation (currently the front direction)
            goal_orientations = []
            for i in goal_pitches:
                goal_orientations.append(quaternion_from_euler(-i, math.radians(-5.0), math.radians(90.0), axes='rxyz'))
            l = 0.05
            # l = 0.17
            goal_poses = []
            for i in goal_pitches:
                dx = math.sin(i - math.pi) * l
                dy = math.cos(i - math.pi) * l
                goal_poses.append([z+0.05, -pick_pose[1] + dx, pick_pose[0] + dy])

            feasibility1 = 0
            i = 0
            while not feasibility1 and i < len(goal_pitches):
                # CLF.add_box_client('can_bottom_x', [goal_poses[i][0] - 0.15, goal_poses[i][1], goal_poses[i][2]], goal_orientations[i], [0.1, 0.005, 0.005], 'red')
                # CLF.add_box_client('can_bottom_y', [goal_poses[i][0] - 0.15, goal_poses[i][1], goal_poses[i][2]], goal_orientations[i], [0.005, 0.1, 0.005], 'blue')
                # CLF.add_box_client('can_bottom_z', [goal_poses[i][0] - 0.15, goal_poses[i][1], goal_poses[i][2]], goal_orientations[i], [0.005, 0.005, 0.1], 'green')
                [feasibility1, trajectory1] = CLF.move_goalpose_client('arm', 'gripper', start_state, goal_poses[i], goal_orientations[i], [], planner_name, n_attempt, c_time, n_repeat)
                i = i + 1
                # time.sleep(1)
                # CLF.del_box_client('can_bottom_x')
                # CLF.del_box_client('can_bottom_y')
                # CLF.del_box_client('can_bottom_z')
        return [goal_poses[i-1], goal_orientations[i-1]]

    def pre_place(self, in_obs_pos, place_pose, current_joints):
        print "place at:", place_pose
        vfh_obs_pos = copy.deepcopy(in_obs_pos)
        vfh_obs_pos.remove(place_pose)
        vfh_obs_pos.extend(self.obs_wall)
        vfh_tar_pos = copy.deepcopy(place_pose)
        ob = len(vfh_obs_pos)
        vfh = influence(ob, vfh_tar_pos, vfh_obs_pos, self.rob_pos, self.d_max, self.eta)
        if vfh[3] == 1:
            print "angle:", vfh[-1]
            xi, yi = place_pose[0], place_pose[1]

            planner_name = 'RRTConnect'
            # planner_name = 'BiTRRT'
            n_attempt = 500
            c_time = 0.5
            n_repeat = 10
            start_state = moveit_msgs.msg.RobotState()
            joint_state = sensor_msgs.msg.JointState()
            joint_state.header = std_msgs.msg.Header()
            joint_state.name = ['j2n6s300_joint_1', 'j2n6s300_joint_2', 'j2n6s300_joint_3', 'j2n6s300_joint_4', 'j2n6s300_joint_5', 'j2n6s300_joint_6']
            # joint_state.position = current_joints
            # joint_state.position = [3.1415927410125732, 4.537856101989746, 5.93411922454834, -0.6108652353286743, 1.7453292608261108, -0.5235987901687622]
            start_state.joint_state = joint_state
            # goal_pose:
            # goal_orientation:
            # goal_pose = [self.object_z, -yi, xi - 0.05]
            # goal_orientation = [-0.00145713772037, -0.998970756926, 0.0364956710831, 0.0268955302573]
            z = self.object_z
            # goal_pose = [z + 0.02, -yi, xi]
            # Set the grasp pose: substract 17cm from the z value of the object centroid
            goal_pitches = []
            goal_pitch = np.deg2rad(vfh[-1])
            # goal_pitch = vfh[-1] + math.pi/2
            goal_pitches.append(goal_pitch)  # approaching_angle: vfh[-1] from the input
            for i in range(12):
                goal_pitches.append(goal_pitch + (i + 1) * (math.pi / 36))
                goal_pitches.append(goal_pitch - (i + 1) * (math.pi / 36))
            # Get the grasp orientation (currently the front direction)
            goal_orientations = []
            for i in goal_pitches:
                goal_orientations.append(quaternion_from_euler(-i, 0, math.radians(90.0), axes='rxyz'))
                # goal_orientations.append(quaternion_from_euler(-i, math.radians(-5.0), math.radians(90.0), axes='rxyz'))

            l = 0.05
            # l = 0.17
            goal_poses = []
            for i in goal_pitches:
                dx = math.sin(i - math.pi) * l
                dy = math.cos(i - math.pi) * l
                goal_poses.append([z + 0.15, -place_pose[1] + dx, place_pose[0] + dy ])

            feasibility1 = 0
            i = 0
            while not feasibility1 and i < len(goal_pitches):
                # CLF.add_box_client('can_bottom_x', [goal_poses[i][0] - 0.15, goal_poses[i][1], goal_poses[i][2]], goal_orientations[i], [0.1, 0.005, 0.005], 'red')
                # CLF.add_box_client('can_bottom_y', [goal_poses[i][0] - 0.15, goal_poses[i][1], goal_poses[i][2]], goal_orientations[i], [0.005, 0.1, 0.005], 'blue')
                # CLF.add_box_client('can_bottom_z', [goal_poses[i][0] - 0.15, goal_poses[i][1], goal_poses[i][2]], goal_orientations[i], [0.005, 0.005, 0.1], 'green')
                [feasibility1, trajectory1] = CLF.move_goalpose_client('arm', 'gripper', start_state, goal_poses[i], goal_orientations[i], [], planner_name, n_attempt, c_time, n_repeat)
                i = i + 1
                # time.sleep(1)
                # CLF.del_box_client('can_bottom_x')
                # CLF.del_box_client('can_bottom_y')
                # CLF.del_box_client('can_bottom_z')
        return [goal_poses[i-1], goal_orientations[i-1]]

    def back_home(self, current_joints):
        print "go to home"
        planner_name = 'RRTConnect'
        # planner_name = 'BiTRRT'
        n_attempt = 100
        c_time = 3
        n_repeat = 5
        start_state = moveit_msgs.msg.RobotState()
        joint_state = sensor_msgs.msg.JointState()
        joint_state.header = std_msgs.msg.Header()
        joint_state.name = ['j2n6s300_joint_1', 'j2n6s300_joint_2', 'j2n6s300_joint_3', 'j2n6s300_joint_4', 'j2n6s300_joint_5', 'j2n6s300_joint_6']
        joint_state.position = current_joints
        # joint_state.position = [3.1415927410125732, 4.537856101989746, 5.93411922454834, -0.6108652353286743, 1.7453292608261108, -0.5235987901687622]
        start_state.joint_state = joint_state
        # goal_pose:
        feasibility1 = 0
        [feasibility1, trajectory1] = CLF.move_goalpose_client('arm', 'gripper', start_state, [self.jaco_home_pos[0]+0.1, self.jaco_home_pos[1], self.jaco_home_pos[2]], self.jaco_home_ori, [], planner_name, n_attempt, c_time, n_repeat)

    def move_to(self, goal_pos):
        print "pos:", goal_pos[0], "ori:", goal_pos[1]
        CLF.move_cartesian('arm', goal_pos[0], goal_pos[1])


class CanInfo:
    def __init__(self, type, pos, grid):
        self.type = type
        self.pos = pos
        self.grid = grid
        self.A = 0
        self.BT = 1
        self.b = 0
        self.ORC = []  # to access, need to remove

    def show(self):
        print "\nCandidate Info"

if __name__=="__main__":
    c = []
    # for i in range(10):
    #     c.append(CandidateInfo())
    #
    # print "c position", c[0].pos
    # print c