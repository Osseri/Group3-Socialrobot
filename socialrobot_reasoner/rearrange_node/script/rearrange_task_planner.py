#!/usr/bin/env python

'''
Print type
CU: custom function
CL: client function
F : starting the function
A : for checking the algorithm
'''


GRID_SIZE = 0.01  # Distance between each grids.
G2P_SIZE = 100  # No. of grids in a meter.

import rospy
import numpy as np
import tf
import matplotlib.pyplot as plt
import copy
import time
import timeit

import custom_function as CUF
import client_function as CLF

from VFHplus_change_radius import influence
from envClass import EnvInfo as EI
from envClass import CanInfo as CI


def rearrange_task_planner(data):
    # check the service data
    # print data.target.object_name, "\n", data.target.object_position
    # for i in range(len(data.objects)):
    #     print data.objects[i].object_name, "\n", data.objects[i].object_position
    #
    # print "\n*** Step 1: get environment from given srv***\n"
    ws_name = data.workspace.object_name
    env_info = []
    for i in range(len(ws_name)):
        tmp_position = [data.workspace.object_position.x, data.workspace.object_position.y, data.workspace.object_position.z]
        tmp_orientation = [data.workspace.object_orientation.x, data.workspace.object_orientation.y, data.workspace.object_orientation.z, data.workspace.object_orientation.w]
        tmp_scale = [data.workspace.object_scale.x, data.workspace.object_scale.y, data.workspace.object_scale.z]

        tmp_info = tmp_position, tmp_orientation, tmp_scale
        env_info.append(tmp_info)

    target_name = data.target.object_name
    target_info = []
    for i in range(len(target_name)):
        tmp_position = [data.target.object_position.x, data.target.object_position.y, data.target.object_position.z]
        tmp_orientation = [data.target.object_orientation.x, data.target.object_orientation.y, data.target.object_orientation.z, data.target.object_orientation.w]
        tmp_scale = [data.target.object_scale.x, data.target.object_scale.y, data.target.object_scale.z]

        tmp_info = tmp_position, tmp_orientation, tmp_scale
        target_info.append(tmp_info)

    obstacle_name = []
    obstacle_info = []
    for i in range(len(data.objects)):
        tmp_position = [data.objects[i].object_position.x, data.objects[i].object_position.y, data.objects[i].object_position.z]
        tmp_orientation = [data.objects[i].object_orientation.x, data.objects[i].object_orientation.y, data.objects[i].object_orientation.z, data.objects[i].object_orientation.w]
        tmp_scale = [data.objects[i].object_scale.x, data.objects[i].object_scale.y, data.objects[i].object_scale.z]

        tmp_info = tmp_position, tmp_orientation, tmp_scale
        obstacle_name.append(data.objects[i].object_name)
        obstacle_info.append(tmp_info)

    ws = env_info[0]
    # print"ws info", env_info[0]
    ws_d, ws_w = int(ws[2][1] * 100), int(ws[2][0] * 100)
    # print"work space width, depth", ws_w, ws_d
    # GRID_SIZE = 0.01
    ws_zero_pos = [round(ws[0][2] - ws[2][0] * 0.5, 2), round(-ws[0][1] - ws[2][1] * 0.5, 2)]
    # print "ws cen pos", ws[0][2], ws[0][1]
    # print "ws, zero pos", ws_zero_pos

    # ws_w, ws_d = 100, 100  # get table size in the v-rep
    ws_cen = [ws[0][0], ws[0][1]]
    rob_pos = [0.0, 0.0]
    OBJ_R = 0.035

    env = EI(rob_pos, ws_w, ws_d, ws_cen, grid_size=GRID_SIZE, wall_r=OBJ_R)
    env.set_env(obstacle_name, obstacle_info, target_name, target_info)

    env.update_env(env.obs_pos, env.obs_grid)
    # print "\tA : rearrangement order:", env.ore_order
    # if len(env.ore_order) == 0:
    #     print "\tA : end rearrangement"
    #     time.sleep(1)

    # CUF.draw_grid_info(env.grid_ori)
    # plt.show()

    space_err = 0
    rearr_cnt = 0

    # env.get_env(obs_r, tar_r, min_ore)
    algorithm_start = timeit.default_timer()

    # for i in range(len(can_info)):
    #     print "can", i, ":", can_info[i].pos

    # CUF.draw_grid_info(env.grid_ori)
    # CUF.draw_grid_info(env.grid_del)
    # CUF.draw_grid_info(env.grid_max_can)
    # for c_i in range(len(can_info)):
    #     plt.text(can_info[c_i].grid[0], can_info[c_i].grid[1], 'Can' + str(c_i), fontsize=20, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
    # for o_i in range(len(env.obs_grid)):
    #     plt.text(env.obs_grid[o_i][0], env.obs_grid[o_i][1], 'Obs' + str(o_i), fontsize=20, ha='center', bbox=dict(facecolor='red', alpha=0.8))
    # plt.show()
    method = 'mine'

    # print "\n*** Step 2: find valid candidates ***\n"
    while len(env.ore_order):  # this while loop is for the algorithm
        obstacle_d = obstacle_info[env.ore_order[0]][2][0]
        # print "\tA : rearrange obstacle diameter:", obstacle_d
        padding = 0.02
        env.get_max_can(env.grid_ori, bt_num=1, trial_num=1000, obstacle_diameter=obstacle_d, padding=padding)  # We get "grid_max_can", "can_grid"
        # env.get_env_case1()
        # env.get_max_can_case1()

        '''
        Make object info!
        Type : target, obstacle, candidate
        Info : pos, grid, A, BT, b, ORC, ORE
        '''
        can_info = []
        for i in range(len(env.can_pos)):
            can_info.append((CI('candidate', env.can_pos[i], env.can_grid[i])))

        # check env info got right
        # if 1:
        #     print "\tA : # of obstacles", len(env.obs_pos), "# of candidates", len(env.can_pos)

        '''
        GET candidates info
        '''
        t_ore_order = copy.deepcopy(env.ore_order)
        # algorithm will go on until it can access to target
        #
        # env.ore_order = []
        CUF.draw_grid_info(env.grid_max_can)
        # plt.show()
        # print "\tA : OR:", env.ore_order
        # Check C.A : just next step
        # print "\tA : check Acc (0: fail, 1: success)"
        t_can_info = []

        in_can_info = copy.deepcopy(can_info)
        in_obs_pos = copy.deepcopy(env.obs_pos)
        in_obs_pos.remove(env.obs_pos[env.ore_order[0]])
        # CLF.del_box_client(obstacle_name[env.ore_order[0]])  # only in MP version
        t_can_info.append(env.get_can_A(in_can_info, in_obs_pos, env.tar_pos, obstacle_d))
        # CLF.add_box_client(obstacle_name[env.ore_order[0]], obstacle_info[env.ore_order[0]][0], obstacle_info[env.ore_order[0]][1], obstacle_info[env.ore_order[0]][2], 'red')  # only in MP version

        # Check C.BT
        # print "\n\tA : check BT (0: success, 1: fail)"
        in_can_info = copy.deepcopy(t_can_info[0])
        in_can_info = env.init_BT(in_can_info)  # init the BT value of candidates to '0'
        in_obs_pos = copy.deepcopy(env.obs_pos)
        for ore_i in range(len(env.ore_order)):  # after rearrange all ORE
            in_obs_pos.remove(env.obs_pos[env.ore_order[ore_i]])
            # CLF.del_box_client(obstacle_name[env.ore_order[ore_i]])  # only in MP version
        t_can_info[0] = env.get_can_BT(in_can_info, in_obs_pos, env.tar_pos, obstacle_d)
        # only in MP version
        # for ore_i in range(len(env.ore_order)):
            # CLF.add_box_client(obstacle_name[env.ore_order[ore_i]], obstacle_info[env.ore_order[ore_i]][0], obstacle_info[env.ore_order[ore_i]][1], obstacle_info[env.ore_order[ore_i]][2], 'red')

        # Check C.BO : BO : other ORE, just before target

        in_can_info = copy.deepcopy(t_can_info[0])
        in_obs_pos = copy.deepcopy(env.obs_pos)
        for ore_i in range(len(env.ore_order)):  # after rearrange all ORE
            in_obs_pos.remove(env.obs_pos[env.ore_order[ore_i]])
            # CLF.del_box_client(obstacle_name[env.ore_order[ore_i]])  # only in MP version
        for j in range(len(env.ore_order)):  # check other ORE just before target
            if j > i:
                t_can_info[0] = env.get_can_BT(in_can_info, in_obs_pos, env.obs_pos[env.ore_order[j]])
        # for ore_i in range(len(env.ore_order)):
        #     CLF.add_box_client(obstacle_name[env.ore_order[ore_i]], obstacle_info[env.ore_order[ore_i]][0], obstacle_info[env.ore_order[ore_i]][1], obstacle_info[env.ore_order[ore_i]][2], 'red')  # only in MP version

        s_v = []
        s_v_index = []
        for i in range(1):
            in_can_info = copy.deepcopy(t_can_info[i])
            ret_can, ret_index = env.get_cf(in_can_info)
            s_v.append(ret_can)
            s_v_index.append(ret_index)
            # print "\n step", i, " has # of cf pos:", len(t_cf[i]), "index", t_cf_index[i]
        # print "\tA : s_v:", len(s_v[0]), "\ns_v_index:", len(s_v_index[0])
        # print "\n\tA : check s_v with can index"
        # for i in range(len(s_v[0])):
        #     print "\tA : s_v index:", [i], s_v_index[0][i]
        # See the feasibile candidate
        # for i in range(len(t_cf[0])):
        #     print "\n Our Cf pos:", i, t_cf[0][i].pos
        # See if this case if case0 or case1
        # print "t_cf:", t_cf, "order", env.ore_order

        if len(s_v[0]) >= len(env.ore_order):
            # print "\n\tA : enough candidate spots"
            t_b = []
            for i in range(1):
                in_obs_pos = copy.deepcopy(env.obs_pos)
                for ore_i in range(i + 1):
                    in_obs_pos.remove(env.obs_pos[env.ore_order[ore_i]])
                t_b.append(env.get_cf_b(s_v[i], in_obs_pos))
                # print "\n step", i, " has cf b:", t_b[i]

                # draw_figs = 1
                # if draw_figs == 1:
                #     for c_i in range(len(can_info)):
                #         plt.text(can_info[c_i].grid[0], can_info[c_i].grid[1], 'Can' + str(c_i), fontsize=20, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
                #     for o_i in range(len(env.obs_grid)):
                #         plt.text(env.obs_grid[o_i][0], env.obs_grid[o_i][1], 'Obs' + str(o_i), fontsize=20, ha='center', bbox=dict(facecolor='red', alpha=0.8))
                #
                #     for step_i in range(1):
                #         step_grid = copy.deepcopy(env.grid_act)
                #         step_obs_grid = copy.deepcopy(env.obs_grid)
                #         for ore_i in range(step_i + 1):
                #             step_obs_grid.remove(env.obs_grid[env.ore_order[ore_i]])
                #         for i in range(len(step_obs_grid)):
                #             step_grid = CUF.obstacle_circle(step_grid, [round(step_obs_grid[i][0], 2), round(step_obs_grid[i][1], 2), env.obs_r[i]], 2)
                #         for ci in range(len(can_info)):
                #             xi, yi = can_info[ci].grid
                #             step_grid = CUF.obstacle_circle(step_grid, [xi, yi, 0.04], 30)
                #
                #         step_grid = CUF.obstacle_circle(step_grid, [env.tar_grid[0], env.tar_grid[1], tar_r], 4)  # target
                #
                #         for cf_i in range(len(t_b[step_i])):
                #             xi = (t_cf[step_i][cf_i].pos[0] - env.ws_zero[0]) * G2P_SIZE
                #             yi = (t_cf[step_i][cf_i].pos[1] - env.ws_zero[1]) * G2P_SIZE
                #             step_grid = CUF.obstacle_circle(step_grid, [xi, yi, 0.04], 3)
                #
                #         CUF.draw_grid_info(step_grid)
                #
                #         for cf_i in range(len(t_b[step_i])):
                #             xi = (t_cf[step_i][cf_i].pos[0] - env.ws_zero[0]) * G2P_SIZE
                #             yi = (t_cf[step_i][cf_i].pos[1] - env.ws_zero[1]) * G2P_SIZE
                #             plt.text(xi, yi, 'b=' + str(t_b[step_i][cf_i]), fontsize=20, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
                #         for ci in range(len(t_can_info[step_i])):
                #             plt.text(t_can_info[step_i][ci].grid[0], t_can_info[step_i][ci].grid[1] - 2.0, '[A, BT] :' + str([t_can_info[step_i][ci].A, t_can_info[step_i][ci].BT]), fontsize=10, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
                #         for o_i in range(len(env.obs_grid)):
                #             plt.text(env.obs_grid[o_i][0], env.obs_grid[o_i][1], 'Obs' + str(o_i), fontsize=20, ha='center', bbox=dict(facecolor='red', alpha=0.8))
                #         plt.title('step' + str(step_i) + " obs: " + str(env.ore_order[step_i]) + " rearranged")
        elif len(s_v[0]) < len(env.ore_order):
            # print "\n\tA : not enough candidate spots"
            # print "Since we meet condition: N(CF) < N(ORE) by", len(t_cf[0]), "<", len(env.ore_order), ",\nwe have to remove additional obstacles."
            ## step1 : "get t_cp", check candidates which have A = 0 and BT = 0
            ## This means that a candidate is not reachable and it does not block the target object

            # Check A for this environment state
            in_can_info = copy.deepcopy(can_info)
            in_obs_pos = copy.deepcopy(env.obs_pos)
            t_can_add = copy.deepcopy(env.get_can_A(in_can_info, in_obs_pos, env.tar_pos, obstacle_d))

            s_e = []  # s_e: extra candidate spots

            in_can_info = copy.deepcopy(t_can_add)

            ret_can, ret_index = env.get_cp(in_can_info)
            # print "\tA : # of OR'", len(ret_can)

            t_s_e = ret_can
            t_s_e_index = ret_index
            # print "t_cp:", len(t_cp), "index", t_cp_index
            # for i in range(len(t_cp)):
            #     print "\n Our Cp:", i, t_cp[i].pos

            if len(t_s_e) == 0:
                # print "\tA : no possible extra candidate exist"
                space_err = 1
                break
            # step2 : check c_ore for each cp and pick min of it
            t_s_r = []  # s_r: candidate spot relocate plan
            in_can_info = copy.deepcopy(t_s_e)
            # tmp_order_time_start = timeit.default_timer()
            # tmp_order_time_start2 = time.clock()
            t_s_r = env.get_c_ore(in_can_info)
            # tmp_order_time_end = timeit.default_timer()
            # tmp_order_time_end2 = time.clock()
            # order_time = order_time + tmp_order_time_end - tmp_order_time_start
            # order_time2 = order_time2 + tmp_order_time_end2 - tmp_order_time_start2
            # order_cnt = order_cnt + 100 * len(t_s_e)
            # print "\n"
            # for i in range(len(t_cp)):
            #     print "cp", t_cp[i].pos, "\nc_ore", c_ore[i]
            s_r = []
            s_e_index = []
            # print "\n"
            # for i in range(len(t_s_e)):
            #     print "\tA : can", t_s_e_index[i], "grid:", t_s_e[i].grid, ", s_r:", t_s_r[i]

            for i in range(len(t_s_e)):
                if t_s_r[i] != []:
                    s_e.append(t_s_e[i])
                    s_r.append(t_s_r[i])
                    s_e_index.append(t_s_e_index[i])

            # tmp_se = copy.deepcopy(s_e)
            # tmp_sr = copy.deepcopy(s_r)
            # emp_sr = []
            # for i in range(len(s_e)):
            #     if s_r[i] == []:
            #         print "remove empty s_e", i
            #         emp_sr.append(i)
            #
            # print "tmp se:", tmp_se, "\ntmp sr", tmp_sr
            # for i in range(len(emp_sr)):
            #
            #     print "tmp_se[emp_sr[i]]", tmp_se[emp_sr[i]].pos
            #     print "tmp_sr[emp_sr[i]]", tmp_sr[emp_sr[i]]
            #     s_e.remove(tmp_se[emp_sr[i]])
            #     s_r.remove(tmp_sr[emp_sr[i]])

            while len(s_e):
                # print "\tA : # of s_e:", len(s_e), s_r
                # print "\n"
                # for i in range(len(s_e)):
                #     print "\tA : can", s_e_index[i], "pos:", s_e[i].pos, ", s_r:", s_r[i]
                min_s_r = CUF.min_len_list(s_r)

                # print "\n\tA : min sr:", min_s_r
                #
                # print "picked ci index:", t_cp.index(t_cp[c_ore.index(min_c_ore)])
                # print "picked ci address:", copy.deepcopy(t_cp[c_ore.index(min_c_ore)]).pos
                cp = copy.deepcopy(s_e[s_r.index(min_s_r)])
                # print "selected cp pos", cp.pos

                ## step3 : "get t_cf", check candidates which have A = 1 and BT' = 0
                ## Check A for this environment state T' is t_cp_i
                in_can_info = copy.deepcopy(can_info)
                in_obs_pos = copy.deepcopy(env.obs_pos)
                in_tar_pos = copy.deepcopy(cp.pos)
                t_can_add = copy.deepcopy(env.get_can_A(in_can_info, in_obs_pos, env.tar_pos, obstacle_d))

                # Check C.BT for this environment state
                in_can_info = copy.deepcopy(t_can_add)
                in_can_info = env.init_BT(in_can_info)  # init the BT value of candidates to '0'
                in_obs_pos = copy.deepcopy(env.obs_pos)

                sorted_min_s_r = copy.deepcopy(min_s_r)
                sorted_min_s_r.sort(reverse=True)
                # print "\tA : sorted min_s_r:", sorted_min_s_r

                if sorted_min_s_r[0] == len(env.obs_pos):  # if OR' has o_t ! remove s_e
                    # print "\tA : o_t is in OR'"
                    s_e.remove(s_e[s_r.index(min_s_r)])
                    s_e_index.remove(s_e_index[s_r.index(min_s_r)])
                    s_r.remove(s_r[s_r.index(min_s_r)])
                else:
                    for ore_i in range(len(min_s_r)):  # after rearrange all OR'
                        in_obs_pos.remove(in_obs_pos[sorted_min_s_r[ore_i]])
                        # CLF.del_box_client(obstacle_name[sorted_min_s_r[ore_i]])  #MP
                    in_tar_pos = copy.deepcopy(cp.pos)
                    t_can_add = env.get_can_BT(in_can_info, in_obs_pos, in_tar_pos)

                    # for ore_i in range(len(min_s_r)):  # after rearrange all OR'  #MP
                    #     CLF.add_box_client(obstacle_name[sorted_min_s_r[ore_i]], obstacle_info[sorted_min_s_r[ore_i]][0], obstacle_info[sorted_min_s_r[ore_i]][1], obstacle_info[sorted_min_s_r[ore_i]][2], 'red')  #MP

                    # for i in range(len(t_can_add)):
                    #     print "can", i, "A:", t_can_add[i].A, "B:", t_can_add[i].BT

                    s_e_v = []
                    s_v_index = []

                    in_can_info = copy.deepcopy(t_can_add)
                    ret_can, ret_index = env.get_cf(in_can_info)
                    s_e_v.append(ret_can)
                    s_v_index.append(ret_index)

                    # print "\tA : s_e_v: ", s_e_v
                    # for i in range(len(s_e_v[0])):
                    #     print s_e_v[0][i].grid

                    if len(s_e_v[0]) >= len(min_s_r) - 1:
                        # print "\tA : this se is possible"
                        if len(min_s_r) == 1:
                            # print "\tA : only one move needed"
                            # t_can_info = []
                            # for i in range(len(env.ore_order)):
                            #     in_can_info = copy.deepcopy(can_info)
                            #     in_obs_pos = copy.deepcopy(env.obs_pos)
                            #     for ore_i in range(i + 1):
                            #         if min_s_r[0] != env.ore_order[ore_i]:
                            #             in_obs_pos.remove(env.obs_pos[env.ore_order[ore_i]])
                            #     in_obs_pos.remove(env.obs_pos[min_s_r[0]])
                            #     t_can_info.append(env.get_can_A(in_can_info, in_obs_pos, env.tar_pos))

                            s_v = [[s_e[s_r.index(min_s_r)]]]
                            s_v_index = [[s_e_index[s_r.index(min_s_r)]]]
                            # print "se v:", s_v, s_v[0], s_v[0][0], s_v[0][0].pos
                            # for i in range(len(env.ore_order)):
                            #     add_can_info = copy.deepcopy(t_can_info[i])
                            #     ret_can, ret_index = env.get_cf(add_can_info)
                            #     s_v.append(ret_can)
                            #     s_v_index.append(ret_index)

                            t_b = [[0]]
                            # for i in range(1):
                            #     in_obs_pos = copy.deepcopy(env.obs_pos)
                            #     for ore_i in range(i+1):
                            #         in_obs_pos.remove(env.obs_pos[env.ore_order[ore_i]])
                            #     t_b.append(env.get_cf_b(s_v[i], in_obs_pos))
                            #     # print "\n step", i, " has cf b:", t_b[i]
                            break  # for out s_e loop
                        else:
                            t_b = []
                            in_obs_pos = copy.deepcopy(env.obs_pos)
                            for ore_i in range(1):
                                in_obs_pos.remove(env.obs_pos[min_s_r[ore_i]])
                            t_b.append(env.get_cf_b(s_e_v[0], in_obs_pos))

                            s_v[0] = s_e_v[0]

                            break  # for out s_e loop
                    else:  # s_e[s_r.index(min_s_r)]
                        # print "\n\tA : remove",
                        # print "\t  : s_e:", s_e
                        # print "\t  : s_r:", s_r
                        # print "\t  : s_e_index:", s_e_index
                        s_e.remove(s_e[s_r.index(min_s_r)])
                        s_e_index.remove(s_e_index[s_r.index(min_s_r)])
                        s_r.remove(s_r[s_r.index(min_s_r)])

            if len(s_e) == 0:
                # print "no possible extra candidate exist"
                break

            env.ore_order = min_s_r
            # draw_figs = 1
            # if draw_figs == 1:
            #     for c_i in range(len(can_info)):
            #         plt.text(can_info[c_i].grid[0], can_info[c_i].grid[1], 'Can' + str(c_i), fontsize=20, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
            #     for o_i in range(len(env.obs_grid)):
            #         plt.text(env.obs_grid[o_i][0], env.obs_grid[o_i][1], 'Obs' + str(o_i), fontsize=20, ha='center', bbox=dict(facecolor='red', alpha=0.8))
            #
            #     step_i = 0
            #     step_grid = copy.deepcopy(env.grid_act)
            #     step_obs_grid = copy.deepcopy(env.obs_grid)
            #     step_obs_grid.remove(env.obs_grid[env.ore_order[0]])
            #     for i in range(len(step_obs_grid)):
            #         # print "i:", i, "step_obs_grid [i]:", step_obs_grid[i]
            #         step_grid = CUF.obstacle_circle(step_grid, [round(step_obs_grid[i][0], 2), round(step_obs_grid[i][1], 2), env.obs_r[i]], 2)
            #     for ci in range(len(can_info)):
            #         xi, yi = can_info[ci].grid
            #         step_grid = CUF.obstacle_circle(step_grid, [xi, yi, 0.04], 30)
            #
            #     step_grid = CUF.obstacle_circle(step_grid, [env.tar_grid[0], env.tar_grid[1], tar_r], 4)  # target
            #
            #     for cf_i in range(len(t_b[step_i])):
            #         xi = (t_cf[step_i][cf_i].pos[0] - env.ws_zero[0]) * G2P_SIZE
            #         yi = (t_cf[step_i][cf_i].pos[1] - env.ws_zero[1]) * G2P_SIZE
            #         step_grid = CUF.obstacle_circle(step_grid, [xi, yi, 0.04], 3)
            #
            #     CUF.draw_grid_info(step_grid)
            #
            #     for cf_i in range(len(t_b[step_i])):
            #         xi = (t_cf[step_i][cf_i].pos[0] - env.ws_zero[0]) * G2P_SIZE
            #         yi = (t_cf[step_i][cf_i].pos[1] - env.ws_zero[1]) * G2P_SIZE
            #         plt.text(xi, yi, 'b=' + str(t_b[step_i][cf_i]), fontsize=20, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
            #     for ci in range(len(t_can_info[step_i])):
            #         plt.text(t_can_info[step_i][ci].grid[0], t_can_info[step_i][ci].grid[1] - 2.0, '[A, BT] :' + str([t_can_info[step_i][ci].A, t_can_info[step_i][ci].BT]), fontsize=10, ha='center', bbox=dict(facecolor='pink', alpha=0.8))
            #     for o_i in range(len(env.obs_grid)):
            #         plt.text(env.obs_grid[o_i][0], env.obs_grid[o_i][1], 'Obs' + str(o_i), fontsize=20, ha='center', bbox=dict(facecolor='red', alpha=0.8))
            #     plt.title('step' + str(step_i) + " obs: " + str(env.ore_order[step_i]) + " rearranged")

        if space_err:
            # print "\tA : no possible extra candidate exist"
            break

        # move obstacle to can(min(b))
        # print "s_v", s_v
        # print "s_v[0]", s_v[0]
        # print "s_v[0][0]", s_v[0][0]
        # print "s_v[0][0].pos", s_v[0][0].pos
        # print "\tA : t_b[0]", t_b[0]

        find_b = copy.deepcopy(t_b[0])
        # print "move to c_", find_b.index(min(find_b))
        if method == 'far':
            t_sel_can_index = [i for i in range(len(find_b))]
        elif method == 'deep':
            t_sel_can_index = [i for i in range(len(find_b))]
        elif method == 'mine':
            t_sel_can_index = [i for i in range(len(find_b)) if find_b[i] == min(find_b)]
        t_sel_can_dist = []
        # print "\ntar grid: ", env.tar_grid
        # print "\ntar pos: ", env.tar_pos
        # print "\tA : t sel can index", t_sel_can_index

        # for the rearrange-node
        print "\tA : candidaate position:"
        return_can_pos = []
        for i in range(len(t_sel_can_index)):
            print "\t\tcan:", s_v_index[0][t_sel_can_index[i]], "pos:", round(can_info[s_v_index[0][t_sel_can_index[i]]].pos[0], 2), round(can_info[s_v_index[0][t_sel_can_index[i]]].pos[1], 2)
            tmp_pos = geometry_msgs.msg.Point()
            tmp_pos.x = round(can_info[s_v_index[0][t_sel_can_index[i]]].pos[0], 2)
            tmp_pos.y = round(can_info[s_v_index[0][t_sel_can_index[i]]].pos[1], 2)
            tmp_pos.z = round(obstacle_info[env.ore_order[0]][0][2], 2)
            return_can_pos.append(tmp_pos)

        return rearrange_env_srvResponse(
            object_name = obstacle_name[env.ore_order[0]],
            rearrange_positions = return_can_pos
        )


def listener():
    rospy.Service('rearrange_srv', rearrange_env_srv, rearrange_task_planner)
    rospy.spin()

if __name__ == "__main__":
    import rospy
    import geometry_msgs.msg
    from rearrange_node.srv._rearrange_env_srv import *
    print "*** Rearrange node started***"
    rospy.init_node('rearrange_node', anonymous=True)

    listener()

