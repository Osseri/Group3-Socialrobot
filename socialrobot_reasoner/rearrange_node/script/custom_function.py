#!/usr/bin/env python
def linspace2D(start, stop, num=50, endpoint=True, retstep=False, dtype=None):
    import numpy as np
    ret = []
    px = np.linspace(start[0], stop[0], num)
    py = np.linspace(start[1], stop[1], num)
    for i in zip(px, py):
        ret.append(list(i))
    return ret


def get_obstacle_re(ob, target_ori, obs_pos_in, Body_position, d_max):
    '''

    :param ob:
    :param target_ori:
    :param obs_pos_in:
    :param Body_position:
    :param d_max:
    :return:
    '''
    import copy
    from VFHplus_change_radius import influence
    obstacle_rearr = []
    obs_pos = copy.deepcopy(obs_pos_in)
    vfh, km = influence(ob, target_ori, obs_pos, Body_position, d_max)
    if vfh == 1:
        return 0
    while 1:
        ob = len(obs_pos)
        vfh, km = influence(ob, target_ori, obs_pos, Body_position, d_max)
        if vfh == 1:
            return obstacle_rearr
        elif vfh == 0:
            obstacle_rearr.append(obs_pos[km[0][1]])
            target_ori = obs_pos[km[0][1]]
            obs_pos.remove(obs_pos[km[0][1]])


def getEmpOcc(grid_list):
    '''
    'getEmpOcc' checks if the grid is empty or not
    :param grid_list: is an array that you want to know each grid is whether empty or occupied.
    :return: emp_g : list of empty grids
           : occ_g : list of occupied grids
    '''
    import numpy as np
    emp_g = []
    occ_g = []
    for wi in range(np.shape(grid_list)[0]):
        for di in range(np.shape(grid_list)[1]):
            if grid_list[wi][di] == 0:
                emp_g.append([wi, di])
            else:
                occ_g.append([wi, di])
    return emp_g, occ_g


def place_circle_object_ig(grid_list, obj_r, obj_type, GRID_SIZE = 0.01):
    '''

    :param grid_list:
    :param obj_r:
    :param obj_type:
    :param GRID_SIZE: distance of every grids
    :return:
    '''
    import numpy as np
    emp_g, occ_g = getEmpOcc(grid_list)
    while 1:
        # This part is for checking the occlusion
        ran_c = np.random.randint(0, len(emp_g))
        empty_check = 0
        # print(ran_c)
        for oc in range(len(occ_g)):
            d_w = emp_g[ran_c][0] - occ_g[oc][0]
            d_d = emp_g[ran_c][1] - occ_g[oc][1]
            d_c = (d_w * d_w + d_d * d_d) ** 0.5 * GRID_SIZE
            if d_c <= obj_r + 0.02:
                empty_check = 1
        # This part is to occlude the empty grid to given grid type
        if empty_check == 0:
            for em in range(len(emp_g)):
                d_w = emp_g[ran_c][0] - emp_g[em][0]
                d_d = emp_g[ran_c][1] - emp_g[em][1]
                d_c = (d_w * d_w + d_d * d_d) ** 0.5 * GRID_SIZE
                if d_c <= obj_r:
                    grid_list[emp_g[em][0]][emp_g[em][1]] = obj_type
                    occ_g.append([emp_g[em][0], emp_g[em][1]])
            return grid_list, emp_g[ran_c]


def mark_edge_grid(input_grid_list):
    '''
    Consider the grid as the workspace. To keep objects inside of the workspace,
    we mark grid edges to int 1.
    :param input_grid_list: grid that we are going to mark
    :return: grid that marked the edge
    '''
    import numpy as np
    import copy

    grid_tmp = copy.deepcopy(input_grid_list)
    w, d = np.shape(grid_tmp)
    grid_tmp[0], grid_tmp[w - 1] = 1, 1
    for i in range(w):
        grid_tmp[i][0] = 1
        grid_tmp[i][d - 1] = 1
    return grid_tmp


def draw_grid_info(input_grid_info):
    '''
    Draw a occupancy map with grid info.
    grid value 1 : black : occupied
    grid value 2 : red   : obstacle
    grid value 3 : pink  : candidate
    grid value 4 : limegreen : target
    :param input_grid_info: grid that we are going to mark
    :return: draw new figure of the input grid
    '''
    import matplotlib.pyplot as plt
    import numpy as np
    fs = 20
    # fs = 10
    new_fig = plt.figure(figsize=(fs, fs))
    for w in range(np.shape(input_grid_info)[0]):
        for d in range(np.shape(input_grid_info)[1]):
        #     if input_grid_info[w][d] == 0:
        #         plt.scatter(w, d, c='gray', alpha=0.2)
            if input_grid_info[w][d] == 1:
                plt.scatter(w, d, c='black', s=5*fs, alpha=0.5, edgecolors='none')
            elif input_grid_info[w][d] == 2:
                plt.scatter(w, d, c='red', s=5*fs, alpha=0.5, edgecolors='none')
            elif input_grid_info[w][d] == 3:
                plt.scatter(w, d, c='pink', s=5*fs, alpha=0.5, edgecolors='none')
            elif input_grid_info[w][d] == 30:
                plt.scatter(w, d, c='pink', s=5*fs, alpha=0.1, edgecolors='none')
            elif input_grid_info[w][d] == 4:
                plt.scatter(w, d, c='limegreen', s=5*fs, alpha=0.5, edgecolors='none')
    plt.axis('equal')
    # new_fig.show()


def obstacle_circle(input_grid_info, circle_xyr, grid_num, GRID_SIZE = 0.01):
    '''
    Set the grid with the information which is grid_num and the shape of circle.
    :param input_grid_info: grid that we want to use in this function
    :param circle_xyr: x and y are center position of the circle and r is the radius.
    :param grid_num: grid that we are going to mark
    grid value 1 : black : occupied
    grid value 2 : red   : obstacle
    grid value 3 : pink  : candidate
    grid value 4 : limegreen : target
    :param GRID_SIZE: distance of every grids
    :return:
    '''

    import numpy as np
    import copy
    grid_tmp = copy.deepcopy(input_grid_info)
    for w in range(np.shape(grid_tmp)[0]):
        for d in range(np.shape(grid_tmp)[1]):
            d_w = w - circle_xyr[0]
            d_d = d - circle_xyr[1]
            d_c = (d_w * d_w + d_d * d_d) ** 0.5 * GRID_SIZE
            if d_c <= circle_xyr[2]:
                grid_tmp[w][d] = grid_num
                # print "paint", w, d, "with", grid_num
    return grid_tmp


def min_len_list(input_list):
    import copy

    tmp = copy.deepcopy(input_list)
    tml = []
    for i in range(len(tmp)):
        tml.append(len(tmp[i]))
    return tmp[tml.index(min(tml))]

if __name__ == "__main__":
    print "This file has list of custom made functions"