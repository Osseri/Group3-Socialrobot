# Modified VFH+ for mobile manipulator

# Original author of VFH+: Jinhwi Lee (jinhooi@kist.re.kr)

# Modified by Changjoo Nam (cjnam@kist.re.kr)

# Date: 10/29/2018

# Description: Modified the original VFH+ developed by J. Lee to use for mobile manipulators

#              where Body_position can have an arbitrary pose. See tags CNAM in the codes to

#              see the modifications.



def influence(ob, target, obstacle, Body_position, d_max, ob_radius, target_radius, direct):



    import numpy as np

    import matplotlib.pyplot as plt

    import math


    if direct:
        eta = 30
    else:
        eta = 30

    front = 0.10

    alpha = 1.0



    beta_e = []

    dis_ob = []

    ramda = []

    l = []

    x_dmax = []

    y_dmax = []



    sector = np.linspace(-eta, eta, eta*2, endpoint=True)



    # The size of the robot implemented by enlarging all the active voxels

    r_r = 0.035

    r_s = 0.002

    #r_t = 0.035

    r_t = target_radius

    d_max = d_max + r_r + r_s



    r_rsv = []

    #r_v = [0 for cols in range(len(ob_radius))]

    #for i in range(len(ob_radius)):

    #    r_v.append(ob_radius[i])

    #    r_rsv.append(r_r + r_s + r_v[i] + r_t)



    for i in range(len(ob_radius)):

        r_rsv.append(r_r + r_s + ob_radius[i] + r_t + 0.01)



    #r_v = 0.035

    #r_t = 0.035

    #r_rsv = r_r + r_s + r_v + r_t



    # CNAM: Get the rotation angles converted to 0-360 degree range

    i_x = target[0]

    i_y = target[1]

    j_x = Body_position[0]

    j_y = Body_position[1]



    rotate_angle = (np.arctan2(abs(j_y - i_y), abs(j_x - i_x)))

    if i_x > j_x and i_y > j_y:

        rotate_angle = math.pi/2 - rotate_angle

    elif i_x <= j_x and i_y > j_y:

        rotate_angle = -math.pi/2 + rotate_angle

    elif i_x > j_x and i_y <= j_y:

        rotate_angle = math.pi/2 + rotate_angle

    elif i_x <= j_x and i_y <= j_y:

        rotate_angle = -math.pi/2 - rotate_angle



    Rz = np.matrix([[np.cos(rotate_angle), -np.sin(rotate_angle)],

                    [np.sin(rotate_angle), np.cos(rotate_angle)]])



    ## The enlargement angle

    for i in range(ob):

        obstacle_r = Rz * (np.matrix([[obstacle[i][0]], [obstacle[i][1]]]) - np.matrix([[target[0]], [target[1]]])) + np.matrix([[target[0]], [target[1]]])



        # CNAM: Get the relative angle of obstacles from the target in 0-360 degree range

        o_x = obstacle_r[0, 0]

        o_y = obstacle_r[1, 0]

        t_x = target[0]

        t_y = target[1]

        beta = (np.arctan2(abs(o_y - t_y), abs(o_x - t_x)))

        if t_x > o_x and t_y > o_y:

            beta = 3*math.pi/2 + beta

        elif t_x <= o_x and t_y > o_y:

            beta = math.pi/2 - beta

        elif t_x > o_x and t_y <= o_y:

            beta = 3*math.pi/2 - beta

        elif t_x <= o_x and t_y <= o_y:

            beta = math.pi/2 + beta



        beta_e.append(np.rad2deg(beta))

        dis_ob.append(np.sqrt((target[0] - obstacle[i][0]) ** 2 + (target[1] - obstacle[i][1]) ** 2))



        if r_rsv[i] / dis_ob[i] > 1:

            ramda.append(np.rad2deg(np.pi / 2))

        else:

            ramda.append(np.rad2deg(np.arcsin(r_rsv[i] / dis_ob[i])) / alpha)

        l.append(dis_ob[i] - r_rsv[i])



        x_dmax.append(target[0] - (front) * np.cos(np.deg2rad(beta_e[i])))

        y_dmax.append(target[1] - (front) * np.sin(np.deg2rad(beta_e[i])))



    ws = d_max

    c = 1.0

    b = 0.5

    a = 1 + b * ((ws - 1) / 2.0) ** 2



    # Convert to 0-360 range

    e = (sector + 720) % 360

    size_sector = len(sector)



    h = [[0 for cols in range(size_sector)] for rows in range(ob)]

    H_p = [[0 for cols in range(size_sector)] for rows in range(ob)]

    H_sum = [0 for cols in range(size_sector)]



    h = np.array(h)

    H_p = np.array(H_p)

    H_sum = np.array(H_sum)



    # CNAM: Computation of the histogram considering the discontinuity around the extreme values of 0-360 degree range

    for i in range(ob):#

        beta_e_lamda_minus = ((beta_e[i] - (ramda[i] / alpha)) + 720) % 360

        beta_e_lamda_plus = ((beta_e[i] + (ramda[i] / alpha)) + 720) % 360

        for j in range(size_sector):

            if e[j] > 180 and beta_e_lamda_minus > 180 and beta_e_lamda_plus <= 180:

                if e[j] >= beta_e_lamda_minus and e[j] >= beta_e_lamda_plus:

                    if dis_ob[i] > d_max:

                        h[i][j] = 0

                    else:

                        h[i][j] = round(c ** 2 * (a - b * dis_ob[i] ** 2))

                else:

                    h[i][j] = 0

            elif e[j] <= 180 and beta_e_lamda_minus > 180 and beta_e_lamda_plus <= 180:

                if e[j] <= beta_e_lamda_minus and e[j] <= beta_e_lamda_plus:

                    if dis_ob[i] > d_max:

                        h[i][j] = 0

                    else:

                        h[i][j] = round(c ** 2 * (a - b * dis_ob[i] ** 2))

                else:

                    h[i][j] = 0

            else:

                if e[j] >= beta_e_lamda_minus and e[j] <= beta_e_lamda_plus:

                    if dis_ob[i] > d_max:

                        h[i][j] = 0

                    else:

                        h[i][j] = round(c ** 2 * (a - b * dis_ob[i] ** 2))

                else:

                    h[i][j] = 0

            H_p[i][j] = H_p[i][j] + h[i][j]



        '''

        plt.subplot(132)

        plt.plot(sector, H_p[i])

    plt.title('magnitude', fontsize=30)

    plt.xlabel('sector(deg)', fontsize=20)

    plt.ylabel('magnitude', fontsize=20)

    plt.axis([-90, 90, 0, 6])

    '''



    for i in range(ob):

        for j in range(size_sector):

            H_sum[j] = H_sum[j] + H_p[i][j]



    H_min = min(H_sum)

    vector_sector = []

    vector_zero = []

    for i in range(int(size_sector/2), size_sector):

        if H_sum[i] == 0:

            vector_zero.append(i)

        elif H_sum[i] == H_min:

            vector_sector.append(i)

    for i in range(int(size_sector/2-1), -1, -1):

        if H_sum[i] == 0:

            vector_zero.append(i)

        elif H_sum[i] == H_min:

            vector_sector.append(i)



    vector_zero.sort()

    min_sector = []

    sector_zero = []

    ms = []

    target_direction = 0

    if len(vector_zero) != 0:

        for i in range(len(vector_zero)):

            min_sector.append(np.abs(vector_zero[i] - size_sector/2))

            sector_zero.append(vector_zero[i])

            ms.append([min_sector[i], sector_zero[i]])

        ms.sort()

        vector_min = ms[0][1]

        vector_max = ms[0][1]

        target_direction = 1

    else:

        vector_min = min(vector_sector)

        vector_max = max(vector_sector)



    angle_min = vector_min-(size_sector/2)

    angle_max = vector_max-(size_sector/2)



    x_min = target[0] - (front) * np.cos(np.deg2rad(angle_min))

    y_min = target[1] - (front) * np.sin(np.deg2rad(angle_min))

    x_max = target[0] - (front) * np.cos(np.deg2rad(angle_max))

    y_max = target[1] - (front) * np.sin(np.deg2rad(angle_max))



    k = []

    m = []

    km = []

    for i in range(ob):

        if dis_ob[i] > d_max:

            k.append(1000)

        else:

            k.append(np.abs((vector_min + vector_max) / 2 - (size_sector / 2 + beta_e[i])))

        m.append(i)

        km.append([k[i], m[i]])

    km.sort()

    #print(km)



    #plt.bar(sector, H_sum)

    #plt.title('Sum of magnitude', fontsize=15)

    #plt.xlabel('sector (deg)', fontsize=15)

    #plt.ylabel('magnitude', fontsize=15)

    #plt.tick_params(labelsize=15)

    #plt.axis([-45, 45, 0, 3], endpoint=True)

    #plt.xticks([-45, -22.5, 0, 22.5, 45])

    #plt.xticks([-180, -135, -90, -45, 0, 45, 90, 135, 180])

    '''

    if beta_e[km[0][1]] >= 45:

        beta_e[km[0][1]] = 45

    elif beta_e[km[0][1]] <= -45:

        beta_e[km[0][1]] = -45

    '''



    #if len(vector_zero) < 30:

    #    target_direction = 0

    return [x_min, y_min, np.deg2rad(angle_min)], [x_max, y_max, np.deg2rad(angle_max)], [x_dmax[km[0][1]], y_dmax[km[0][1]], np.deg2rad(beta_e[km[0][1]]), km[0][1]], target_direction