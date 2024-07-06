import numpy as np
import matplotlib.pyplot as plt
import scipy.io


def hyperplane_shifting(W, t_min, t_max, m):

    c_list = []
    d1_list = []
    d2_list = []
    for col_index in range(W.shape[1] - 1):
        for it_index in range(1, W.shape[1] - col_index):
            c = np.cross(W[:, col_index], W[:, col_index + it_index])
            c_list.append(c)

    for c_index, c in enumerate(c_list):
        d1 = m * np.dot(c, np.array([0, 0, -9.8]))
        d2 = -m * np.dot(c, np.array([0, 0, -9.8]))
        for col_index in range(W.shape[1]):
            if np.dot(c, W[:, col_index]) > 0:
                d1 += t_max * np.dot(c, W[:, col_index])
                d2 -= t_min * np.dot(c, W[:, col_index])
            elif np.dot(c, W[:, col_index]) < 0:
                d1 += t_min * np.dot(c, W[:, col_index])
                d2 -= t_max * np.dot(c, W[:, col_index])
            else:
                pass
        d1_list.append(d1)
        d2_list.append(d2)

    return c_list, d1_list, d2_list


def calculate_static_raw(W, t_min, t_max, m):

    c_list, d1_list, d2_list = hyperplane_shifting(W, t_min, t_max, m)

    r_list = []
    for index in range(len(d1_list)):
        c = c_list[index]
        d1 = d1_list[index]
        d2 = d2_list[index]
        if d1 * d2 < 0:
            r_list.append(-1)
        else:
            r_list.append(min([np.abs(d1) / np.linalg.norm(c),
                               np.abs(d2) / np.linalg.norm(c)]))

    return min(r_list)


def check_inside(pos):
    middle_level = 0.172
    center_x = 0.249
    center_y = 0.1515
    Ot = np.array([center_x, center_y, 0.337]) - np.array([center_x, center_y, 0])
    Ob1 = np.array([0.362, 0.264, 0.000]) - np.array([center_x, center_y, 0])
    Ob2 = np.array([0.136, 0.264, 0.000]) - np.array([center_x, center_y, 0])
    Ob3 = np.array([0.136, 0.039, 0.000]) - np.array([center_x, center_y, 0])

    if ((Ob2[0] < pos[0]) and (pos[0] < Ob1[0]) and (Ob3[1] < pos[1]) and (
            pos[1] < Ob1[1]) and (0 < pos[2]) and (pos[2] < middle_level)) or (
            ((middle_level < pos[2]) and (pos[2] < Ot[2])) and (
            Ob2[0] < pos[0]) and (pos[0] < Ob1[0]) and (
                    (Ob3[1] / Ob1[0] * abs(pos[0]) < pos[1]) and
                    (pos[1] < Ob1[1] / Ob1[0] * abs(pos[0])))):
        is_inside = True
    else:
        is_inside = False

    return is_inside


def check_collision(pos):
    is_coll = False

    A1 = np.array([0.342, 0.342, 0.727])
    A2 = np.array([-0.342, 0.342, 0.727])
    A3 = np.array([-0.342, -0.342, 0.727])
    A4 = np.array([0.342, -0.342, 0.727])
    A_list = [A1, A2, A3, A4]

    for cable_index in range(4):
        if not is_coll:
            for i in range(100):
                check_point = pos + (A_list[cable_index] - pos) / 100 * i
                if check_inside(check_point):
                    is_coll = True
                    break

    return is_coll


if __name__ == "__main__":

    A1 = np.array([0.342, 0.342, 0.727])
    A2 = np.array([-0.342, 0.342, 0.727])
    A3 = np.array([-0.342, -0.342, 0.727])
    A4 = np.array([0.342, -0.342, 0.727])

    x_num = 20
    y_num = 20
    z_num = 20

    x_step_len = (A1[0] - A2[0] - 0.02) / (x_num - 1)
    y_step_len = (A1[1] - A3[1] - 0.02) / (y_num - 1)
    z_step_len = (A1[2] - 0.01) / (z_num - 1)

    raw_matrix = np.zeros([x_num, y_num, z_num])

    ax = plt.axes(projection='3d')

    for x_step in range(x_num):
        for y_step in range(y_num):
            for z_step in range(z_num):
                x = A2[0] + 0.01 + x_step * x_step_len
                y = A3[1] + 0.01 + y_step * y_step_len
                z = 0 + z_step * z_step_len
                pos = np.array([x, y, z])

                if check_collision(pos):
                    raw = 0
                else:

                    u1 = (A1 - pos) / np.linalg.norm(A1 - pos)
                    u2 = (A2 - pos) / np.linalg.norm(A2 - pos)
                    u3 = (A3 - pos) / np.linalg.norm(A3 - pos)
                    u4 = (A4 - pos) / np.linalg.norm(A4 - pos)

                    J = np.vstack([u1, u2, u3, u4])
                    raw = calculate_static_raw(J.T, 0, 50, 1)
                # print(raw)

                raw_matrix[x_step, y_step, z_step] = raw

    scipy.io.savemat("raw.mat", {'raw_matrix': raw_matrix})


