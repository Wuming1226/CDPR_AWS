import numpy as np
import scipy.io
from calculate_separation_v1 import calculate_separation_1, calculate_separation_2, calculate_separation_3
from hyperplane_shifting import calculate_static_raw

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

    middle_level = 0.172
    center_x = 0.249
    center_y = 0.1515
    Ot = np.array([center_x, center_y, 0.337]) - np.array([center_x, center_y, 0])
    Om1 = np.array([0.362, 0.264, middle_level]) - np.array([center_x, center_y, 0])
    Om2 = np.array([0.136, 0.264, middle_level]) - np.array([center_x, center_y, 0])
    Om3 = np.array([0.136, 0.039, middle_level]) - np.array([center_x, center_y, 0])
    Om4 = np.array([0.362, 0.039, middle_level]) - np.array([center_x, center_y, 0])
    Ob1 = np.array([0.362, 0.264, 0.000]) - np.array([center_x, center_y, 0])
    Ob2 = np.array([0.136, 0.264, 0.000]) - np.array([center_x, center_y, 0])
    Ob3 = np.array([0.136, 0.039, 0.000]) - np.array([center_x, center_y, 0])
    Ob4 = np.array([0.362, 0.039, 0.000]) - np.array([center_x, center_y, 0])

    x_num = 20
    y_num = 20
    z_num = 20

    x_step_len = (A1[0] - A2[0] - 0.02) / (x_num - 1)
    y_step_len = (A1[1] - A3[1] - 0.02) / (y_num - 1)
    z_step_len = (A1[2] - 0.01) / (z_num - 1)

    raw_matrix = np.zeros([x_num, y_num, z_num])

    for x_step in range(x_num):
        for y_step in range(y_num):
            for z_step in range(z_num):
                x = A2[0] + 0.01 + x_step * x_step_len
                y = A3[1] + 0.01 + y_step * y_step_len
                z = 0 + z_step * z_step_len
                pos = np.array([x, y, z])

                if x < 0 and -x <= y:
                    if (not check_inside(pos)) and check_collision(pos):
                        u1 = A1 - pos
                        u2 = A2 - pos

                        seps, _ = calculate_separation_2(A3, pos, Ot, Om1, Om2, 3)
                        u31 = seps[-1, :] - pos
                        seps, _ = calculate_separation_2(A4, pos, Om1, Om2, Ot, 1)
                        u41 = seps[-1, :] - pos

                        u32 = A3 - pos
                        seps, _ = calculate_separation_3(A3, pos, Ot, Om2, Ob2, Om3, 4)
                        u42 = seps[-1, :] - pos

                        u33 = A3 - pos
                        seps, _ = calculate_separation_2(A3, pos, Ot, Om1, Om2, 3)
                        u43 = seps[-1, :] - pos

                        seps, _ = calculate_separation_1(A3, pos, Om2, Ob2)
                        u34 = seps[-1, :] - pos
                        seps, _ = calculate_separation_2(A4, pos, Om1, Om2, Ot, 1)
                        u44 = seps[-1, :] - pos

                        J1 = np.vstack((u1, u2, u31, u41))
                        J2 = np.vstack((u1, u2, u32, u42))
                        J3 = np.vstack((u1, u2, u33, u43))
                        J4 = np.vstack((u1, u2, u34, u44))

                        raw1 = calculate_static_raw(J1.T, 0, 50, 1)
                        raw2 = calculate_static_raw(J2.T, 0, 50, 1)
                        raw3 = calculate_static_raw(J3.T, 0, 50, 1)
                        raw4 = calculate_static_raw(J4.T, 0, 50, 1)

                        raw = max([raw1, raw2, raw3, raw4])

                    else:
                        raw = 0

                    raw_matrix[x_step, y_step, z_step] = raw
                    raw_matrix[x_num - x_step - 1, y_step, z_step] = raw
                    raw_matrix[x_step, y_num - y_step - 1, z_step] = raw
                    raw_matrix[x_num - x_step - 1, y_num - y_step - 1, z_step] = raw

                    raw_matrix[y_num - y_step - 1, x_num - x_step - 1, z_step] = raw
                    raw_matrix[x_num - (y_num - y_step), x_num - x_step - 1, z_step] = raw
                    raw_matrix[y_num - y_step - 1, y_num - (x_num - x_step), z_step] = raw
                    raw_matrix[x_num - (y_num - y_step), y_num - (x_num - x_step), z_step] = raw

    scipy.io.savemat("raw_add.mat", {'raw_matrix_add': raw_matrix})








