import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def rotate_point(point, Cpoint1, Cpoint2, theta):
    """
    :param point: point to be rotated
    :param Cpoint1: left end of the bar
    :param Cpoint2: right end of the bar
    :param theta: rotation angle around rotation axis
    :return: rotated point
    """
    vector = point - Cpoint1       # vector from one end of the axis to the point
    axis = (Cpoint2 - Cpoint1) / np.linalg.norm(Cpoint2 - Cpoint1)      # unit orientation vector of the axis
    return Cpoint1 + vector * np.cos(theta) + np.cross(axis, vector) * np.sin(theta) + axis * np.dot(axis, vector) * (1 - np.cos(theta))       # rotate the vector


def get_united_normal_vector(*points):
    """
    :param points: 3 points on the plane in counter-clockwise order
    :return: unit normal vector of the plane which points outside the surface
    """
    vector1 = points[1] - points[0]
    vector2 = points[2] - points[1]
    normal_vector = np.cross(vector1, vector2)
    return normal_vector / np.linalg.norm(normal_vector)


def get_intersection(Apoint, Bpoint, Cpoint1, Cpoint2):
    """
    :param Apoint: fixed end of the cable
    :param Bpoint: free end of the cable
    :param Cpoint1: left end of the bar
    :param Cpoint2: right end of the bar
    :return: intersection point if existed, else false

    case:
                        A
                        |
                        |
            C1  ————————|——————————  C2
                        |
                        |
                        B

    """
    vector1 = Cpoint2 - Cpoint1     # orientation vector of the line1 (bar)
    vector2 = Bpoint - Apoint       # orientation vector of the line2 (cable)
    vector3 = Apoint - Cpoint1

    vecS1 = np.cross(vector1, vector2)      # signed area
    vecS2 = np.cross(vector3, vector2)      # signed area

    # check whether two lines are coplanar
    if np.dot(vector3, vecS1) >= 0.0001 or np.dot(vector3, vecS1) <= -0.0001:
        return False

    # distance from one end of the segment to the intersection / length of the segment
    ratio = np.dot(vecS1, vecS2) / np.dot(vecS1, vecS1)

    # check whether the intersection is on the segment
    if ratio > 1 or ratio < 0:
        return False
    else:
        return Cpoint1 + vector1 * ratio        # position of the intersection


def check_collision_infinite(Apoint, Bpoint, Cpoint1, Cpoint2):
    """
    :param Apoint: fixed end of the cable
    :param Bpoint: free end of the cable
    :param Cpoint1: left end of the bar
    :param Cpoint2: right end of the bar
    :return: whether collision occurs on the bar

    case:
                        A
                        |
                        |
            C1  ————————|——————————  C2
                        |
                        |
                        B

    """
    norm_vector1 = get_united_normal_vector(Apoint, Cpoint1, Cpoint2)
    norm_vector2 = get_united_normal_vector(Bpoint, Cpoint2, Cpoint1)
    axis = (Cpoint2 - Cpoint1) / np.linalg.norm(Cpoint2 - Cpoint1)
    # collision occurs when the angle between norm vectors of two planes is bigger than 90°
    if np.dot(np.cross(norm_vector1, norm_vector2), axis) > 0:
        return True
    else:
        return False


def calculate_separation_1(Apoint, Bpoint, Cpoint1, Cpoint2, print_flag=False):
    """
    :param Apoint: fixed end of the cable
    :param Bpoint: free end of the cable
    :param Cpoint1: left end of the bar
    :param Cpoint2: right end of the bar
    :return: separation point

    case:
                        A
                        |
                        |
            C1  ————————|——————————  C2
                        |
                        |
                        B

    """
    # definition
    bar_left = Cpoint1
    bar_right = Cpoint2
    collision_flag = False
    if check_collision_infinite(Apoint, Bpoint, bar_left, bar_right):
        if print_flag:
            print("possible collision on the bar")
        v1 = get_united_normal_vector(Apoint, bar_left, bar_right)
        v2 = get_united_normal_vector(Bpoint, bar_right, bar_left)
        theta = np.arccos(np.dot(v1, v2))
        Apoint_r = rotate_point(Apoint, bar_left, bar_right, theta)
        intersection = get_intersection(Apoint_r, Bpoint, bar_left, bar_right)
        if intersection is False:
            if print_flag:
                print("no collision on the bar")
            pass
        else:
            separation = intersection
            collision_flag = True

    if not collision_flag:
        if print_flag:
            print("no collision")
        separation = Apoint

    return separation.reshape(-1, 3), collision_flag


def calculate_separation_2(Apoint, Bpoint, Cpoint1, Cpoint2, Cpoint3, shared, print_flag=False):
    """
    :param Apoint: fixed end of the cable
    :param Bpoint: free end of the cable
    :param Cpoint1: the left end of bar 1
    :param Cpoint2: the second end in counterclockwise order, namely the private end of bar 2
    :param Cpoint3: the third end in counterclockwise order, namely the right end of bar 1
    :param shared: the number of Cpoint shared by bar1 and bar2
    :return separation: the separation points
            collision_flag: the collision condition, False for no collision, 'both' for 2 collisions and '1' and '2' for 1 collision

    case1:                                          case2:
                        A                                        A
             1l                    1r                  1l                   1r
         2l  1  —————————————————  3                   1  ————————————————  3  2r
                |                                                        |
                |                                                        |
         B      |                                                        |      B
                |                                                        |
                |                                                        |
           2r   2                                                        2  2l

    """
    # definition
    bar1_left = Cpoint1
    bar1_right = Cpoint3
    if shared == 1:
        bar2_left = Cpoint1
        bar2_right = Cpoint2
        bar1_private = Cpoint3
    elif shared == 3:
        bar2_left = Cpoint2
        bar2_right = Cpoint3
        bar1_private = Cpoint1
    else:
        print("wrong number")
        return False

    collision_flag = False

    # check if collisions occur on both bars
    if check_collision_infinite(Apoint, Cpoint2, Cpoint1, Cpoint3) and check_collision_infinite(bar1_private, Bpoint, bar2_left, bar2_right):
        if print_flag:  
            print("possible 2 collisions")
        v1 = get_united_normal_vector(Apoint, bar1_left, bar1_right)
        v2 = get_united_normal_vector(Cpoint1, Cpoint2, Cpoint3)
        v3 = get_united_normal_vector(Bpoint, bar2_right, bar2_left)
        theta1 = np.arccos(np.dot(v1, v2))
        theta2 = np.arccos(np.dot(v2, v3))
        Apoint_r = rotate_point(Apoint, bar1_left, bar1_right, theta1)
        Bpoint_r = rotate_point(Bpoint, bar2_right, bar2_left, theta2)
        intersection1 = get_intersection(Apoint_r, Bpoint_r, bar1_left, bar1_right)
        intersection2 = get_intersection(Apoint_r, Bpoint_r, bar2_left, bar2_right)
        if intersection1 is False or intersection2 is False:
            if print_flag:  
                print("not 2 collisions")
        else:
            separation = intersection1, intersection2
            separation = np.array(separation)
            collision_flag = 'both'

    # check if collision only occurs on bar1
    if check_collision_infinite(Apoint, Bpoint, bar1_left, bar1_right):
        if print_flag:
            print("possible collision on bar1")
        intersection, collision = calculate_separation_1(Apoint, Bpoint, bar1_left, bar1_right)
        if collision is False:
            if print_flag:
                print("no collision on bar1")
        else:
            if check_collision_infinite(intersection[-1], Bpoint, bar2_left, bar2_right):
                inter, coll = calculate_separation_1(intersection[-1], Bpoint, bar2_left, bar2_right)
                if coll is False:
                    separation = intersection
                    collision_flag = '1'
                else:
                    if print_flag:
                        print("no collision on bar1")
            else:
                separation = intersection
                collision_flag = '1'

    # check if collision only occurs on bar2
    if check_collision_infinite(Apoint, Bpoint, bar2_left, bar2_right):
        if print_flag:
            print("possible collision on bar2")
        intersection, collision = calculate_separation_1(Apoint, Bpoint, bar2_left, bar2_right)
        if collision is False:
            if print_flag:
                print("no collision on bar2")
        else:
            # check if cable goes into the obstacle, which is impossible
            if check_collision_infinite(Apoint, intersection[0], bar1_left, bar1_right):
                inter, coll = calculate_separation_1(Apoint, intersection[0], bar1_left, bar1_right)
                if coll is False:
                    separation = intersection
                    collision_flag = '2'
                else:
                    if print_flag:
                        print("no collision on bar2")
            else:
                separation = intersection
                collision_flag = '2'

    if not collision_flag:
        if print_flag:
            print("no collisions on both bars")
        separation = Apoint

    if print_flag:
        print(separation)

    return separation.reshape(-1, 3), collision_flag


def calculate_separation_3(Apoint, Bpoint, Cpoint1, Cpoint2, Cpoint3, Cpoint4, shared1, print_flag=False):
    """
    :param Apoint: fixed end of the cable
    :param Bpoint: free end of the cable
    :param Cpoint1: the left end of bar 1
    :param Cpoint2: the second end in counterclockwise order
    :param Cpoint3: the third end in counterclockwise order
    :param Cpoint4: the forth end in counterclockwise order, namely the right end of bar 1
    :param shared1: the number of Cpoint shared by bar1 and bar2
    :return separation: the separation points
            collision_flag: the collision condition, False for no collision, 'all' for 3 collisions, '12' and '23' for 2 collisions and '1', '2' and '3' for 1 collision

    case1:                                          case2:

                                1  1l                 1r  4
                                |                         |
                                |                         |
                                |      A            A     |
                                |                         |
             2l                 |                         |                 2r
         3l  2  —————————————————  4  1r            1l 1  ————————————————  3  3r
                |                  2r                  2l                |
                |                                                        |
         B      |                                                        |      B
                |                                                        |
                |                                                        |
           3r   3                                                        2  3l

    """
    # definition
    bar1_left = Cpoint1
    bar1_right = Cpoint4
    bar3_left = Cpoint2
    bar3_right = Cpoint3
    if shared1 == 4:
        bar2_left = Cpoint2
        bar2_right = Cpoint4
        bar2_private1 = Cpoint2
        bar1_private2 = Cpoint1
        bar3_private2 = Cpoint3
        bar2_private3 = Cpoint4
    elif shared1 == 1:
        bar2_left = Cpoint1
        bar2_right = Cpoint3
        bar2_private1 = Cpoint3
        bar1_private2 = Cpoint4
        bar3_private2 = Cpoint2
        bar2_private3 = Cpoint1
    else:
        print("wrong number")
        return False

    collision_flag = False

    # check if collisions occur on all bars
    if check_collision_infinite(Apoint, bar2_private1, bar1_left, bar1_right) and check_collision_infinite(bar1_private2, bar3_private2, bar2_left, bar2_right) and check_collision_infinite(bar2_private3, Bpoint, bar3_left, bar3_right):
        if print_flag:
            print("possible 3 collisions")
        v1 = get_united_normal_vector(Apoint, bar1_left, bar1_right)
        v2 = get_united_normal_vector(bar1_private2, bar2_left, bar2_right)
        v3 = get_united_normal_vector(bar3_private2, bar2_right, bar2_left)
        v4 = get_united_normal_vector(Bpoint, bar3_right, bar3_left)
        theta1 = np.arccos(np.dot(v1, v2))
        theta2 = np.arccos(np.dot(v2, v3))
        theta3 = np.arccos(np.dot(v3, v4))
        Apoint_r1 = rotate_point(Apoint, bar1_left, bar1_right, theta1)
        Apoint_r12 = rotate_point(Apoint_r1, bar2_left, bar2_right, theta2)
        bar1_left_r2 = rotate_point(bar1_left, bar2_left, bar2_right, theta2)
        bar1_right_r2 = rotate_point(bar1_right, bar2_left, bar2_right, theta2)
        Bpoint_r3 = rotate_point(Bpoint, bar3_right, bar3_left, theta3)
        intersection1_r2 = get_intersection(Apoint_r12, Bpoint_r3, bar1_left_r2, bar1_right_r2)
        intersection2 = get_intersection(Apoint_r12, Bpoint_r3, bar2_left, bar2_right)
        intersection3 = get_intersection(Apoint_r12, Bpoint_r3, bar3_left, bar3_right)
        if intersection1_r2 is False or intersection2 is False or intersection3 is False:
            if print_flag:
                print("not 3 collisions")
        else:
            intersection1 = rotate_point(intersection1_r2, bar2_right, bar2_left, theta2)
            separation = intersection1, intersection2, intersection3
            separation = np.array(separation)
            collision_flag = 'all'

    # check if collision occurs on bar1 or bar2
    if check_collision_infinite(Apoint, Bpoint, bar1_left, bar1_right) or check_collision_infinite(Apoint, Bpoint, bar2_left, bar2_right):
        if print_flag:
            print("possible collision on bar1 or bar2")
        if shared1 == 4:
            intersection, collision = calculate_separation_2(Apoint, Bpoint, bar1_left, bar2_private1, bar1_right, shared=3)
        elif shared1 == 1:
            intersection, collision = calculate_separation_2(Apoint, Bpoint, bar1_left, bar2_private1, bar1_right, shared=1)
        if collision is False:
            if print_flag:
                print("no collision on bar1 and bar2")
        else:
            if check_collision_infinite(intersection[-1], Bpoint, bar3_left, bar3_right):
                inter, coll = calculate_separation_1(intersection[-1], Bpoint, bar3_left, bar3_right)
                if coll is False:
                    separation = intersection
                    collision_flag = '1' if collision == '1' else('2' if collision == '2' else '1&2')
                else:
                    if print_flag:
                        print("no collision on bar1 and bar2")
            else:
                separation = intersection
                collision_flag = '1' if collision == '1' else('2' if collision == '2' else '1&2')

    # check if collision only occurs on bar2
    if check_collision_infinite(Apoint, Bpoint, bar2_left, bar2_right) or check_collision_infinite(Apoint, Bpoint, bar3_left, bar3_right):
        if shared1 == 4:
            intersection, collision = calculate_separation_2(Apoint, Bpoint, bar2_left, bar3_private2, bar2_right, shared=1)
        elif shared1 == 1:
            intersection, collision = calculate_separation_2(Apoint, Bpoint, bar2_left, bar3_private2, bar2_right, shared=3)
        if print_flag:
            print('inter: {}'.format(intersection))
        if collision is False:
            if print_flag:
                print("no collision on bar2 and bar3")
        else:
            # check if cable goes into the obstacle, which is impossible
            if check_collision_infinite(Apoint, intersection[0], bar1_left, bar1_right):
                inter, coll = calculate_separation_1(Apoint, intersection[0], bar1_left, bar1_right)

                if coll is False:
                    separation = intersection
                    collision_flag = '2' if collision == '1' else('3' if collision == '2' else '2&3')
                else:
                    if print_flag:
                        print("no collision on bar2 and bar3")
            else:
                separation = intersection
                collision_flag = '2' if collision == '1' else('3' if collision == '2' else '2&3')

    if not collision_flag:
        if print_flag:
            print("no collisions on all bars")
        separation = Apoint

    if print_flag:
        print(separation, collision_flag)
    return separation.reshape(-1, 3), collision_flag


def calculate_cable_length(Apoint, separations, Bpoint):
    """
    :param Apoint: fixed end of the cable
    :param Bpoint: free end of the cable
    :param separations: collision points
    :return: cable length
    """
    length = 0
    ends = np.vstack((Apoint, separations, Bpoint))

    for i in range(ends.shape[0] - 1):
        length += np.linalg.norm(ends[i+1] - ends[i])

    return length
    


if __name__ == "__main__":
    # A = np.array([-1, 0, 1])
    # B = np.array([1, -1, 0])
    # C = np.array([1, 1, 0])
    # print(rotate_point(A, B, C, np.arctan(2)))
    #
    # print(get_united_normal_vector(A, B, C))
    #
    # A = np.array([-1, 0, 1])
    # B = np.array([1, 0, -1])
    # C1 = np.array([1, -1, 0])
    # C2 = np.array([1, 1, 0])
    # print(check_collision_infinite(A, C1, C1, C2))
    #
    # A = np.array([-1, 0, 0])
    # B = np.array([1, 0, 0])
    # C = np.array([0, -1, 1])
    # D = np.array([0, 1, -1])
    # print(get_intersection(A, B, C, D))

    obs_side = 22.5
    frame_side = 72
    height = 50
    A3 = np.array([-frame_side / 2, -frame_side / 2, height])
    A2 = np.array([-frame_side / 2, frame_side / 2, height])
    Om1 = np.array([obs_side / 2, obs_side / 2, obs_side])
    Om2 = np.array([-obs_side / 2, obs_side / 2, obs_side])
    Om3 = np.array([-obs_side / 2, -obs_side / 2, obs_side])
    Om4 = np.array([obs_side / 2, -obs_side / 2, obs_side])
    Ob1 = np.array([obs_side / 2, obs_side / 2, 0])
    Ob2 = np.array([-obs_side / 2, obs_side / 2, 0])
    Ob3 = np.array([-obs_side / 2, -obs_side / 2, 0])
    Ob4 = np.array([obs_side / 2, -obs_side / 2, 0])
    Ot = np.array([0, 0, obs_side + obs_side / np.sqrt(2)])

    fig = plt.figure()
    ax = plt.axes(projection='3d')

    x = np.array([Om4[0], Om1[0], Ot[0], Om1[0], Ob1[0], Om1[0], Om2[0]])
    y = np.array([Om4[1], Om1[1], Ot[1], Om1[1], Ob1[1], Om1[1], Om2[1]])
    z = np.array([Om4[2], Om1[2], Ot[2], Om1[2], Ob1[2], Om1[2], Om2[2]])
    ax.plot(x, y, z, label='parametric curve', color='blue')
    x = np.array([Ot[0], Om4[0], Ob4[0], Om4[0], Om3[0], Ot[0], Om3[0], Ob3[0]])
    y = np.array([Ot[1], Om4[1], Ob4[1], Om4[1], Om3[1], Ot[1], Om3[1], Ob3[1]])
    z = np.array([Ot[2], Om4[2], Ob4[2], Om4[2], Om3[2], Ot[2], Om3[2], Ob3[2]])
    ax.plot(x, y, z, label='parametric curve', color='blue')
    # ax.legend()

    for i in range(30):
        print('------------------------------------')
        print(i)
        y = 15 - 1 * i
        B = np.array([15, y, 18])
        separation2, collision2 = calculate_separation_1(A2, B, Om1, Ob1)
        separation3, collision3 = calculate_separation_2(A3, B, Om4, Om1, Ot, shared=1)
        if separation2[0, 2] > Om1[2]:
            separation2, collision2 = calculate_separation_2(A2, B, Ot, Om4, Om1, shared=3)
        if collision3 == 0:
            separation3, collision3 = calculate_separation_1(A3, B, Ob4, Om4)

        print(B)
        if collision2 == 'both':
            x = np.array([A2[0], separation2[0, 0], separation2[1, 0], B[0]])
            y = np.array([A2[1], separation2[0, 1], separation2[1, 1], B[1]])
            z = np.array([A2[2], separation2[0, 2], separation2[1, 2], B[2]])
            ax.plot(x, y, z, label='parametric curve', color='red')
        else:
            x = np.array([A2[0], separation2[0, 0], B[0]])
            y = np.array([A2[1], separation2[0, 1], B[1]])
            z = np.array([A2[2], separation2[0, 2], B[2]])
            ax.plot(x, y, z, label='parametric curve', color='red')
        if collision3 == 'both':
            x = np.array([A3[0], separation3[0, 0], separation3[1, 0], B[0]])
            y = np.array([A3[1], separation3[0, 1], separation3[1, 1], B[1]])
            z = np.array([A3[2], separation3[0, 2], separation3[1, 2], B[2]])
            ax.plot(x, y, z, label='parametric curve', color='yellow')
        else:
            x = np.array([A3[0], separation3[0, 0], B[0]])
            y = np.array([A3[1], separation3[0, 1], B[1]])
            z = np.array([A3[2], separation3[0, 2], B[2]])
            ax.plot(x, y, z, label='parametric curve', color='yellow')

    for i in range(30):
        print('------------------------------------')
        print(i)
        x = 15 - 1 * i
        B = np.array([x, -15, 18])
        separation2, collision2 = calculate_separation_3(A2, B, Ot, Om4, Ob4, Om1, shared1=4)
        # separation2, collision2 = calculate_separation_3(A2, B, Ot, Om3, Om4, Om1, shared1=1)
        separation3, collision3 = calculate_separation_1(A3, B, Om4, Om1)
        if not collision2:
            separation2, collision2 = calculate_separation_3(A2, B, Ot, Om3, Om4, Om1, shared1=1)
            # separation2, collision2 = calculate_separation_2(A2, B, Om3, Om4, Ot, shared=1)
            if not (collision2 == '1&2'):
                # separation2, collision2 = calculate_separation_1(A2, B, Ob3, Om3)
                separation2, collision2 = calculate_separation_2(A2, B, Om3, Om4, Ot, shared=1)
                if not collision2:
                    separation2, collision2 = calculate_separation_1(A2, B, Ob3, Om3)

        print(B)
        print(collision2)
        print(calculate_cable_length(A2, B, separation2))
        print(np.linalg.norm(A2 - B))

        if collision2 == 'all':
            x = np.array([A2[0], separation2[0, 0], separation2[1, 0], separation2[2, 0], B[0]])
            y = np.array([A2[1], separation2[0, 1], separation2[1, 1], separation2[2, 1], B[1]])
            z = np.array([A2[2], separation2[0, 2], separation2[1, 2], separation2[2, 2], B[2]])
            ax.plot(x, y, z, label='parametric curve', color='red')
        elif collision2 == '1&2' or collision2 == '2&3' or collision2 == 'both':
            print('sep: {}'.format(separation2))
            x = np.array([A2[0], separation2[0, 0], separation2[1, 0], B[0]])
            y = np.array([A2[1], separation2[0, 1], separation2[1, 1], B[1]])
            z = np.array([A2[2], separation2[0, 2], separation2[1, 2], B[2]])
            ax.plot(x, y, z, label='parametric curve', color='red')
        else:
            x = np.array([A2[0], separation2[0, 0], B[0]])
            y = np.array([A2[1], separation2[0, 1], B[1]])
            z = np.array([A2[2], separation2[0, 2], B[2]])
            ax.plot(x, y, z, label='parametric curve', color='red')

        # x = np.array([A3[0], separation3[0, 0], B[0]])
        # y = np.array([A3[1], separation3[0, 1], B[1]])
        # z = np.array([A3[2], separation3[0, 2], B[2]])
        # ax.plot(x, y, z, label='parametric curve', color='yellow')

    plt.show()






