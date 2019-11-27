# coding=utf-8
import math
import numpy as np
import robot_controller
import random
import time
from scipy.spatial.transform import Rotation


def move(pose_g, rot_z=0, a=0.08, v=0.5, ip="192.168.1.211", port_write=30003, port_read=30002, check_joits_TF = True):

    manipulator = robot_controller.Ur3(ip, port_write, port_read)
    akt_pose = get_pose()
    pose_matrix = [[akt_pose[0], 0, 0, akt_pose[3]],
                   [0, akt_pose[1], 0, akt_pose[4]],
                   [0, 0, akt_pose[2], akt_pose[5]],
                   [0, 0, 0, 1]]
    pose_gcnn = [[pose_g[0], 0, 0, 0],
                 [0, pose_g[1], 0, 0],
                 [0, 0, pose_g[2], pose_g[5]],
                 [0, 0, 0, 1]]

    new_matrix = np.matmul(pose_matrix, pose_gcnn)

    mat_to_calc = [[new_matrix[0][0], new_matrix[0][1], new_matrix[0][2]],
                   [new_matrix[1][0], new_matrix[1][1], new_matrix[1][2]],
                   [new_matrix[2][0], new_matrix[2][1], new_matrix[2][2]]]
    fi = np.arccos((np.trace(mat_to_calc) - 1) / 2)
    ux = 1 / (2 * np.sin(fi)) * (mat_to_calc[2][1] - mat_to_calc[1][2])
    uy = 1 / (2 * np.sin(fi)) * (mat_to_calc[0][2] - mat_to_calc[2][0])
    uz = 1 / (2 * np.sin(fi)) * (mat_to_calc[1][0] - mat_to_calc[0][1])

    rx = ux * fi
    ry = uy * fi
    rz = uz * fi

    pose_goal = [new_matrix[0][3], new_matrix[1][3], new_matrix[2][3], rx, ry, rz]
    pose_goal = np.transpose(pose_goal)
    trajectory = list()
    trajectory.append(pose_goal)
    manipulator.move(trajectory, False, a=a, v=v)

    akt_to_round = manipulator.get_pose()
    pos_akt = np.round(akt_to_round, 3)
    pos_cel = np.round(pose_goal, 3)
    time_before = time.time()

    while pos_akt[0] != pos_cel[0] or pos_akt[1] != pos_cel[1] or pos_akt[2] != pos_cel[2] or pos_akt[3] != pos_cel[3] or pos_akt[4] != pos_cel[4] or pos_akt[5] != pos_cel[5]:
        akt_to_round = manipulator.get_pose()
        pos_akt = np.round(akt_to_round, 3)
        time_now = time.time()
        passed = int(time_now - time_before )
        if passed > 10:
            return -1
    # if check_joits_TF:
    #     check_joints(ip, port_write, port_read)
    return 0


def get_pose(ip="192.168.1.211", port_write=30003, port_read=30002):
    manipulator = robot_controller.Ur3(ip, port_write, port_read)
    pose = manipulator.get_pose()
    inv_matrix = np.linalg.inv(matrix)

    point = [pose[0], pose[1], pose[2], 1]
    n_point = np.matmul(inv_matrix, point)
    point = np.matmul(inv_matrix, point)
    rotation = [pose[3], pose[4], pose[5]]
    r = Rotation.from_rotvec(rotation)

    matrix_from_axis = r.as_dcm()

    tmp_matrix = [[matrix_from_axis[0][0], matrix_from_axis[0][1], matrix_from_axis[0][2], point[0]],
                  [matrix_from_axis[1][0], matrix_from_axis[1][1], matrix_from_axis[1][2], point[1]],
                  [matrix_from_axis[0][0], matrix_from_axis[0][1], matrix_from_axis[0][2], point[2]],
                  [0, 0, 0, 1]]
    try:
        tmp_matrix = np.linalg.inv(tmp_matrix)
    except:
        tmp_matrix = tmp_matrix
    new_mat = np.matmul(tmp_matrix, inv_matrix)
    #print new_mat

    tmp_matrix2 = [[tmp_matrix[0][0], tmp_matrix[0][1], tmp_matrix[0][2]],
                   [tmp_matrix[1][0], tmp_matrix[1][1], tmp_matrix[1][2]],
                   [tmp_matrix[2][0], tmp_matrix[2][1], tmp_matrix[2][2]]]

    new_r = Rotation.from_dcm(tmp_matrix2)

    euler = new_r.as_euler("XYZ", degrees=False)
    new_pose = [n_point[0], n_point[1], n_point[2], euler[0], euler[1], euler[2]]
    #print ("LALALA: ", new_pose)
    return new_pose


def get_joints(ip="192.168.1.211", port_write=30003, port_read=30002):
    manipulator = robot_controller.Ur3(ip, port_write, port_read)
    joints = manipulator.get_joints()
    return joints


def new_point(point, ip="192.168.1.211", port_write=30003, port_read=30002):
    point.append(1)
    point = np.transpose(point)
    inv_matrix = np.linalg.inv(matrix)
    new_point = np.matmul(inv_matrix, point)
    #print new_point


def random_z(rad_min=-2.5, rad_max=2.5):
    rad_max *= 100
    rad_min *= 100
    rad_z = random.uniform(rad_min, rad_max)
    rad_z /= 100
    # rad_z -= 1.3962634  # radians for pos 0 0 0
    return rad_z


def start_stop(ip="192.168.1.211", port_write=30003, port_read=30002):
    move([-0.4, 0, -0.15], ip=ip, port_read=port_read, port_write=port_write)
    move([0, 0, 0], ip=ip, port_read=port_read, port_write=port_write)


def save_pose():
    array = np.zeros(10)
    print ("Move to bottom minimum X: ")
    raw_input("Press Enter to continue...")
    array[0] = get_pose()[0]
    print ("Move to bottom maximum X: ")
    raw_input("Press Enter to continue...")
    array[1] = get_pose()[0]
    print ("Move to bottom minimum Y: ")
    raw_input("Press Enter to continue...")
    array[2] = get_pose()[1]
    print ("Move to bottom maximum Y: ")
    raw_input("Press Enter to continue...")
    array[3] = get_pose()[1]
    print ("Move to minimum Z")
    raw_input("Press Enter to continue...")
    array[4] = get_pose()[2]
    print ("Move to upper minimum X: ")
    raw_input("Press Enter to continue...")
    array[5] = get_pose()[0]
    print ("Move to upper maximum X: ")
    raw_input("Press Enter to continue...")
    array[6] = get_pose()[0]
    print ("Move to upper minimum Y: ")
    raw_input("Press Enter to continue...")
    array[7] = get_pose()[1]
    print ("Move to upper maximum Y: ")
    raw_input("Press Enter to continue...")
    array[8] = get_pose()[1]
    print ("Move to maximum Z")
    raw_input("Press Enter to continue...")
    array[9] = get_pose()[2]

    return array


def start(ip="192.168.1.211", port_write=30003, port_read=30002):
    print ("Move the arm to goal position")
    raw_input("Press Enter to continue...")
    pose_goal = get_pose(ip, port_write, port_read)
    point = [pose_goal[0], pose_goal[1], pose_goal[2]]
    move(point, pose_goal[5], ip=ip, port_read=port_read, port_write=port_write, check_joits_TF=False)


def check_joints(ip="192.168.1.211", port_write=30003, port_read=30002):
    manipulator = robot_controller.Ur3(ip, port_write, port_read)
    joints_read = list(manipulator.get_joints())
    #print
    #print("JOINTS: ", joints_read)
    if joints_read[5] > np.pi:
        joints_read[5] -= 2*np.pi
    if joints_read[5] < -np.pi:
        joints_read[5] += 2*np.pi

    joints = np.copy(joints_read)

    #print
    #print("JOINTS2: ", joints)
    trajectory = list()
    trajectory.append(joints)
    manipulator.move(trajectory, is_movej=True, is_pose=False, a=0.2, v=0.6)
    pos_akt = manipulator.get_joints()
    pos_akt = np.round(pos_akt, 2)
    joints = np.round(joints, 2)
    time_before = time.time()
    rob = True
    while rob and (pos_akt[0] != joints[0] or pos_akt[1] != joints[1] or pos_akt[2] != joints[2] or pos_akt[3] != joints[3] or pos_akt[4] != joints[4] or pos_akt[5] != joints[5]):
        akt_to_round = manipulator.get_joints()
        pos_akt = np.round(akt_to_round, 2)
        time_now = time.time()
        passed = int(time_now - time_before)
        if passed > 10:
            rob = False
