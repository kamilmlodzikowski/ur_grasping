# coding=utf-8

import numpy as np
import robot_controller
import time
from scipy.spatial.transform import Rotation


def move2(pose_g, manipulator, rot_z=0, a=0.08, v=0.5, ip="192.168.1.211", port_write=30003, port_read=30002, check_joits_TF = True):

    akt_pose = get_pose(manipulator)
    akt_z = get_z_orienation(manipulator)
    rot_x = [[1, 0, 0, akt_pose[0]],
            [0, 0.707, 0.707, akt_pose[1]],
            [0, -0.707, 0.707, akt_pose[2]],
            [0, 0, 0, 1]]

    c = np.cos(akt_z-pose_g.data[3])
    s = np.sin(akt_z-pose_g.data[3])
    print ("AKT Z: ", akt_z)

    pose_ggcnn = [[1, 0, 0, -pose_g.data[0]],
                 [0, 1, 0, -pose_g.data[1]+0.06],
                 [0, 0, 1, pose_g.data[2]-0.13],
                 [0, 0, 0, 1]]

    newish_matrix = np.matmul(rot_x, pose_ggcnn)

    orientation_ggcnn = [[c, -s, 0, 0],
                        [s, c, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]]

    new_matrix = np.matmul(newish_matrix, orientation_ggcnn)

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

    # akt_to_round = manipulator.get_pose()
    # pos_akt = np.round(akt_to_round, 3)
    # pos_cel = np.round(pose_goal, 3)
    # time_before = time.time()

    # while pos_akt[0] != pos_cel[0] or pos_akt[1] != pos_cel[1] or pos_akt[2] != pos_cel[2] or pos_akt[3] != pos_cel[3] or pos_akt[4] != pos_cel[4] or pos_akt[5] != pos_cel[5]:
    #     akt_to_round = manipulator.get_pose()
    #     pos_akt = np.round(akt_to_round, 3)
    #     time_now = time.time()
    #     passed = int(time_now - time_before )
    #     if passed > 0.2:
    #         return -1

    return 0


def get_pose(manipulator):
    # manipulator = robot_controller.Ur3(ip, port_write, port_read)
    pose = manipulator.get_pose()
    return pose

def get_z_orienation(manipulator):

    pose = manipulator.get_pose()
    matrix = [[1, 0, 0],
              [0, 0.707, 0.707],
              [0, -0.707, 0.707]]

    inv_matrix = np.linalg.inv(matrix)

    rotation = [pose[3], pose[4], pose[5]]
    r = Rotation.from_rotvec(rotation)
    matrix_from_axis = r.as_dcm()

    tmp_matrix = [[matrix_from_axis[0][0], matrix_from_axis[0][1], matrix_from_axis[0][2]],
                  [matrix_from_axis[1][0], matrix_from_axis[1][1], matrix_from_axis[1][2]],
                  [matrix_from_axis[2][0], matrix_from_axis[2][1], matrix_from_axis[2][2]]]

    tmp_matrix = np.matmul(tmp_matrix, inv_matrix)

    new_r = Rotation.from_dcm(tmp_matrix)

    euler = new_r.as_euler("XYZ", degrees=False)
    z_orientation = euler[2]
    return z_orientation


def get_joints(manipulator):
    joints = manipulator.get_joints()
    return joints


def check_joints(ip="192.168.1.211", port_write=30003, port_read=30002):
    manipulator = robot_controller.Ur3(ip, port_write, port_read)
    joints_read = list(manipulator.get_joints())
    if joints_read[5] > np.pi:
        joints_read[5] -= 2*np.pi
    if joints_read[5] < -np.pi:
        joints_read[5] += 2*np.pi
    joints = np.copy(joints_read)
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


