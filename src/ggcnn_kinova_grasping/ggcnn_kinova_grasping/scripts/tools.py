# coding=utf-8

import numpy as np
import robot_controller
import time
import socket
import move

socket_write = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


def move2(pose_g, manipulator, rot_z=0, a=0.08, v=0.5, ip="192.168.1.211", port_write=30003, port_read=30002, check_joits_TF = True):
    c = np.cos(pose_g.data[3])
    s = np.sin(pose_g.data[3])

    move.suma += pose_g.data[4]
    move.i += 1
    akt_pose = get_pose(manipulator)

    rotz = [[1, 0, 0, akt_pose[0]],
            [0, 0.707, 0.707, akt_pose[1]],
            [0, -0.707, 0.707, akt_pose[2]],
            [0, 0, 0, 1]]

    pose_gcnn = [[1, 0, 0, -pose_g.data[0]],
                 [0, 1, 0, -pose_g.data[1]+0.06],
                 [0, 0, 1, pose_g.data[2]-0.13],
                 [0, 0, 0, 1]]

    new_matrix = np.matmul(rotz, pose_gcnn)

    # translation = [[c, -s, 0, pose_g.data[0]+0.035],
    #              [s, c, 0, -pose_g.data[1]+0.08],
    #              [0, 0, 1, pose_g.data[2]-0.13],
    #              [0, 0, 0, 1]]

    # new_matrix = np.matmul(new_matrix, translation)

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

    # while pos_akt[0] != pos_cel[0] or pos_akt[1] != pos_cel[1] or pos_akt[2] != pos_cel[2] or pos_akt[3] != pos_cel[3] or pos_akt[4] != pos_cel[4] or pos_akt[5] != pos_cel[5]:
    #     akt_to_round = manipulator.get_pose()
    #     pos_akt = np.round(akt_to_round, 3)
    #     time_now = time.time()
    #     passed = int(time_now - time_before )
    #     if passed > 0.2:
    #         return -1

    while pos_akt[0] == pos_cel[0] and pos_akt[1] == pos_cel[1] and pos_akt[2] == pos_cel[2]:
        time_now = time.time()
        passed = int(time_now - time_before )
        if passed > 3:
            srednia = move.suma/move.i
            grip(srednia)

        else:
            return -1

    return 0


def grip(range_open):
    assert isinstance(range_open, float) or isinstance(range_open, int)
    command = rg6_cmd(range_open)
    socket_write.send(command)


def get_pose(manipulator):
    # manipulator = robot_controller.Ur3(ip, port_write, port_read)
    pose = manipulator.get_pose()
    return pose


def get_joints(ip="192.168.1.211", port_write=30003, port_read=30002):
    manipulator = robot_controller.Ur3(ip, port_write, port_read)
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


def rg6_cmd(range_open, force=50):
    cmd_str = "def rg6ProgOpen():\n";
    cmd_str += "\ttextmsg(\"inside RG6 function called\")\n";

    cmd_str += "\ttarget_width={0}\n".format(range_open);
    cmd_str += "\ttarget_force={0}\n".format(force);
    cmd_str += "\tpayload=1.0\n";
    cmd_str += "\tset_payload1=False\n";
    cmd_str += "\tdepth_compensation=False\n";
    cmd_str += "\tslave=False\n";

    cmd_str += "\ttimeout = 0\n";
    cmd_str += "\twhile get_digital_in(9) == False:\n";
    cmd_str += "\t\ttextmsg(\"inside while\")\n";
    cmd_str += "\t\tif timeout > 400:\n";
    cmd_str += "\t\t\tbreak\n";
    cmd_str += "\t\tend\n";
    cmd_str += "\t\ttimeout = timeout+1\n";
    cmd_str += "\t\tsync()\n";
    cmd_str += "\tend\n";
    cmd_str += "\ttextmsg(\"outside while\")\n";

    cmd_str += "\tdef bit(input):\n";
    cmd_str += "\t\tmsb=65536\n";
    cmd_str += "\t\tlocal i=0\n";
    cmd_str += "\t\tlocal output=0\n";
    cmd_str += "\t\twhile i<17:\n";
    cmd_str += "\t\t\tset_digital_out(8,True)\n";
    cmd_str += "\t\t\tif input>=msb:\n";
    cmd_str += "\t\t\t\tinput=input-msb\n";
    cmd_str += "\t\t\t\tset_digital_out(9,False)\n";
    cmd_str += "\t\t\telse:\n";
    cmd_str += "\t\t\t\tset_digital_out(9,True)\n";
    cmd_str += "\t\t\tend\n";
    cmd_str += "\t\t\tif get_digital_in(8):\n";
    cmd_str += "\t\t\t\tout=1\n";
    cmd_str += "\t\t\tend\n";
    cmd_str += "\t\t\tsync()\n";
    cmd_str += "\t\t\tset_digital_out(8,False)\n";
    cmd_str += "\t\t\tsync()\n";
    cmd_str += "\t\t\tinput=input*2\n";
    cmd_str += "\t\t\toutput=output*2\n";
    cmd_str += "\t\t\ti=i+1\n";
    cmd_str += "\t\tend\n";
    cmd_str += "\t\treturn output\n";
    cmd_str += "\tend\n";
    cmd_str += "\ttextmsg(\"outside bit definition\")\n";

    cmd_str += "\ttarget_width=target_width+0.0\n";
    cmd_str += "\tif target_force>40:\n";
    cmd_str += "\t\ttarget_force=40\n";
    cmd_str += "\tend\n";

    cmd_str += "\tif target_force<4:\n";
    cmd_str += "\t\ttarget_force=4\n";
    cmd_str += "\tend\n";
    cmd_str += "\tif target_width>150:\n";
    cmd_str += "\t\ttarget_width=150\n";
    cmd_str += "\tend\n";
    cmd_str += "\tif target_width<0:\n";
    cmd_str += "\t\ttarget_width=0\n";
    cmd_str += "\tend\n";
    cmd_str += "\trg_data=floor(target_width)*4\n";
    cmd_str += "\trg_data=rg_data+floor(target_force/2)*4*111\n";
    cmd_str += "\tif slave:\n";
    cmd_str += "\t\trg_data=rg_data+16384\n";
    cmd_str += "\tend\n";

    cmd_str += "\ttextmsg(\"about to call bit\")\n";
    cmd_str += "\tbit(rg_data)\n";
    cmd_str += "\ttextmsg(\"called bit\")\n";

    cmd_str += "\tif depth_compensation:\n";
    cmd_str += "\t\tfinger_length = 55.0/1000\n";
    cmd_str += "\t\tfinger_heigth_disp = 5.0/1000\n";
    cmd_str += "\t\tcenter_displacement = 7.5/1000\n";

    cmd_str += "\t\tstart_pose = get_forward_kin()\n";
    cmd_str += "\t\tset_analog_inputrange(2, 1)\n";
    cmd_str += "\t\tzscale = (get_analog_in(2)-0.026)/2.976\n";
    cmd_str += "\t\tzangle = zscale*1.57079633-0.087266462\n";
    cmd_str += "\t\tzwidth = 5+110*sin(zangle)\n";

    cmd_str += "\t\tstart_depth = cos(zangle)*finger_length\n";

    cmd_str += "\t\tsync()\n";
    cmd_str += "\t\tsync()\n";
    cmd_str += "\t\ttimeout = 0\n";

    cmd_str += "\t\twhile get_digital_in(9) == True:\n";
    cmd_str += "\t\t\ttimeout=timeout+1\n";
    cmd_str += "\t\t\tsync()\n";
    cmd_str += "\t\t\tif timeout > 20:\n";
    cmd_str += "\t\t\t\tbreak\n";
    cmd_str += "\t\t\tend\n";
    cmd_str += "\t\tend\n";
    cmd_str += "\t\ttimeout = 0\n";
    cmd_str += "\t\twhile get_digital_in(9) == False:\n";
    cmd_str += "\t\t\tzscale = (get_analog_in(2)-0.026)/2.976\n";
    cmd_str += "\t\t\tzangle = zscale*1.57079633-0.087266462\n";
    cmd_str += "\t\t\tzwidth = 5+110*sin(zangle)\n";
    cmd_str += "\t\t\tmeasure_depth = cos(zangle)*finger_length\n";
    cmd_str += "\t\t\tcompensation_depth = (measure_depth - start_depth)\n";
    cmd_str += "\t\t\ttarget_pose = pose_trans(start_pose,p[0,0,-compensation_depth,0,0,0])\n";
    cmd_str += "\t\t\tif timeout > 400:\n";
    cmd_str += "\t\t\t\tbreak\n";
    cmd_str += "\t\t\tend\n";
    cmd_str += "\t\t\ttimeout=timeout+1\n";
    cmd_str += "\t\t\tservoj(get_inverse_kin(target_pose),0,0,0.008,0.033,1700)\n";
    cmd_str += "\t\tend\n";
    cmd_str += "\t\tnspeed = norm(get_actual_tcp_speed())\n";
    cmd_str += "\t\twhile nspeed > 0.001:\n";
    cmd_str += "\t\t\tservoj(get_inverse_kin(target_pose),0,0,0.008,0.033,1700)\n";
    cmd_str += "\t\t\tnspeed = norm(get_actual_tcp_speed())\n";
    cmd_str += "\t\tend\n";
    cmd_str += "\tend\n";
    cmd_str += "\tif depth_compensation==False:\n";
    cmd_str += "\t\ttimeout = 0\n";
    cmd_str += "\t\twhile get_digital_in(9) == True:\n";
    cmd_str += "\t\t\ttimeout = timeout+1\n";
    cmd_str += "\t\t\tsync()\n";
    cmd_str += "\t\t\tif timeout > 20:\n";
    cmd_str += "\t\t\t\tbreak\n";
    cmd_str += "\t\t\tend\n";
    cmd_str += "\t\tend\n";
    cmd_str += "\t\ttimeout = 0\n";
    cmd_str += "\t\twhile get_digital_in(9) == False:\n";
    cmd_str += "\t\t\ttimeout = timeout+1\n";
    cmd_str += "\t\t\tsync()\n";
    cmd_str += "\t\t\tif timeout > 400:\n";
    cmd_str += "\t\t\t\tbreak\n";
    cmd_str += "\t\t\tend\n";
    cmd_str += "\t\tend\n";
    cmd_str += "\tend\n";
    cmd_str += "\tif set_payload1:\n";
    cmd_str += "\t\tif slave:\n";
    cmd_str += "\t\t\tif get_analog_in(3) < 2:\n";
    cmd_str += "\t\t\t\tzslam=0\n";
    cmd_str += "\t\t\telse:\n";
    cmd_str += "\t\t\t\tzslam=payload\n";
    cmd_str += "\t\t\tend\n";
    cmd_str += "\t\telse:\n";
    cmd_str += "\t\t\tif get_digital_in(8) == False:\n";
    cmd_str += "\t\t\t\tzmasm=0\n";
    cmd_str += "\t\t\telse:\n";
    cmd_str += "\t\t\t\tzmasm=payload\n";
    cmd_str += "\t\t\tend\n";
    cmd_str += "\t\tend\n";
    cmd_str += "\t\tzsysm=0.0\n";
    cmd_str += "\t\tzload=zmasm+zslam+zsysm\n";
    cmd_str += "\t\tset_payload(zload)\n";
    cmd_str += "\tend\n";
    cmd_str += "end\n\n";

    return cmd_str
