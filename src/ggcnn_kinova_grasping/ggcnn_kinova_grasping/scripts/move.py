#! /usr/bin/env python

import tools
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import time
import robot_controller

rospy.init_node('move')
manipulator = robot_controller.Ur3("192.168.1.211", 30003, 30002)
time.sleep(5)

last_z = 1000
ok = False

box = tools.set_box(manipulator)
start = tools.set_start(manipulator)
table_z = tools.set_table_z(manipulator)
tools.check_joints(manipulator)

def convert_pose():
    pose = PoseStamped()
    pose.pose.position.z = tools.get_pose(manipulator)[2]
    return pose


def move(pose_g):
    global last_z
    global ok
    global time_before
    if not (pose_g.data[0] == 0 or pose_g.data[1] == 0 or pose_g.data[2] <= 0.15 or pose_g.data[2] > last_z):
        tools.move2(pose_g, manipulator, rot_z=0, a=0.04, v=0.2, ip="192.168.1.211", port_write=30003, port_read=30002, check_joits_TF=True)
        last_z = pose_g.data[2]
        time_before = time.time()
        ok = True
    else:
        time_now = time.time()
        if ok == True:
            passed = int(time_now - time_before)
            print passed
            if passed > 5:
                time.sleep(0.5)
                manipulator.grip(0)
                time.sleep(5)
                teraz = tools.get_pose(manipulator)
                teraz[1] -= 10
                teraz[2] -= 10
                trajectory = list()
                trajectory.append(teraz)
                manipulator.move(trajectory, False, a=0.1, v=0.8)
                time.sleep(3)
                tools.goto(box, manipulator)
                tools.goto(start, manipulator)
                last_z = 1000
                ok = False
                tools.check_joints(manipulator)


Pose = rospy.Publisher('/UR5_pose', PoseStamped, queue_size=1)
pose_goal = rospy.Subscriber('/ggcnn/out/command', Float32MultiArray, move, queue_size=1)

while not rospy.is_shutdown():
    rospy.spin()


