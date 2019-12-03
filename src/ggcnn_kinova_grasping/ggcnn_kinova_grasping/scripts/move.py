#! /usr/bin/env python

import tools
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import time
import robot_controller

rospy.init_node('movee')
manipulator = robot_controller.Ur3("192.168.1.211", 30003, 30002)
time.sleep(5)

last_z = 1000
suma = 0
i = 0
print "lalala"


def convert_pose():
    pose = PoseStamped()
    pose.pose.position.z = tools.get_pose(manipulator)[2]
    return pose


def move(pose_g):
    global last_z
    global time_before
    if not (pose_g.data[0] == 0 or pose_g.data[1] == 0 or pose_g.data[2] == 0.15 or last_z - pose_g.data[2] < 0.05):
        tools.move2(pose_g, manipulator, rot_z=0, a=0.01, v=0.05, ip="192.168.1.211", port_write=30003, port_read=30002, check_joits_TF=True)
        last_z = pose_g.data[2]
        time_before = time.time()
    else:
        time_now = time.time()
        passed = int(time_now - time_before)
        if passed > 3:
            manipulator.grip(5)


Pose = rospy.Publisher('/UR5_pose', PoseStamped, queue_size=1)
pose_goal = rospy.Subscriber('/ggcnn/out/command', Float32MultiArray, move, queue_size=1)


# msg1 = rospy.wait_for_message('/ggcnn/out/command', Float32MultiArray)
# move(msg1)
print "babababa"
while not rospy.is_shutdown():
    rospy.spin()


