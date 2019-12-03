#! /usr/bin/env python

import tools
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import time
import robot_controller
print 1
rospy.init_node('movee')
print 2
manipulator = robot_controller.Ur3("192.168.1.211", 30003, 30002)
print 3
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
    print("TESTETETETETETEEST")
    if not (pose_g.data[0] == 0 or pose_g.data[1] == 0 or pose_g.data[2] == 0.15 or last_z - pose_g.data[2] < 0.05):
        tools.move2(pose_g, manipulator, rot_z=0, a=0.01, v=0.05, ip="192.168.1.211", port_write=30003, port_read=30002, check_joits_TF=True)
        last_z = pose_g.data[2]

        # print pose_g.data
        # tools.get_pose(manipulator)
    # else:
    #     msg = rospy.wait_for_message('/ggcnn/out/command', Float32MultiArray)
    #     move(msg)

Pose = rospy.Publisher('/UR5_pose', PoseStamped, queue_size=1)
pose_goal = rospy.Subscriber('/ggcnn/out/command', Float32MultiArray, move, queue_size=1)


# msg1 = rospy.wait_for_message('/ggcnn/out/command', Float32MultiArray)
# move(msg1)
print "babababa"
while not rospy.is_shutdown():
    rospy.spin()


