#! /usr/bin/env python

import tools
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
rospy.init_node('move')


def convert_pose():
    pose = PoseStamped()
    pose.pose.position.z = tools.get_pose()[2]
    return pose


def move(pose_g):
    if not (pose_g.data[0] == 0 or pose_g.data[1] == 0 or pose_g.data[2] <= 0.1):
        tools.move(pose_g, rot_z=0, a=0.01, v=0.03, ip="192.168.1.211", port_write=30003, port_read=30002, check_joits_TF=True)
        tools.get_pose()
    # else:
    #     msg = rospy.wait_for_message('/ggcnn/out/command', Float32MultiArray)
    #     move(msg)

Pose = rospy.Publisher('/UR5_pose', PoseStamped, queue_size=1)
pose_goal = rospy.Subscriber('/ggcnn/out/command', Float32MultiArray, move, queue_size=1)


# msg1 = rospy.wait_for_message('/ggcnn/out/command', Float32MultiArray)
# move(msg1)

while not rospy.is_shutdown():
    rospy.spin()


