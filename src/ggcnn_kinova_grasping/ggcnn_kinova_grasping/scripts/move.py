
import tools
import rospy
from geometry_msgs.msg import PoseStamped


def convert_pose():
    pose = PoseStamped()
    pose.pose.position.z = tools.get_pose()[2]
    return pose


def move():
    tools.move(pose_goal, rot_z=0, a=0.08, v=0.5, ip="192.168.1.211", port_write=30003, port_read=30002, check_joits_TF=True)


Pose = rospy.Publisher('/UR5_pose', PoseStamped, queue_size=1)
pose_goal = rospy.Subscriber('/ggcnn/out/command', move, queue_size=1)

while not rospy.is_shutdown():
    rospy.spin()


