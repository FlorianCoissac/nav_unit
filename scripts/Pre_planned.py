#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseWithCovariance, Pose, PoseArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from numpy import arctan2, pi, sqrt, cos
import tf

def publish_path():
    pub = rospy.Publisher('/trajectory', PoseArray, queue_size=1)
    rospy.init_node('path_publisher', anonymous=True)
    positions = [(2.0, 0.0), (4.0, -2.0), (4.5, -1.0), (5.5, -1.0)]
    traj_list = [Pose() for _ in range(len(positions))]
    for k in range(len(traj_list)):
        traj_list[k].position.x, traj_list[k].position.y = positions[k][0], positions[k][1]
    header=Header()
    pub.publish(header, traj_list)

if __name__ == '__main__':
    try:
        publish_path()
    except rospy.ROSInterruptException:
        pass