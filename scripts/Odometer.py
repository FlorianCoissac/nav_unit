#!/usr/bin/env python3
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseWithCovariance, Pose
from geometry_msgs.msg import TwistWithCovariance, Twist
from razor_imu_9dof.msg import orientation
from nav_msgs.msg import Odometry
from numpy import arctan2, pi, sqrt, cos, sin

class Odometer():
    # This class works as a short term solution to getting a odom_complete topic using the /odom topic's
    # incomplete information and the imu's yaw estimate
    def __init__(self):
        rospy.init_node('odometer', anonymous=True)
        self.odom_publisher = rospy.Publisher('/odom_complete', Odometry, queue_size=1)
        self.imu_subscriber = rospy.Subscriber('orientation', orientation, self.update_imu)
        self.odom_complete_msg = Odometry()
        self.odumb_msg = Odometry()
        self.imu_msg = orientation()
        self.got_new_imu=False
        self.previous_time = rospy.get_rostime().to_sec()

    def update_odom(self, odom_data):
        self.odumb_msg = odom_data
        self.broadcast()

    def update_imu(self, imu_data):
        self.imu_msg = imu_data
        self.got_new_imu = True

    def broadcast(self):
        now = rospy.get_rostime().to_sec()
        self.dt = now-self.previous_time
        self._previous_time = now
        if self.got_new_imu:
            self.odom_complete_msg.pose.pose.orientation.z = self.imu_msg.yaw
        cur_vel = self.odumb_msg.twist.twist.linear.x
        yaw = self.odom_complete_msg.pose.pose.orientation.z
        self.odom_complete_msg.pose.pose.position.x += cur_vel*cos(yaw)*self.dt
        self.odom_complete_msg.pose.pose.position.y += cur_vel*sin(yaw)*self.dt
        self.odom_publisher.publish(self.odom_complete_msg)

    def listen(self):
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.update_odom)
        rospy.spin()
            

if __name__ == '__main__':
    try:
        odometer = Odometer()
        # Do something
        odometer.listen()
    except rospy.ROSInterruptException:
        pass
