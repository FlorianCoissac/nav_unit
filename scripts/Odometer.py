#!/usr/bin/env python3
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseWithCovariance, Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from numpy import arctan2, pi, sqrt, cos

class Odometer():
    # This class works as a short term solution to getting a odom_complete topic using the /odom topic's
    # incomplete information and the imu's yaw estimate
    def __init__(self):
        rospy.init_node('odometer', anonymous=True)
        self.odom_publisher = rospy.Publisher('odom_complete', Odometry, queue_size=1)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.update_odom)
        self.imu_subscriber = rospy.Subscriber('imu', Imu, self.update_imu)
        self.rate = rospy.Rate(10) # 10hz
        self.odom_complete = Odometry()
        self.odumb = Odometry()
        self.imu = Imu()
        self.got_Imu=False

    def update_odom(self, odom_data):
        self.odumb = odom_data

    def update_imu(self, imu_data):
        self.imu = imu_data
        self.got_imu = True

    def broadcast(self):
        while not rospy.is_shutdown():
            if self.got_imu:
                self.odom_complete.pose.pose.orientation.z = self.imu.yaw
            self.odom_publisher.publish(self.odom_complete)
            self.rate.sleep()
            

if __name__ == '__main__':
    try:
        odometer = Odometer()
        # Do something
        odometer.broadcast()
    except rospy.ROSInterruptException:
        pass
