#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseWithCovariance, Pose
from geometry_msgs.msg import TwistWithCovariance, Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from numpy import arctan2, pi, sqrt, cos, sin
import tf

class Odometer():
    # This class works as a short term solution to getting a odom_combined topic using the /odom topic's
    # wheel encoder-based estimate and the imu's magnetometer yaw estimate
    def __init__(self):
        rospy.init_node('odometer', anonymous=True)
        self.odom_publisher = rospy.Publisher('/odom_combined', Odometry, queue_size=1)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.update_odom)
        self.imu_subscriber = rospy.Subscriber('/imu', Imu, self.update_imu)
        self.odom_combined_msg = Odometry()
        self.odom_msg = Odometry()
        self.imu_msg = Imu()
        self.imu_yaw = 0.0
        self.previous_time = rospy.get_rostime().to_sec()
        self.pos_x = 0
        self.pos_y = 0
        self.yaw = 0
        self.init_yaw = 0
        self.rate = rospy.Rate(30)

    def update_odom(self, odom_data): # After updating odometry, broadcast()
        self.odom_msg = odom_data

    def update_imu(self, imu_data): # Whenever a new orientation is published by the imu, get it
        self.imu_msg = imu_data
        explicit_quat = [self.imu_msg.orientation.x, self.imu_msg.orientation.y, self.imu_msg.orientation.z, self.imu_msg.orientation.w]
        r, p, self.imu_yaw = tf.transformations.euler_from_quaternion(explicit_quat)
        if self.init_yaw == 0:
            self.init_yaw = self.imu_yaw

    def broadcast(self): # Publish odom_combined whenever odom is received
        rospy.loginfo("Initiated")
        while not rospy.is_shutdown():
            #Update time estimate
            now = rospy.get_rostime().to_sec()
            self.dt = now-self.previous_time
            self.previous_time = now

            if self.odom_msg.twist.twist.angular.z**2+self.odom_msg.twist.twist.linear.x**2<0.0001:
                # If motors are off orientation is imu's yaw
                self.yaw = self.imu_yaw - self.init_yaw
            else:
                # Then rover is turning around
                self.yaw += self.odom_msg.twist.twist.angular.z*self.dt

            # Now update linear
            cur_vel = self.odom_msg.twist.twist.linear.x
            # Simple estimate for the updated position
            self.pos_x += cur_vel*cos(self.yaw)*self.dt
            self.pos_y += cur_vel*sin(self.yaw)*self.dt
            self.odom_combined_msg.pose.pose.position.x = self.pos_x
            self.odom_combined_msg.pose.pose.position.y = self.pos_y
            q_tmp = tf.transformations.quaternion_from_euler(0.0,0.0,self.yaw)
            self.odom_combined_msg.pose.pose.orientation.x = q_tmp[0]
            self.odom_combined_msg.pose.pose.orientation.y = q_tmp[1]
            self.odom_combined_msg.pose.pose.orientation.z = q_tmp[2]
            self.odom_combined_msg.pose.pose.orientation.w = q_tmp[3]
            self.odom_combined_msg.twist=self.odom_msg.twist
            self.odom_publisher.publish(self.odom_combined_msg)
            self.rate.sleep()
            rospy.loginfo("Broadcasting")
            


odometer = Odometer()
# Do something
odometer.broadcast()