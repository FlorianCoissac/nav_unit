#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from numpy import arctan2, pi, sqrt, cos
import tf

class Navigator():
    def __init__(self):
        rospy.init_node('navigator', anonymous=True)
        self.vel_publisher = rospy.Publisher('cmd_vel/managed', Twist, queue_size=1)
        self.odom_subscriber = rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.update_pos)
        self.rate = rospy.Rate(30) # 10hz
        self.pos_with_co = PoseWithCovarianceStamped()
        self.pos = Pose()
        self.goals=[]
        self.going_to_goal = False
        self.interrupt = False

    def update_pos(self, odom_data):
        self.pos_with_co = odom_data.pose
        self.pos = self.pos_with_co.pose

    def got_new_trajectory(self, trajectory):
        if not self.going_to_goal:
            self.goals = trajectory.poses
            self.follow_trajectory()
        else:
            self.interrupt = True
            rospy.sleep(0.5)
            self.interrupt = False
            self.goals = trajectory.poses
            self.follow_trajectory()

    def got_new_goal(self, goal):
        if not self.going_to_goal:
            self.go_to_goal(goal)
        else:
            self.interrupt = True
            rospy.sleep(0.5)
            self.interrupt = False
            self.go_to_goal(goal)
        
    def follow_trajectory(self):
        n_goal = 0
        while n_goal<len(self.goals):
            goal = self.goals[n_goal]
            succeeded = self.go_to_goal(goal)
            if succeeded:
                rospy.loginfo("Goal reached : " + str(n_goal))
                n_goal+=1
            else:
                self.goals = []
                rospy.loginfo("Error : Trajectory supressed")
        if succeeded:
            rospy.loginfo("Trajectory completed")
        else:
            rospy.loginfo("Trajectory failed")



    def go_to_goal(self, goal):
        if not self.going_to_goal:
            self.going_to_goal = True
            # Init the Dx, Dy for the while loop check
            Dx = goal.position.x - self.pos.position.x
            Dy = goal.position.y - self.pos.position.y
            vel_msg = Twist()
            while (Dx**2+Dy**2)>0.01 and not rospy.is_shutdown() and not self.interrupt:
                # Compute the vector towards goal
                explicit_quat = [self.pos.orientation.x, self.pos.orientation.y, self.pos.orientation.z, self.pos.orientation.w]
                roll, pitch, yaw = tf.transformations.euler_from_quaternion(explicit_quat)
                Dx = goal.position.x - self.pos.position.x
                Dy = goal.position.y - self.pos.position.y
                Da = self.transfo2pipi(arctan2(Dy, Dx) - yaw)
                if Da**2 > (pi/6)**2: # If the angle difference is too large, turn without going forward
                    rospy.loginfo("Turning " + str(Da*180/pi) + " degrees")
                    vel_msg.angular.z = Da/min(sqrt(Da**2), 1)
                    vel_msg.linear.x = 0
                    self.vel_publisher.publish(vel_msg)
                else:
                    rospy.loginfo("Let's go ! Rotation = " + str(Da*180/pi) + " degrees")
                    vel_msg.angular.z = Da/min(sqrt(Da**2), 1)
                    vel_msg.linear.x = min(sqrt(Dx**2+Dy**2)*cos(Da), 1)
                    self.vel_publisher.publish(vel_msg)
                self.rate.sleep()

            # At that point the position is correct, let's get to the right orientation
            explicit_quat = [self.pos.orientation.x, self.pos.orientation.y, self.pos.orientation.z, self.pos.orientation.w]
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(explicit_quat)
            explicit_quat = [goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w]
            goal_roll, goal_pitch, goal_yaw = tf.transformations.euler_from_quaternion(explicit_quat)
            Da = goal_yaw - yaw
            while Da**2>0.01 and not rospy.is_shutdown() and not self.interrupt:
                explicit_quat = [self.pos.orientation.x, self.pos.orientation.y, self.pos.orientation.z, self.pos.orientation.w]
                roll, pitch, yaw = tf.transformations.euler_from_quaternion(explicit_quat)
                explicit_quat = [goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w]
                goal_roll, goal_pitch, goal_yaw = tf.transformations.euler_from_quaternion(explicit_quat)
                Da = self.transfo2pipi(goal_yaw - yaw)
                rospy.loginfo("Reorienting : " + str(Da*180/pi) + "degrees")
                vel_msg.angular.z = Da/min(sqrt(Da**2), 1)
                vel_msg.linear.x = 0
                self.vel_publisher.publish(vel_msg)
                self.rate.sleep()

            # Now the robot has reached the goal !
            vel_msg.angular.z = 0
            vel_msg.linear.x = 0
            self.vel_publisher.publish(vel_msg)
            rospy.loginfo("Goal is reached !")
            self.going_to_goal = False
            return True


    def listen(self):
        while not True in ['/robot_pose_ekf/odom_combined' in tops for tops in rospy.get_published_topics()] and not rospy.is_shutdown():
            rospy.logwarn("Navigator waiting for robot_pose_ekf to launch (topic robot_pose_ekf/odom_combined not received")
            rospy.sleep(1)
        while not True in ['/imu' in tops for tops in rospy.get_published_topics()] and not rospy.is_shutdown():
            rospy.logwarn("Navigator waiting for Imu to launch (topic /imu not received)")
            rospy.sleep(1)
        rospy.loginfo("Navigator is now launching and listening for goal input")
        rospy.Subscriber("/trajectory", PoseArray, self.got_new_trajectory)
        rospy.Subscriber("/goal", Pose, self.got_new_goal)
        rospy.spin()

    def transfo2pipi(self, a):
        a = a%(2*pi)
        if a>pi:
            a = a-2*pi
        return a

nav = Navigator()
nav.listen()
