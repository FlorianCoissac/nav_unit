#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovariance, Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from numpy import arctan2, pi, sqrt, cos

class Navigator():
    def __init__(self):
        rospy.init_node('navigation', anonymous=True)
        self.vel_publisher = rospy.Publisher('cmd_vel/managed', Twist, queue_size=1)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.update_pos)
        self.rate = rospy.Rate(10) # 10hz
        self.pos_with_co = PoseWithCovariance()
        self.pos = Pose()
        self.rate = rospy.Rate(5)
        self.going_to_goal = False

    def update_pos(self, odom_data):
        self.pos_with_co = odom_data.pose
        self.pos = self.pos_with_co.pose

    def navigation(self, goal):
        if not self.going_to_goal:
            self.going_to_goal = True
            # Init the Dx, Dy for the while loop check
            Dx = goal.position.x - self.pos.position.x
            Dy = goal.position.y - self.pos.position.y
            vel_msg = Twist()
            while (Dx**2+Dy**2)>0.01 and not rospy.is_shutdown():
                # Compute the vector towards goal
                Dx = goal.position.x - self.pos.position.x
                Dy = goal.position.y - self.pos.position.y
                Da = arctan2(Dy, Dx) - self.pos.orientation.z
                if Da**2 > (pi/6)**2: # If the angle difference is too large, turn without going forward
                    rospy.loginfo("Turning " + str(Da*180/pi) + " degrees")
                    vel_msg.angular.z = min(Da, 1)
                    vel_msg.linear.x = 0
                    self.vel_publisher.publish(vel_msg)
                else:
                    rospy.loginfo("Let's go !")
                    vel_msg.angular.z = min(Da, 1)
                    vel_msg.linear.x = min(sqrt(Dx**2+Dy**2)*cos(Da), 1)
                    self.vel_publisher.publish(vel_msg)
                self.rate.sleep()

            # At that point the position is correct, let's get to the right orientation
            Da = goal.orientation.z - self.pos.orientation.z
            while Da**2>0.01 and not rospy.is_shutdown():
                rospy.loginfo("Reorienting : " + str(Da*180/pi) + "degrees")
                Da = goal.orientation.z - self.pos.orientation.z
                vel_msg.angular.z = min(Da, 1)
                vel_msg.linear.x = 0
                self.vel_publisher.publish(vel_msg)
                self.rate.sleep()

            # Now the robot has reached the goal !
            vel_msg.angular.z = 0
            vel_msg.linear.x = 0
            self.vel_publisher.publish(vel_msg)
            rospy.loginfo("Goal is reached !")
            self.going_to_goal = False


    def listen(self):
        rospy.Subscriber("goal", Pose, self.navigation)
        rospy.spin()

if __name__ == '__main__':
    try:
        nav = Navigator()
        nav.listen()
    except rospy.ROSInterruptException:
        pass
