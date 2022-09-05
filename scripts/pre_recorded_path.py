#!/usr/bin/env python3
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist

def pre_recorded_path():
    pub = rospy.Publisher('cmd_vel/managed', Twist, queue_size=10)
    rospy.init_node('pre_recorded_path', anonymous=True)
    rate = rospy.Rate(2) # 10hz

    while not rospy.is_shutdown():
        small_step = Twist()
        small_step.linear.x=1.0; small_step.angular.z=0.0
        for _ in range(5):
            pub.publish(small_step)
        rate.sleep()
        small_step.linear.x=-1.0
        for _ in range(5):
            pub.publish(small_step)
        rate.sleep()
        small_step.linear.x=1.0; small_step.angular.z=-1.0
        for _ in range(5):
            pub.publish(small_step)
        rate.sleep()
        small_step.linear.x=-1.0;
        for _ in range(5):
            pub.publish(small_step)
        rate.sleep()

if __name__ == '__main__':
    try:
        pre_recorded_path()
    except rospy.ROSInterruptException:
        pass
