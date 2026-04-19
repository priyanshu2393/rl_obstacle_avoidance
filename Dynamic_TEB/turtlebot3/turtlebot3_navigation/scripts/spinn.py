#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import time

class InitialPoseSpinner:
    def __init__(self):
        rospy.init_node('initial_pose_spinner')

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.pose_callback)

        self.done = False
        rospy.loginfo("Waiting for initial pose...")

    def pose_callback(self, msg):
        if self.done:
            return

        rospy.loginfo("Initial pose received. Rotating robot...")

        twist = Twist()
        twist.angular.z = 0.5  # rotation speed

        duration = 12  # ~360 deg (tune this)

        start_time = time.time()
        rate = rospy.Rate(10)

        while time.time() - start_time < duration and not rospy.is_shutdown():
            self.cmd_pub.publish(twist)
            rate.sleep()

        # Stop robot
        twist.angular.z = 0
        self.cmd_pub.publish(twist)

        rospy.loginfo("Rotation complete.")
        self.done = True


if __name__ == '__main__':
    InitialPoseSpinner()
    rospy.spin()