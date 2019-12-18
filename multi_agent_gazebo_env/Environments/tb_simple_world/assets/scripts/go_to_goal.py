#! /usr/bin/env python
from __future__ import division

import rospy
from math import pi, asin, acos
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool, Int8
import time
import goal_controller
import tf

class GoToGoal:

    def __init__(self, namespace):
        self.ns = namespace
        self.controller = goal_controller.GoalController()
        rospy.init_node('{}_go2goal'.format(self.ns))
        self.twistPub = rospy.Publisher('{}/cmd_vel'.format(self.ns), Twist, queue_size=10)
        self.stopMotor = rospy.Publisher('{}/goalReached'.format(self.ns), Int8, queue_size=10)
        rospy.Subscriber('{}/ground_truth_pose'.format(self.ns), Odometry, self.odomCallback)
        rospy.Subscriber('{}/goal'.format(self.ns), PoseStamped, self.goalCallback)
        self.goal_set = False

    def update(self):
        twist = Twist()
        self.controller.polarize()
        print(self.ns, self.controller.state)
        if self.controller.at_goal():
            # rospy.loginfo("Goal Reached")
            twist.linear.x = 0
            twist.angular.z = 0
        else:
            twist.linear.x, twist.angular.z = self.controller.get_control_op()
            print(twist.linear.x, twist.angular.z)
        self.twistPub.publish(twist)

    def odomCallback(self, newPose):
        pos = newPose.pose.pose.position
        orientation = newPose.pose.pose.orientation
        qt = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = tf.transformations.euler_from_quaternion(qt)
        self.controller.set_pose(x=pos.x, y=pos.y, t=euler[2])
        if not self.goal_set:
            self.controller.set_goal(x=pos.x, y=pos.y, t=euler[2])

    def goalCallback(self, goal):
        pos = goal.pose.position
        orientation = goal.pose.orientation
        qt = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = tf.transformations.euler_from_quaternion(qt)
        self.controller.set_goal(x=pos.x, y=pos.y, t=euler[2])
        self.goal_set = True

    def spin(self):
        rospy.loginfo("Goal Point Set")
        rate = rospy.Rate(100)
        rospy.on_shutdown(self.shutdown)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Goal Interrupted")
        self.twistPub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    import sys
    ns = sys.argv[1]
    gotoGoal = GoToGoal(ns)
    gotoGoal.spin()