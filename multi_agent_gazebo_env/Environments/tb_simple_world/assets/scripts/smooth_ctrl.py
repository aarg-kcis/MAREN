#########################################################################
#
#              Author: b51
#                Mail: b51live@gmail.com
#            FileName: smooth_ctrl.py
#          Created On: Tue 11/ 7 14:24:13 2017
#
#      Licensed under The MIT License [see LICENSE for details]
#
#########################################################################

#!/usr/bin/env python

import sys
import os
import math
import traceback
import rospy
import time
import tf
from std_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class SmoothCtrl(object):

  def __init__(self, namespace, debug):

    self.k1 = 1;
    self.k2 = 3;
    self.beta = 0.4;
    self.lamda = 2;
    self.v_max = 0.5;
    self.debug = debug

    self.active = False;

    self.home_pose = [0.0, 0.0, 0.0];
    rospy.Subscriber("{}/ground_truth_pose".format(namespace), Odometry, self.PoseCB);
    # currently taking tb_pose
    rospy.Subscriber("{}/move_base_simple/goal".format(namespace), Pose, self.GoalCB_TB);
    self.cmd_vel_pub = rospy.Publisher("{}/cmd_vel".format(namespace), Twist, queue_size = 10);

  def PoseRelative(self, home_pose, pose):
    ca = math.cos(pose[2]);
    sa = math.sin(pose[2]);
    px = home_pose[0]-pose[0];
    py = home_pose[1]-pose[1];
    pa = home_pose[2]-pose[2];
    return [ca*px + sa*py, -sa*px + ca*py, self.mod_angle(pa)];

  def PoseCB(self, msg):
    self.current_pose = [0.0, 0.0, 0.0]
    self.current_pose[0] = msg.pose.pose.position.x;
    self.current_pose[1] = msg.pose.pose.position.y;
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,\
                                                                   msg.pose.pose.orientation.y,\
                                                                   msg.pose.pose.orientation.z,\
                                                                   msg.pose.pose.orientation.w]);
    self.current_pose[2] = self.mod_angle(yaw);
#    rospy.loginfo("current_pose[0] = %f, current_pose[1] = %f, current_pose[2] = %f",\
#                  self.current_pose[0], self.current_pose[1], self.current_pose[2]);
  def GoalCB_TB(self, msg):
    self.home_pose[0] = msg.x
    self.home_pose[1] = msg.y
    self.home_pose[2] = msg.theta
    self.active = True;
    rospy.loginfo("home_pose = {}".format(self.home_pose));
  

  def GoalCB(self, msg):
    self.home_pose[0] = msg.pose.position.x;
    self.home_pose[1] = msg.pose.position.y;
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([msg.pose.orientation.x,\
                                                                   msg.pose.orientation.y,\
                                                                   msg.pose.orientation.z,\
                                                                   msg.pose.orientation.w]);
    self.home_pose[2] = self.mod_angle(yaw);
    self.active = True;
    rospy.loginfo("home_pose[0] = %f, home_pose[1] = %f, home_pose[2] = %f",\
                  self.home_pose[0], self.home_pose[1], self.home_pose[2]);
    pass;

  # Reduce angle to [-pi, pi)
  def mod_angle(self, a):
    if a == None:
      return None
    a = a % (2 * math.pi);
    if (a >= math.pi):
      a = a - 2 * math.pi;
    return a;

  def spin(self):
    while not rospy.is_shutdown():
      if self.active:
        cmd_vel = Twist();

        pose_relative = self.PoseRelative(self.home_pose, self.current_pose);
        dr = math.sqrt(pose_relative[0]**2 + pose_relative[1]**2);

        sigma = self.mod_angle(math.atan2(pose_relative[1], pose_relative[0]));
        theta = self.mod_angle(sigma - pose_relative[2]);

        part1 = self.k2 * (-sigma - math.atan(self.k1 * theta));
        part2 = (1 + self.k1 / (1 + (self.k1 * theta)**2)) * math.sin(-sigma);
        k = (part1 + part2)/-dr;

        v = self.v_max / (1 + self.beta * abs(k)**self.lamda);
        w = k * v;
        if self.debug:
          rospy.loginfo("dr = %f, sigma = %f, theta = %f, k = %f, v = %f, w = %f",\
                       dr, sigma, theta, k, v, w);
        cmd_vel.linear.x = v;
        cmd_vel.angular.z = w;
        self.cmd_vel_pub.publish(cmd_vel);
        rospy.sleep(0.1);

      pass;

if __name__ == '__main__':
    import sys
    import rospy
    try:
      debug = bool(argv[2])
    except Exception as e:
      debug = False
    namespace = sys.argv[1]
    rospy.init_node('{}_smooth_ctrl'.format(namespace));
    SmoothCtrlNode = SmoothCtrl(namespace, debug);
    SmoothCtrlNode.spin();