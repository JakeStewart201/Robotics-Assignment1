#!/usr/bin/env python
from __future__ import division
import rospy
import tf
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

actual_x = 0.0
actual_y = 0.0
est_x = 0.0
est_y = 0.0
mat = 0.0

def calc_dist(x, y, x1, y1):
  return math.sqrt((x - x1) ** 2 + (y - y1) ** 2)

def update_estimate(data):
  global actual_x
  global actual_y
  global test
  try:
    new_p = listener.transformPose('/odom', data)
    x = new_p.pose.position.x
    y = new_p.pose.position.y
    if test:
      print(str(actual_x) + "," + str(actual_y) + "," + str(x) + "," + str(y))
      d = calc_dist(x, y, actual_x, actual_y)
      dist.append(d)
      print(sum(dist)/len(dist))
  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    return
    
def update_truth(data):
  global actual_x
  global actual_y
  global test
  actual_x = data.pose.pose.position.x
  actual_y = data.pose.pose.position.y
  test = True

def init():
  global listener
  global test
  global dist
  test = False
  dist = []
  rospy.init_node('Analysis', anonymous=True)#
  listener = tf.TransformListener()
  rospy.Subscriber('estimatedpose', PoseStamped, update_estimate)
  rospy.Subscriber('base_pose_ground_truth', Odometry, update_truth)
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    rate.sleep()

if __name__ == '__main__':
  try:
    init()
  except rospy.ROSInterruptException:
    pass
