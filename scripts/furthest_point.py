#!/usr/bin/env python
from __future__ import division
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

TOO_CLOSE_TO_WALL = 1.0
TURN_AMOUNT = 1
FORWARD_AMOUNT = 0.2
FORWARD_BIAS = 0.3
speed = 0
turn = 0

def update_scanner(data):
  global speed
  global turn
  left = 5.0;
  right = 5.0;
  ahead = 5.0;

  for i, dist in enumerate(data.ranges, start = 1):
    angle = i / len(data.ranges)
    if (angle < 0.3):
      right = min(right, dist)
    elif (angle < 0.7):
      ahead = min(ahead, dist)
    else:
      left = min(left, dist)

  turn = 0
  ahead += FORWARD_BIAS

  if (ahead <= TOO_CLOSE_TO_WALL):
    speed = 0
  else:
    speed = FORWARD_AMOUNT

  rospy.loginfo("left:" + str(left) + ", ahead:" + str(ahead) + ", right:" + str(right))
  if (left > ahead):
    if (left > right):
      turn = TURN_AMOUNT
    else:
      turn = -TURN_AMOUNT
  else:
    if (ahead < right):
      turn = -TURN_AMOUNT

  if (ahead <= TOO_CLOSE_TO_WALL and left <= TOO_CLOSE_TO_WALL):
    turn = -TURN_AMOUNT
    

def talker():
  pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
  rospy.init_node('Mover', anonymous=True)#
  rospy.Subscriber('base_scan', LaserScan, update_scanner)
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    base_data = Twist()
    base_data.linear.x = speed
    base_data.angular.z = turn
    pub.publish( base_data )#
    rate.sleep()

if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass
