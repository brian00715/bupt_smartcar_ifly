#!/usr/bin/env python
# coding = utf-8
import rospy
from sensor_msgs.msg import LaserScan
import dynamic_reconfigure.client

def scan_callback(msg):
  range_ahead = msg.ranges[len(msg.ranges)/2]
  print ("range ahead: %0.3f" % range_ahead)

 
rospy.init_node('range_ahead')
scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
rospy.spin()
