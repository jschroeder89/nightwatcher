#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def callback(msg):
    print(msg.pose.pose)

rospy.init_node('odopy')
odom_sub = rospy.Subcriber('amiro1/odom', Odometry, callback)
rospy.spin()