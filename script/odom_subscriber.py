#!/usr/bin/env python

import rospy
from ITC.msg import robot_odom

def odom_callback(msg):
	print("speed=%f and position=%f"%(msg.Speed,msg.Position))
	
rospy.init_node("odom_subscriber")

sub=rospy.Subscriber("odoms",robot_odom,odom_callback)

rospy.spin()
