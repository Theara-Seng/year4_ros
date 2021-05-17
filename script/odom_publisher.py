#!/usr/bin/env python

import rospy
from ITC.msg import robot_odom

rospy.init_node("odom_publisher")
odom=rospy.Publisher("odoms",robot_odom,queue_size=10)

robot_speed=0.0
robot_position=0.0

rate =rospy.Rate(10)

while not rospy.is_shutdown():
	msg=robot_odom()
	msg.Speed=robot_speed
	msg.Position=robot_position
	print("speed=%.2f and position=%.2f"%(msg.Speed,msg.Position))
	odom.publish(msg)
	robot_speed+=2.0
	robot_position=robot_speed/10.0
	rate.sleep()
