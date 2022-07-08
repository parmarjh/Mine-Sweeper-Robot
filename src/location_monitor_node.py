#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

metal_voltage = 0

def md_callback(data):
	metal_voltage = data.data

def position_callback(msg):
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	rospy.Subscriber("/md_volt",String,md_callback)
	if metal_voltage > 4.5:
	   print('Robot find the mine! The location is posted:')
	   rospy.loginfo('x: {}, y: {}'.format(x,y))

def main():
	rospy.init_node('location_monitor',anonymous=True)
	rospy.Subscriber("/odom",Odometry, position_callback)
	rospy.spin()

if __name__ == '__main__':
	main()
