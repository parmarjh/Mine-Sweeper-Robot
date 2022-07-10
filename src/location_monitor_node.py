#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
#from geometry_msgs.msg import PoseWithCovarianceStamped

count = 0
metal_voltage = 0

def md_callback(msg):
	global metal_voltage
	metal_voltage = msg.data
	

def position_callback(msg):
	global count 
	global metal_voltage
	x = msg.pose.pose.position.x + 0.3048
	y = msg.pose.pose.position.y + 0.3048		

	rospy.Subscriber("/md_volt",Float32,md_callback)
	
	if metal_voltage > 4.5 and count == 0 :
	   print('Robot find the mine! The location is posted:')
	   rospy.loginfo('x: {}, y: {}'.format(x,y))
	   count = 1
	
	if metal_voltage == 0 and count == 1 :
	   count = 0
	

def main():
	rospy.init_node('location_monitor')
	rospy.Subscriber("/odom",Odometry, position_callback)
	#rospy.Subscriber("/amcl",PoseWithCovarianceStamped, position_callback)	
	rospy.spin()

if __name__ == '__main__':
	main()
