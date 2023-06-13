#!/usr/bin/env python
#Exercise 2: Trace a square by driving the robot

import rospy
import sys
import math
from geometry_msgs.msg import Twist

def publisher():
	pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
	rospy.init_node('Walker', anonymous=True)
	rate = rospy.Rate(10) #10hz
	#Loop until rospy is shutdown when crtl-c
	while not rospy.is_shutdown():
		#A twist message for moving the robot is created
		desired_velocity_move = Twist()
		desired_velocity_move.linear.x = 0.2 #linear velocity with 0.2 m/sec.
		for i in range(0,40):
			#Publish velocity move
			pub.publish(desired_velocity_move)
			rate.sleep()
		#A twist message for turning the robot at right angle is created
		desired_velocity_turn = Twist()
		#Math.radians converts degree value 90 into radians
		#Turn robot in order to trace a square
		desired_velocity_turn.angular.z = math.radians(90)
		for i in range(0,10):
			#Publish velocity turn
			pub.publish(desired_velocity_turn)
			rate.sleep()

if __name__ == "__main__":
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
