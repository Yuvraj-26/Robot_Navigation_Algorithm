#!/usr/bin/env python
#Exercise 3: Robot stops when bumber is pressed
#Robot stops if bumper is pressed which indicates robot collision and exercise3.py program ends

#Required imports
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
import rospy
import sys
import math
bump_state = 0

#bumperEventCallback function executes when new topic data arrives
def bumperEventCallback(data):
	global bump_state
	bump_state += 1

def publisher():
	#Publisher used from exercise2
	pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
	rospy.init_node('Walker', anonymous=True)
	#Subscribe in ROS to allow for receiving messages on given topic
	#Invoke call to ROS master node, for registry of publishing and subcribing nodes
	#Messages passed to bumperEventCallback function
	##rostopic echo /mobile_base/events/bumper checks the sensor
	rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, bumperEventCallback)
	rate = rospy.Rate(10) #10hz
	#A twist message for moving the robot is created
	desired_velocity_move = Twist()
	#A twist message for turning the robot is created
	desired_velocity_turn = Twist()
	#Loop until rospy is shutdown when crtl-c
	while not rospy.is_shutdown():
		#If bump_state is true, stop robot
		if bump_state != 0:
			break
		for i in range(0,40):
			#If bump_state is true, stop robot
			if bump_state != 0:
				break
			#Drive robot straight (linear)
			desired_velocity_move.linear.x = 0.2 #Linear velocity with 0.2 m/sec.
				#Publish velocity move
	    		pub.publish(desired_velocity_move)
			rate.sleep()
		for i in range(0,10):
			#If bump_state is true, stop robot
			if bump_state != 0:
				break
			#Math.radians converts degree value 90 into radians
			#Turn robot in order to trace a square
			desired_velocity_turn.angular.z = math.radians(90)
				#Publish velocity turn
	    		pub.publish(desired_velocity_turn)
			rate.sleep()
	#Sleep until shutdown flags true
	#rospy.spin()
	#Signal shutdown with reason
	rospy.signal_shutdown("Exercise finished")

if __name__ == "__main__":
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
