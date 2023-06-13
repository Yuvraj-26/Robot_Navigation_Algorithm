#!/usr/bin/env python
#Exercise 1: Continuously drive the robot in a circle
import rospy
from geometry_msgs.msg import Twist

def publisher():
	pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
	rospy.init_node('Walker', anonymous=True)
	rate = rospy.Rate(10) #10hz
	#A twist message created
	desired_velocity = Twist()
	#Desried velociy to drive robot in a continuous circle - delivering velocities at same time
	desired_velocity.linear.x = 0.2 #Linear velocity with 0.2 m/sec.
	desired_velocity.angular.z = 0.2 #Angular velocity with 0.2 m/sec.
	#Loop until rospy is shutdown when crtl-c
	while not rospy.is_shutdown():
		pub.publish(desired_velocity)
		rate.sleep()

if __name__ == "__main__":
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass
