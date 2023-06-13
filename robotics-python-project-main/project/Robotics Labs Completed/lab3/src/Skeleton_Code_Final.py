#!/usr/bin/env python
# This final piece of skeleton code will be centred around
# to follow a colour and stop upon sight of another one.
# This piece of skeleton code works by the robot following the blue object
# in rotating movement directly to the object. When the red object is visible
# by the robot, the robot stops. Additionally, the robot will not collide with
# another object

from __future__ import division
import cv2
import numpy as np
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class colourIdentifier():
	def __init__(self):
        # Initialise a publisher to publish messages to the robot base
        # We covered which topic receives messages that move the robot in the 2nd Lab Session
		self.publisher_move_robot = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
		rospy.init_node('image_final', anonymous=True)
        # Initialise any flags that signal a colour has been detected in view (default to false)
		self.flag_r = False
		self.flag_b = False
        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
		self.sensitivity = 10
        # Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)
		# A twist message for moving the robot is created
		des_velocity = Twist()
		# linear velocity with 0.2 m/sec
		des_velocity.linear.x = 0.2
		# angular velocity with 0 m/sec
		des_velocity.angular.z = 0
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
		self.initial_bridge = CvBridge()
        # We covered which topic to subscribe to should you wish to receive image data
		self.subscriber_receive_image_topic = rospy.Subscriber("camera/rgb/image_raw", Image, self.callback)

	def callback(self, data):
        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
		try:
			cv_image = self.initial_bridge.imgmsg_to_cv2(data, "passthrough")
		except CvBridgeError as e:
			print(e)
        # Set the upper and lower bounds for the two colours you wish to identify
        # hue value = 0, 60 or 120 (red, green, or blue)
        # Blue bounds
		lower_bound_blue = np.array([120 - self.sensitivity, 100, 100])
		upper_bound_blue = np.array([120 + self.sensitivity, 255, 255])
        # Red bounds
		lower_bound_red = np.array([0 - self.sensitivity, 100, 100])
		upper_bound_red = np.array([0 + self.sensitivity, 255, 255])
        # Convert the rgb image into a hsv image
		HSV_image_conv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # Filter out everything but particular colours using the cv2.inRange() method
        # Be sure to do this for the second or third colour as well
		mask1 = cv2.inRange(HSV_image_conv, lower_bound_blue, upper_bound_blue)
		mask2 = cv2.inRange(HSV_image_conv, lower_bound_red, upper_bound_red)
        # To combine the masks you should use the cv2.bitwise_or() method
        # You can only bitwise_or two image at once, so multiple calls are necessary for more than two colours
        # Only two colours are red and blue used so one call to combine mask required
		mask_combined = cv2.bitwise_or(mask2,mask1)
        # Apply the mask to the original image using the cv2.bitwise_and() method
        # As mentioned on the worksheet the best way to do this is to bitwise and an image with itself and pass the mask to the mask parameter
        # As opposed to performing a bitwise_and on the mask and the image.
		mask_applied = cv2.bitwise_and(cv_image,cv_image, mask = mask_combined)
        # Find the contours that appear within the certain colour mask using the cv2.findContours() method
        # For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE
		bluecontours, _ = cv2.findContours(mask1, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		redcontours, _ = cv2.findContours(mask2, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # Loop over the contours
        # Array of cs contours
		cs = []
		find_biggest_c = 0
		# initialise check
		check_b = 0
		check_r = 0
        #if len(greencontours)>0:
        #if len(colour1contours)>0:
		if len(bluecontours) > 0:
            # There are a few different methods for identifying which contour is the biggest
            # Loop throguht the list and keep track of which contour is biggest or
            # Use the max() method to find the y contour
            #c = max(<contours>, key=cv2.contourArea)
            #colour 1 is blue
			c1 = max(bluecontours, key=cv2.contourArea)
			cs.append(c1)
            # M = cv2.moments(c)
			M1 = cv2.moments(c1)
            # cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
			c_x_b, c_y_b = int(M1['m10']/M1['m00']), int(M1['m01']/M1['m00'])
            # circle completely covers the object shape with minimal area
			(_, r) = cv2.minEnclosingCircle(c1)
            # check for colour 1
			check_b = cv2.contourArea(c1)
		if len(redcontours) > 0:
            # There are a few different methods for identifying which contour is the biggest
            # Loop throguht the list and keep track of which contour is biggest or
            # Use the max() method to find the y contour
            #c = max(<contours>, key=cv2.contourArea)
			c2 = max(redcontours, key=cv2.contourArea)
			cs.append(c2)
            # M = cv2.moments(c)
			M2 = cv2.moments(c2)
            # cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
			c_x_r, c_y_r = int(M2['m10']/M2['m00']), int(M2['m01']/M2['m00'])
            # circle completely covers the object shape with minimal area
			(_, r) = cv2.minEnclosingCircle(c2)
            # check for colour 2
			check_r = cv2.contourArea(c2)
		if len(cs) > 0:
			find_biggest_c = max(cs, key=cv2.contourArea)
			y = cv2.contourArea(find_biggest_c)
        #Check if the area of the shape you want is big enough to be considered
        # If it is then change the flag for that colour to be True(1)
        #if cv2.contourArea(c) > x: #<What do you think is a suitable area?>:
		if check_b > 425:
            # draw a circle on the contour you're identifying
            # cv2.circle(<image>, (<center x>,<center y>), <r>, <colour (rgb tuple)>, <thickness (defaults to 1)>)
			cv2.circle(mask1,(c_x_b,c_y_b),int(r),(0,0,255),5)
            # Then alter the values of any flags
			self.flag_b = True
        #Check if a flag has been set to true
        #if self.colour1_flag == 1:
		if self.flag_b == 1:
        #if cv2.contourArea(c) > ****:
			if (check_b) > 125252:
                # Too close to object, need to move backwards
                # linear = negative
                # Using Twist(), if robot gets too close too object,
                # move backwards some distance until following can resume
				des_velocity = Twist()
				des_velocity.linear.x = -0.2
            #elif cv2.contourArea(c) < ****:
			if (check_b) < 111111:
                # Too far away from object, need to move forwards
                # linear = positive
                # Using Twist(), if robot gets too far away from obkect,
                # move forwards some distance until too close or stopping occurs
				des_velocity = Twist()
				des_velocity.linear.x = 0.2
				_, final_step_image_shape, _ = cv_image.shape
                #Alternatively cx,cy can be used to position and rotate towards the object
				if (c_x_b-(final_step_image_shape/2))/36 > 0:
					des_velocity.angular.z = -0.2
				elif (c_x_b-(final_step_image_shape/2))/36 < 0:
					des_velocity.angular.z = 0.2
                #else:
                    #linear = 0
				else:
					des_velocity.angular.x = 0
        # Be sure to do this for the other colour as well
        #Again... for next colour
		if check_r > 425:
			cv2.circle(mask2,(c_x_r,c_y_r),int(r),(0,0,255),5)
			self.flag_r = True
			des_velocity = Twist()
			des_velocity.linear.x = 0
			des_velocity.angular.z = 0
        # self.<publisher_name>.publish(<Move>)
		self.publisher_move_robot.publish(des_velocity)
        #Show the resultant images you have created. You can show all of them or just the end result if you wish to.
		cv2.imshow("raw feed", cv_image)
		cv2.imshow("final_image_final_step with mask_applied", mask_applied)
		cv2.waitKey(3)

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
    # Instantiate your class
    # And rospy.init the entire node
	#rospy.init_node('final_step', anonymous=True)
	colourIdentify = colourIdentifier()
    # Ensure that the node continues running with rospy.spin()
    # You may need to wrap it in an exception handler in case of KeyboardInterrupts
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Interrupted")

#Remember to destroy all image windows before closing node
cv2.destroyAllWindows()

# Check if the node is executing in the main path
if __name__ == '__main__':
	main(sys.argv)
