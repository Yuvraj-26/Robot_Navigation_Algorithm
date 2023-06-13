#!/usr/bin/env python
# This third piece of skeleton code will be centred around the properties of the object, and determining
# when a colour object is detected, to set a flag.

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
        # Initialise a publisher to publish messages or print the message when the colour is detected
        # We covered which topic receives messages in the 1st Lab Session
		self.publisher_move_robot = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
        # Initialise any flags that signal a colour has been detected in view (default to false)
		flag_r = False
		flag_b = False
        flag.g = False
        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
		self.sensitivity = 10
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
        # Set the upper and lower bounds for two colours you wish to identify
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
		mask1 = cv2.inRange(HSV_image_conv, lower_bound_blue, upper_bound_blue)
		mask2 = cv2.inRange(HSV_image_conv, lower_bound_red, upper_bound_red)
        # To combine the masks you should use the cv2.bitwise_or() method
        # You can only bitwise_or two image at once, so multiple calls are necessary for more than two colours
		mask_combined = cv2.bitwise_or(mask2,mask1)
        # Apply the mask to the original image using the cv2.bitwise_and() method
        # As mentioned on the worksheet the best way to do this is to bitwise and an image with itself and pass the mask to the mask parameter
        # As opposed to performing a bitwise_and on the mask and the image.
		bluecontours, _ = cv2.findContours(mask1, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		redcontours, _ = cv2.findContours(mask2, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # Find the contours that appear within the certain colour mask using the cv2.findContours() method
        # For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE

        # Loop over the contours
        #if len(greencontours)>0:

            # There are a few different methods for identifying which contour is the biggest:
            # Loop through the list and keep track of which contour is biggest
            #OR
            # Use the max() method to find the largest contour
            #c = max(<contours>, key=cv2.contourArea)

            #Moments can calculate the center of the contour
            # M = cv2.moments(c)
            # cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

            #Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
            if cv2.contourArea(c) > x #<What do you think is a suitable area?>:
                # draw a circle on the contour you're identifying as a green object as well
                #minEnclosingCircle can find the centre and radius of the largest contour(result from max())
                #(x, y), radius = cv2.minEnclosingCircle(c)

                # cv2.circle(<image>,(<center x>,<center y>),<radius>,<colour (rgb tuple)>,<thickness (defaults to 1)>)

                # Then alter the values of any flags

        #if the flag is true (colour detected)
            #print the flag to check it works
            #alternatively publish to the lab1 talker/listener

        #Show the resultant images you have created. You can show all of them or just the end result if you wish to.

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
    # Instantiate your class
    # And rospy.init the entire node
    colourIdentify = colourIdentifier()
    # Initialize the node and rospy.init node
    rospy.init_node('third_step', anonymous=True)
    # Ensure that the node continues running with rospy.spin()
    # You may need to wrap rospy.spin() in an exception handler in case of KeyboardInterrupts
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Interrupted")
    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()

# Check if the node is executing in the main path
if __name__ == '__main__':
    main(sys.argv)
