#!/usr/bin/env python
# This first piece of skeleton code will be centred around
# extracting a specific colour and displaying the output

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
        # Initialise the value for self.sensitivity in the colour detection (10 should be enough)
        self.sensitivity = 10
        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        self.initial_bridge = CvBridge()
        # We covered which topic to subscribe to should you wish to receive image data
        self.image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.callback)

    def callback(self, data):
        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
        try:
            cv_image = self.initial_bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)
        # Set the upper and lower bounds for the green colour (HSV) here
        lower_bound_green = np.array([60 - self.sensitivity, 100, 100])
        upper_bound_green = np.array([60 + self.sensitivity, 255, 255])
        # Convert the rgb image into a hsv image
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # Filter out everything but the green colour using the cv2.inRange() method here. Use the green upper and lower for the range.
        ##mask_name = cv2.inRange(converted_image, lower, upper)
        mask_first_step = cv2.inRange(hsv, lower_bound_green, upper_bound_green)
        # Apply the mask to the original image using the cv2.bitwise_and() method
        # As mentioned on the worksheet the best way to do this is to bitwise_and an image with itself and pass the mask to the mask parameter
        # As opposed to performing a bitwise_and on the mask and the image.
        #mask_image=cv2.bitwise_and(image_name,image_name, mask=mask_name)
        mask_image = cv2.bitwise_and(cv_image, cv_image, mask = mask_first_step)
        # Show the resultant images you have created. You can show all of them or just the end result if you wish to.
        cv2.imshow("mask_first_step result", mask_first_step)
        cv2.imshow("mask_image result", mask_image)
        cv2.imshow("raw feed", cv_image)
        cv2.waitKey(3)

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
    # Instantiate your class
    # And rospy.init the entire node
    colourIdentify = colourIdentifier()
    # Initialize the node and rospy.init node
    rospy.init_node('first_step', anonymous=True)
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
