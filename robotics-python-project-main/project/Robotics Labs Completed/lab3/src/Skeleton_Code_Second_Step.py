#!/usr/bin/env python
# This second piece of skeleton code will be centred around
# combining colours and creating a mask

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
        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)

        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use

        # We covered which topic to subscribe to should you wish to receive image data

    def callback(self, data):
        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler

        # Set the upper and lower bounds for the green colour (HSV) here
        # Set the upper and lower bounds for red and blue here, similar to before
        ##Note green = 60, blue = 120, red = 0 for HSV in OpenCV

        # Convert the rgb image into a hsv image

        # Filter out everything but particular colours using the cv2.inRange() method
        #Filter each colour separately

        # To combine the three colour masks above you should use the cv2.bitwise_or() method
        # You can only bitwise_or TWO images at once, so multiple calls are necessary for more than two colours

        # Apply the mask to the original image using the cv2.bitwise_and() method
        # As mentioned on the worksheet the best way to do this is to bitwise_and an image with itself and pass the mask to the mask parameter
        # As opposed to performing a bitwise_and on the mask and the image.

        #Show the resultant images you have created. You can show all of them or just the end result if you wish to.

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
    # Instantiate your class
    # And rospy.init the entire node
    cI = colourIdentifier()
    # Ensure that the node continues running with rospy.spin()
    # You may need to wrap rospy.spin() in an exception handler in case of KeyboardInterrupts

    # Remember to destroy all image windows before closing node

# Check if the node is executing in the main path
if __name__ == '__main__':
    main(sys.argv)
