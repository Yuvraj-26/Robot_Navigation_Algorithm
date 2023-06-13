#!/usr/bin/env python
from __future__ import division
import cv2
import numpy as np
import rospy
import sys

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from actionlib_msgs.msg import *


class rectangleIdentification():
    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('image_topic', Image, self.imageCallback)

        self.pub_rectangle = rospy.Publisher('rectangle_topic', Bool, queue_size=10)
        self.pub_rectangle_in_bounds = rospy.Publisher('rectangle_in_bounds_topic', Bool, queue_size=10)
        # Publishes location rectangle corners in form r1x, r1y, r2x, r2y. Publishes -1,-1,-1,-1 if no rectangle is identified
        self.pub_rectangle_ints = rospy.Publisher('rectangle_ints_topic', Int32MultiArray, queue_size=10)

        # Initialise sensitivity variable for colour detection
        self.hue_sensitivity = 30
        self.sat_sensitivity = 10
        self.val_sensitivity = 10
        # Black colour value of outer rectangle for Cludeo characters
        self.hue_lower = 220.0 / 2
        self.hue_upper = 360.0 / 2
        self.saturation_lower = 0.0 / 100 * 255
        self.saturation_upper = 15.0 / 100 * 255
        self.value_lower = 0.0 / 100 * 255
        self.value_upper = 10.0 / 100 * 255

        self.image_x = 0
        self.image_y = 0

    def imageCallback(self, data):
        # Try to convert from image message to cv2 Image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print("Image conversion failed")
            print(e)
            pass

        self.image_x = cv_image.shape[1]
        self.image_y = cv_image.shape[0]

        hsv_black_lower = np.array([self.hue_lower - self.hue_sensitivity, self.saturation_lower - self.sat_sensitivity,
                                    self.value_lower - self.val_sensitivity])
        hsv_black_upper = np.array([self.hue_upper + self.hue_sensitivity, self.saturation_upper + self.sat_sensitivity,
                                    self.value_upper + self.val_sensitivity])

        # Convert the rgb image into a hsv image
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Filter out everything but predefined colours
        mask_black = cv2.inRange(hsv_image, hsv_black_lower, hsv_black_upper)

        # Apply the mask to the original image
        mask_image_black = cv2.bitwise_and(cv_image, cv_image, mask=mask_black)

        self.findRectangle(self, mask_image_black)

    def findRectangle(self, rI, cv_image):
        # Copy input image
        output = cv_image.copy()

        # Find all fully black pixels
        black_pixels = np.where(
            (cv_image[:, :, 0] == 0) &
            (cv_image[:, :, 1] == 0) &
            (cv_image[:, :, 2] == 0)
        )
        # Change black pixels to white
        cv_image[black_pixels] = [255, 255, 255]

        # Find all none white pixels
        other_pixels = np.where(
            (cv_image[:, :, 0] != 255) &
            (cv_image[:, :, 1] != 255) &
            (cv_image[:, :, 2] != 255)
        )

        rect_flag = False
        rectangle_in_bounds_flag = False
        min_in_x = 0
        min_in_y = 0
        max_in_x = 0
        max_in_y = 0
        # If there exist more than 50 non white pixels draw a rectangle from first to last pixels and then set rectangle flag to true
        if len(other_pixels[0]) >= 50:
            rect_flag = True

            # Find first and last none white pixels
            min_in_x = min(other_pixels[0])
            min_in_y = min(other_pixels[1])
            max_in_x = max(other_pixels[0])
            max_in_y = max(other_pixels[1])

            # If first and last aren't to close to the edge of the edge publish rectangle is in bounds
            if min_in_x > 50 and max_in_x < self.image_x - 50 and min_in_y > 20 and max_in_x < self.image_y - 20:
                rectangle_in_bounds_flag = True

            # Draw rectangle on image
            cv2.rectangle(output, (min_in_y, min_in_x), (max_in_y, max_in_x), (255, 255, 255), 3)

        # Publish rectangle information
        self.pub_rectangle.publish(rect_flag)
        self.pub_rectangle_in_bounds.publish(rectangle_in_bounds_flag)
        if rect_flag:
            self.pub_rectangle_ints.publish(data=[min_in_x, min_in_y, max_in_x, max_in_y])
        else:
            self.pub_rectangle_ints.publish(data=[-1, -1, -1, -1])

        # Debugging
        cv2.imshow("output rectangle", np.hstack([cv_image, output]))
        cv2.waitKey(3)


def main(args):
    rospy.init_node('rectangle_identification', anonymous=True)
    rI = rectangleIdentification()
    print("Initializing rectangle finder")
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


# Check if the node is executing in the main path
if __name__ == '__main__':
    main(sys.argv)
