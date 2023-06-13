#!/usr/bin/env python
# This final piece of skeleton code will be centred around
# to follow a colour and stop upon sight of another one.

from __future__ import division
import cv2
import cv
import numpy as np
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from actionlib_msgs.msg import *

class circleFinder():
    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('image_topic', Image, self.imageCallback)
        self.green_circle_flag = False
        self.red_circle_flag = False

        self.green_circle_flag_global = False
        self.red_circle_flag_global = False

        # Initialise sensitivity variable for colour detection
        self.Rsensitivity = 1
        self.Gsensitivity = 20
        self.pub_red_circle = rospy.Publisher('red_circle_topic', Bool, queue_size=10)
        self.pub_green_circle = rospy.Publisher('green_circle_topic', Bool, queue_size=10)
        self.pub_circle_based_velocity = rospy.Publisher('mobile_base/commands/velocity', Twist)
        self.pub_main_room_moving_sub = rospy.Subscriber('turtle_bot_main_room_moving_topic', Bool, self.externalMovementCallback )

        self.desired_velocity = Twist()
        self.forward = 0.2
        self.backwards = -0.2
        self.left = -0.2
        self.right = 0.2
        self.stop = 0

        self.circle_x = 0
        self.circle_y = 0
        self.circle_r = 0

        self.image_x = 0
        self.image_y = 0

        self.moving = False

    def externalMovementCallback(self, data):
        self.moving = data.data

    def imageCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print("Image conversion failed")
            print(e)
            pass

        self.image_x = cv_image.shape[1]
        self.image_y = cv_image.shape[0]

        # Set the upper and lower bounds for red and green circles
        hsv_red_lower = np.array([0 - self.Rsensitivity, 100, 100])
        hsv_red_upper = np.array([0 + self.Rsensitivity, 255, 255])
        hsv_green_lower = np.array([60 - self.Gsensitivity, 50, 50])
        hsv_green_upper = np.array([60 + self.Gsensitivity, 255, 255])

        # Convert the rgb image into a hsv image
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Filter out everything but predefined colours
        mask_red = cv2.inRange(hsv_image, hsv_red_lower, hsv_red_upper)
        mask_green = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)

        # Combine masks
        mask_rg = cv2.bitwise_or(mask_red, mask_green)

        # Apply the mask to the original image
        mask_image_rg = cv2.bitwise_and(cv_image, cv_image, mask=mask_rg)
        mask_image_r = cv2.bitwise_and(cv_image, cv_image, mask=mask_red)
        mask_image_g = cv2.bitwise_and(cv_image, cv_image, mask=mask_green)

        self.findGreenCircle(self, mask_image_g)
        self.findRedCircle(self, mask_image_r)

    def findGreenCircle(self, cF, cv_image):
        output = cv_image.copy()
        grey_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(grey_image, 100, 200)
        blur = cv2.GaussianBlur(edges, (5, 5), 0)

        # circles = cv2.HoughCircles(blur, cv.CV_HOUGH_GRADIENT, 1.5, 1000, 0, 500)
        circles = cv2.HoughCircles(blur, cv.CV_HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=10, maxRadius=0)

        # For debugging show green greyscale, green edges, blurred edges
        # cv2.imshow("outputgrey green", grey_image)
        # cv2.imshow("output edge green", edges)
        # cv2.imshow("output blur green", blur)
        self.desired_velocity.linear.x = 0
        self.desired_velocity.angular.z = 0

        self.green_circle_flag = False
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                # Get x, y and r of circle
                x = i[0]
                y = i[1]
                r = i[2]

                # If circle is within bounds return green flag as true or readjust
                if x > self.image_x / 2 - self.image_x / 20 and x < self.image_x / 2 + self.image_x / 20:
                    if y > 0 + r + self.image_y/10 and y < self.image_y - r - self.image_y/10:
                        self.green_circle_flag = True
                        self.green_circle_flag_global = True
                    else:
                        if not self.moving:
                            if not y > 0 + r + self.image_y / 10:
                                self.desired_velocity.linear.x = self.forward
                            elif not y < self.image_y - r - self.image_y/10:
                                self.desired_velocity.linear.x = self.backwards
                else:
                    if not self.moving:
                        if not x > self.image_x / 2 - self.image_x / 20:
                            self.desired_velocity.angular.z = self.right
                        elif not  x < self.image_x / 2 + self.image_x / 20:
                            self.desired_velocity.angular.z = self.left

                cv2.circle(output, (i[0], i[1]), i[2], (0, 255, 0), 2)
        # If no circle is found we are facing the wrong direction and should spin (behaviour dependant on other nodes and has been disabled for the time being)
        # else:
        #     if self.red_circle_flag_global == False and self.green_circle_flag_global == False:
        #         self.desired_velocity.angular.z = self.left

        self.pub_green_circle.publish(self.green_circle_flag)
        if not self.moving:
            self.pub_circle_based_velocity.publish(self.desired_velocity)

        # Debugging show green circle and input image
        # cv2.imshow("output green", np.hstack([cv_image, output]))
        # cv2.waitKey(3)

    def findRedCircle(self, cF, cv_image):
        output = cv_image.copy()
        grey_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(grey_image, 100, 200)
        blur = cv2.GaussianBlur(edges, (5, 5), 0)

        circles = cv2.HoughCircles(blur, cv.CV_HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=10, maxRadius=0)

        # For debugging show red greyscale, red edges, blurred edges
        # cv2.imshow("outputgrey  red", grey_image)
        # cv2.imshow("output edge red", edges)
        # cv2.imshow("output blur red", blur)

        self.red_circle_flag = False
        if circles is not None:
            circles = np.uint16(np.around(circles))
            self.red_circle_flag = True
            self.red_circle_flag_global = True
            for i in circles[0, :]:
                cv2.circle(output, (i[0], i[1]), i[2], (0, 0, 255), 2)

        self.pub_red_circle.publish(self.red_circle_flag)

        # Debugging show red circle and input image
        # cv2.imshow("output red", np.hstack([cv_image, output]))
        # cv2.waitKey(3)

def main(args):
    rospy.init_node('circle_finder', anonymous=True)
    cF = circleFinder()
    print("Initializing circle finder")
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

# Check if the node is executing in the main path
if __name__ == '__main__':
    main(sys.argv)
