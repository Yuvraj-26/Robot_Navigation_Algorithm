#!/usr/bin/env python
# This final piece of skeleton code will be centred around
# to follow a colour and stop upon sight of another one.

from __future__ import division
import cv2
import numpy as np
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from actionlib_msgs.msg import *

class characterIdentification():
    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('image_topic', Image, self.imageCallback)
        # sub_rect = rospy.Subscriber('rectangle_topic', Bool, self.callbackRectangle)
        self.character_printed = False

        # Initialise sensitivity variable for colour detection
        self.Rsensitivity = 5
        self.Ysensitivity = 10
        self.Psensitivity = 10
        self.Bsensitivity = 10
        self.pub_mustard = rospy.Publisher('mustard_topic', Bool, queue_size=10)
        self.pub_peacock = rospy.Publisher('peacock_topic', Bool, queue_size=10)
        self.pub_plum = rospy.Publisher('plum_topic', Bool, queue_size=10)
        self.pub_scarlet = rospy.Publisher('scarlet_topic', Bool, queue_size=10)
        self.pub_circle_based_velocity = rospy.Publisher('mobile_base/commands/velocity', Twist)

        self.desired_velocity = Twist()
        self.forward = 0.2
        self.backwards = -0.2
        self.left = -0.2
        self.right = 0.2
        self.stop = 0

        self.character_x = 0
        self.character_y = 0
        self.character_w = 0
        self.character_h = 0

        self.image_x = 0
        self.image_y = 0

    def imageCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print("Image conversion failed")
            print(e)
            pass

        self.image_x = cv_image.shape[1]
        self.image_y = cv_image.shape[0]

        hsv_yellow_lower = np.array([60.0/2 - self.Ysensitivity, 38.0/100*255, 50.0/100*255])
        hsv_yellow_upper = np.array([70.0/2 + self.Ysensitivity, 70.0/100*255, 90.0/100*255])

        hsv_blue_lower = np.array([210.0/2 - self.Bsensitivity, 30.0/100*255, 30.0/100*255])
        hsv_blue_upper = np.array([250.0/2 + self.Bsensitivity, 65.0/100*255, 80.0/100*255])

        hsv_purple_lower = np.array([280.0/2 - self.Psensitivity, 15.0/100*255, 15.0/100*255])
        hsv_purple_upper = np.array([320.0/2 + self.Psensitivity, 50.0/100*255, 35.0/100*255])

        hsv_red_lower = np.array([0.0 / 2 - self.Rsensitivity, 50, 50])
        hsv_red_upper = np.array([0.0 / 2 + self.Rsensitivity, 255, 255])


        # Convert the rgb image into a hsv image
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Filter out everything but predefined colours
        mask_yellow = cv2.inRange(hsv_image, hsv_yellow_lower, hsv_yellow_upper)
        mask_blue = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)
        mask_purple = cv2.inRange(hsv_image, hsv_purple_lower, hsv_purple_upper)
        mask_red = cv2.inRange(hsv_image, hsv_red_lower, hsv_red_upper)

        # Combine masks
        # mask_yb = cv2.bitwise_or(mask_yellow, mask_blue)
        # mask_ybp = cv2.bitwise_or(mask_yb, mask_purple)
        # mask_ybpr = cv2.bitwise_or(mask_ybp, mask_red)

        # Apply the mask to the original image
        mask_image_y = cv2.bitwise_and(cv_image, cv_image, mask=mask_yellow)
        mask_image_b = cv2.bitwise_and(cv_image, cv_image, mask=mask_blue)
        mask_image_p = cv2.bitwise_and(cv_image, cv_image, mask=mask_purple)
        mask_image_r = cv2.bitwise_and(cv_image, cv_image, mask=mask_red)

        self.findMustard(self, mask_image_y)
        self.findPeacock(self, mask_image_b)
        self.findPlum(self, mask_image_p)
        self.findScarlet(self, mask_image_r)

    def findMustard(self, cI, cv_image):
        output = cv_image.copy()
        one_D_cv_image = cv_image.reshape((-1))
        mCounter = cv2.countNonZero(one_D_cv_image) / 3

        m_flag = False
        if mCounter >= 300:
            m_flag = True
            # if not self.character_printed:
            #     self.character_printed = True
            #     file = open("cluedo_character.txt", "w")
            #     file.write("Colonel Mustard")
            #     file.close()

        self.pub_mustard.publish(m_flag)
        # Debugging
        # print(mCounter)
        # cv2.imshow("output yellow", np.hstack([cv_image, output]))
        # cv2.waitKey(3)

    def findPeacock(self, cI, cv_image):
        output = cv_image.copy()
        one_D_cv_image = cv_image.reshape((-1))
        pCounter = cv2.countNonZero(one_D_cv_image) / 3

        p_flag = False
        if pCounter >= 300:
            p_flag = True
            # if not self.character_printed:
            #     self.character_printed = True
            #     file = open("cluedo_character.txt", "w")
            #     file.write("Mrs Peacock")
            #     file.close()

        self.pub_peacock.publish(p_flag)
        # Debugging
        # print(pCounter)
        # cv2.imshow("output blue", np.hstack([cv_image, output]))
        # cv2.waitKey(3)

    def findPlum(self, cI, cv_image):
        output = cv_image.copy()
        one_D_cv_image = cv_image.reshape((-1))
        plCounter = cv2.countNonZero(one_D_cv_image) / 3

        pl_flag = False
        if plCounter >= 300:
            pl_flag = True
            # if not self.character_printed:
            #     self.character_printed = True
            #     file = open("cluedo_character.txt", "w")
            #     file.write("Professor Plum")
            #     file.close()

        self.pub_plum.publish(pl_flag)
        # Debugging
        # print(plCounter)
        # cv2.imshow("output purple", np.hstack([cv_image, output]))
        # cv2.waitKey(3)

    def findScarlet(self, cI, cv_image):
        output = cv_image.copy()
        one_D_cv_image = cv_image.reshape((-1))
        sCounter = cv2.countNonZero(one_D_cv_image) / 3

        s_flag = False
        if sCounter >= 300:
            s_flag = True
            # if not self.character_printed:
            #     self.character_printed = True
            #     file = open("cluedo_character.txt", "w")
            #     file.write("Miss Scarlett")
            #     file.close()

        self.pub_scarlet.publish(s_flag)

        # Debugging
        # print(sCounter)
        # cv2.imshow("output red s ", np.hstack([cv_image, output]))
        # cv2.waitKey(3)

    # def callbackRectangle(self, data):
        # print("callbackRectangle " + str(data.data))


def main(args):
    rospy.init_node('character_identification', anonymous=True)
    cI = characterIdentification()
    print("Initializing character finder")
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

# Check if the node is executing in the main path
if __name__ == '__main__':
    main(sys.argv)
