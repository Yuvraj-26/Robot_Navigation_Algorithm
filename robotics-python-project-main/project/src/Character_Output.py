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
from std_msgs.msg import Bool, Int32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from actionlib_msgs.msg import *


class CharacterOutput():
    def __init__(self):
        self.bridge = CvBridge()
        # Subscribers
        self.sub_mustard_topic = rospy.Subscriber('mustard_topic', Bool, self.callbackMustard)
        self.sub_peacock_topic = rospy.Subscriber('peacock_topic', Bool, self.callbackPeacock)
        self.sub_plum_topic = rospy.Subscriber('plum_topic', Bool, self.callbackPlum)
        self.sub_scarlet_topic = rospy.Subscriber('scarlet_topic', Bool, self.callbackScarlet)
        self.sub_rectangle_topic = rospy.Subscriber('rectangle_topic', Bool, self.callbackRectangle)
        self.sub_rectangle_in_bounds_topic = rospy.Subscriber('rectangle_in_bounds_topic', Bool,
                                                              self.callbackRectangleInBounds)
        self.sub_rectangle_ints_topic = rospy.Subscriber('rectangle_ints_topic', Int32MultiArray,
                                                         self.callbackRectangleInts)
        self.sub = rospy.Subscriber('image_topic', Image, self.imageCallback)

        self.pub_shutdown_all_nodes = rospy.Publisher('shutdown_topic', Bool, queue_size=10)

        self.character_printed = False
        self.character_captured = False
        self.shutdown_all_nodes = False

        # Initialise sensitivity variable for colour detection
        self.r_sensitivity = 5
        self.Ysensitivity = 10
        self.Psensitivity = 10
        self.Bsensitivity = 10
        self.pub_mustard = rospy.Publisher('mustard_topic', Bool, queue_size=10)
        self.pub_peacock = rospy.Publisher('peacock_topic', Bool, queue_size=10)
        self.pub_plum = rospy.Publisher('plum_topic', Bool, queue_size=10)
        self.pub_scarlet = rospy.Publisher('scarlet_topic', Bool, queue_size=10)
        self.pub_circle_based_velocity = rospy.Publisher('mobile_base/commands/velocity', Twist)

        self.mustard = False
        self.peacock = False
        self.plum = False
        self.scarlet = False
        self.rectangle = False
        self.rectangle_in_bounds = False
        self.rectangle_ints = False

        self.character_x = 0
        self.character_y = 0
        self.character_w = 0
        self.character_h = 0
        self.character_center = 0

        self.image_x = 0
        self.image_y = 0

        self.cv_image = np.zeros((480, 640, 3), np.uint8)

    def imageCallback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print("Image conversion failed")
            print(e)
            pass

        self.image_x = self.cv_image.shape[1]
        self.image_y = self.cv_image.shape[0]

        self.captureImage()
        self.writeCharacter()
        self.shutdownAllNodes()

    def callbackMustard(self, data):
        self.mustard = data.data

    def callbackPeacock(self, data):
        self.peacock = data.data

    def callbackPlum(self, data):
        self.plum = data.data

    def callbackScarlet(self, data):
        self.scarlet = data.data

    def callbackRectangle(self, data):
        self.rectangle = data.data

    def callbackRectangleInBounds(self, data):
        self.rectangle_in_bounds = data.data

    def callbackRectangleInts(self, data):
        self.rectangle_ints = data.data

    def captureImage(self):
        if self.rectangle and self.rectangle_in_bounds and (
                self.mustard or self.peacock or self.plum or self.scarlet) and not self.character_captured:
            print("capturing")
            cv2.imwrite('cluedo_character.png', self.cv_image)
            self.character_captured = True

    def writeCharacter(self):
        if not self.character_printed:
            if self.mustard:
                file = open("cluedo_character.txt", "w")
                file.write("Colonel Mustard")
                file.close()
                self.character_printed = True

            if self.peacock:
                file = open("cluedo_character.txt", "w")
                file.write("Mrs Peacock")
                file.close()
                self.character_printed = True

            if self.plum:
                file = open("cluedo_character.txt", "w")
                file.write("Professor Plum")
                file.close()
                self.character_printed = True

            if self.scarlet:
                file = open("cluedo_character.txt", "w")
                file.write("Miss Scarlett")
                file.close()
                self.character_printed = True

    def shutdownAllNodes(self):
        if self.character_printed and self.character_captured:
            self.pub_shutdown_all_nodes.publish(True)
        else:
            self.pub_shutdown_all_nodes.publish(False)


def main(args):
    rospy.init_node('character_output', anonymous=True)
    c_o = CharacterOutput()
    print("Initializing character finder")
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


# Check if the node is executing in the main path
if __name__ == '__main__':
    main(sys.argv)
