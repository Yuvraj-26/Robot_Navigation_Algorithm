#!/usr/bin/env python

from __future__ import division
import cv2
import rospy
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from actionlib_msgs.msg import *


class imageConverter():
    def __init__(self):
        # Initialise CvBridge
        self.bridge = CvBridge()

        self.sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.imageCallback)
        self.pub = rospy.Publisher('image_topic', Image, queue_size=10)

    def imageCallback(self, data):
        # Convert the received image into a opencv image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print("Image conversion failed")
            print(e)
            pass
        # Show the camera feed in a window
        cv2.namedWindow('Camera_Feed')
        cv2.imshow('Camera_Feed', cv_image)
        cv2.waitKey(3)
        self.pub.publish(self.bridge.cv2_to_imgmsg(cv_image))
        self.pub.publish(data)


def main(args):
    rospy.init_node('image_converter', anonymous=True)
    iC = imageConverter()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting Down image Converter")
        pass
    cv2.destroyAllWindows()


# Check if the node is executing in the main path
if __name__ == '__main__':
    main(sys.argv)
