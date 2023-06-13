#!/usr/bin/env python
# This final piece of skeleton code will be centred around
# to follow a colour and stop upon sight of another one.

from __future__ import division
import cv2
import cv
import numpy as np
import rospy
import sys
import yaml
from os.path import expanduser

from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import math
from actionlib_msgs.msg import *


class colourIdentifier():
    def __init__(self):
        # Initialise publisher to send messages to the robot base
        self.goal_sent = False
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(5))

        self.pubMove = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)

        # Initialise flags
        self.mustard_flag = False
        self.peacock_flag = False
        self.plum_flag = False
        self.scarlet_flag = False
        self.redflag = False
        self.greenflag = False

        self.in_green_room = False

        # Get home directory
        home = expanduser("~")
        print(home)
        # and open input_points.yaml file
        with open(home + "/catkin_ws/src/group_project/project/example/input_points.yaml", 'r') as stream:
            points = yaml.safe_load(stream)

        # Use file data to set our variables
        self.room1_entrance_x = points['room1_entrance_xy'][0]
        self.room1_entrance_y = points['room1_entrance_xy'][1]
        self.room2_entrance_x = points['room2_entrance_xy'][0]
        self.room2_entrance_y = points['room2_entrance_xy'][1]

        self.room1_centre_x = points['room1_centre_xy'][0]
        self.room1_centre_y = points['room1_centre_xy'][1]
        self.room2_centre_x = points['room2_centre_xy'][0]
        self.room2_centre_y = points['room2_centre_xy'][1]

        self.green_circle_found = False

        # Initialise sensitivity variable for colour detection
        self.Rsensitivity = 1
        self.Gsensitivity = 20

        # Initialise some movement variables
        self.desired_velocity = Twist()
        self.forward = 0.2
        self.backwards = -0.2
        self.stop = 0
        self.rate = rospy.Rate(10)  # 10hz

        # Initialise CvBridge
        self.bridge = CvBridge()

        self.sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.imageCallback)

    def goto(self, pos, quat):
        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        # Start moving
        self.move_base.send_goal(goal)

        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60))

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            print("searching")
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

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
        return cv_image

        # TODO Orange in cones is too similar to red in sphere - change red sensitivity
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

        # Find the contours that appear within the certain mask
        redcontours = cv2.findContours(mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        greencontours = cv2.findContours(mask_green, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # print(len(greencontours[0]))

        self.redflag = False
        self.greenflag = False
        print(cv_image)
        redcircle = self.findRedCircle(self, mask_image_rg)
        greencircle = self.findGreenCircle(self, mask_image_g)

        if len(greencontours[0]) > 0:
            # Find maximum contour in green
            maxGreenC = max(greencontours[0], key=cv2.contourArea)

            M = cv2.moments(maxGreenC)
            if int(M['m00']) != 0:
                greenCx, greenCy = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])

            # Check if the area of the shape you want is big enough to be considered
            area = 50
            if cv2.contourArea(maxGreenC) > area:
                # draw a circle on the contour you're identifying
                (x, y), radius = cv2.minEnclosingCircle(maxGreenC)
                cv2.circle(cv_image, (int(x), int(y)), int(radius), (0, 255, 0), 2)

                # Set green flag to true
                self.greenflag = True

        if len(redcontours[0]) > 0:
            # Find maximum contour in red
            maxRedC = max(redcontours[0], key=cv2.contourArea)

            M = cv2.moments(maxRedC)
            if int(M['m00']) != 0:
                redCx, redCy = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])

            # Check if the area of the shape you want is big enough to be considered
            area = 50
            if cv2.contourArea(maxRedC) > area:  # <What do you think is a suitable area?>:
                # draw a circle on the contour you're identifying
                (x, y), radius = cv2.minEnclosingCircle(maxRedC)
                cv2.circle(cv_image, (int(x), int(y)), int(radius), (0, 0, 255), 2)

                # Set red flag to true
                self.redflag = True

        # Behaviour if green flag is true
        if self.greenflag == 1:
            # Center green object in vision
            if (greenCx) > 325:
                self.desired_velocity.angular.z = -0.1
            elif (greenCx) < 315:
                self.desired_velocity.angular.z = 0.1
            else:
                self.desired_velocity.angular.z = 0

            # move to towards object if too far away or backwards if too close
            if cv2.contourArea(maxGreenC) > 50000:
                # Too close to object, need to move backwards
                self.desired_velocity.linear.x = self.backwards
            elif cv2.contourArea(maxGreenC) < 45000:
                # Too far away from object, need to move forwards
                # linear = positive
                self.desired_velocity.linear.x = self.forward
            else:
                # stop once desired distance has been achieved
                self.desired_velocity.linear.x = self.stop
                self.desired_velocity.angular.z = 0

        # Behaviour if red flag is true
        if self.redflag == 1:
            # Center green object in vision
            if (redCx) > 325:
                self.desired_velocity.angular.z = -0.1
            elif (redCx) < 315:
                self.desired_velocity.angular.z = 0.1
            else:
                self.desired_velocity.angular.z = 0

            # move to towards object if too far away or backwards if too close
            if cv2.contourArea(maxRedC) > 20000:
                # Too close to object, need to move backwards
                self.desired_velocity.linear.x = self.backwards
            elif cv2.contourArea(maxRedC) < 15000:
                # Too far away from object, need to move forwards
                # linear = positive
                self.desired_velocity.linear.x = self.forward
            else:
                # stop once desired distance has been achieved
                self.desired_velocity.linear.x = self.stop
                self.desired_velocity.angular.z = 0

        # If no red or green objects are detected spin to find some objects
        if self.redflag == 0 and self.greenflag == 0:
            self.desired_velocity.linear.x = self.stop
            self.desired_velocity.angular.z = -0.2

        # Stop all movement with detection of a red object
        if self.redflag == 1:
            self.desired_velocity.linear.x = self.stop
            self.desired_velocity.angular.z = 0

        # Update movement
        # self.pubMove.publish(self.desired_velocity)

        # Show the camera feed in a window
        cv2.namedWindow('Camera_Feed')
        cv2.imshow('Camera_Feed', cv_image)
        cv2.waitKey(3)

    def findRedCircle(self, cI, cv_image):
        output = cv_image.copy()
        grey_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(grey_image, 100, 200)
        blur = cv2.GaussianBlur(edges, (5, 5), 0)

        circles = cv2.HoughCircles(blur, cv.CV_HOUGH_GRADIENT, 1.5, 1000, 0, 500,)
        cv2.imshow("outputgrey", grey_image)
        cv2.imshow("output edge", edges)
        cv2.imshow("output blur", blur)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                cv2.circle(output, (i[0], i[1]), i[2], (0, 255, 0), 2)

        cv2.imshow("output", np.hstack([cv_image, output]))
        cv2.waitKey(3)

    def findGreenCircle(self, cI, cv_image):
        output = cv_image.copy()
        grey_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(grey_image, 100, 200)
        blur = cv2.GaussianBlur(edges, (5, 5), 0)

        circles = cv2.HoughCircles(blur, cv.CV_HOUGH_GRADIENT, 1.5, 1000, 0, 500,)
        cv2.imshow("outputgrey green", grey_image)
        cv2.imshow("output edge green", edges)
        cv2.imshow("output blur green", blur)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                cv2.circle(output, (i[0], i[1]), i[2], (0, 255, 0), 2)

        cv2.imshow("output green", np.hstack([cv_image, output]))
        cv2.waitKey(3)


def main(args):
    rospy.init_node('Character_Finder', anonymous=True)
    cI = colourIdentifier()

    try:
        # Customize the following values so they are appropriate for your location
        x = cI.room1_entrance_x  # SPECIFY X COORDINATE HERE
        y = cI.room1_entrance_y  # SPECIFY Y COORDINATE HERE
        theta = 0
        if x - cI.room1_centre_x != 0:
            theta = math.atan((y-cI.room1_centre_y)/(x - cI.room1_centre_x))
        else:
            theta = math.atan((y - cI.room1_centre_y))

        theta -= math.pi
        # theta = 0  # SPECIFY THETA (ROTATION) HERE
        position = {'x': x, 'y': y}
        quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': np.sin(theta / 2.0), 'r4': np.cos(theta / 2.0)}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        # success = cI.goto(position, quaternion)
        # print(success)
        print(cI.sub)
        # if (success):
        # image = cI.imageCallback()
        # cI.findGreenCircle(cI, image)
        rospy.spin()

    except rospy.ROSInterruptException:
        print("failed")
        pass
    cv2.destroyAllWindows()


# Check if the node is executing in the main path
if __name__ == '__main__':
    main(sys.argv)
