#!/usr/bin/env python
import rospy
import sys
import math
import random

from geometry_msgs.msg import Twist, Vector3, Pose, Point, Quaternion
from std_msgs.msg import Bool
from kobuki_msgs.msg import BumperEvent


# notice about angle handling:
#   All angles shown in numbers are in degrees
#   All rotations performed must be converted to radians using angle*(PI/180)

class greenNavigation():
    def __init__(self):
        # internal flag for handling execution
        self.in_green_room = False  # Change to True to test code in isolation
        self.nav_completed = False
        self.rectangle_flag = False
        self.bumped = False

        # subscribers to determine when to start execution
        self.green_room_flag_sub = rospy.Subscriber('turtle_bot_in_green_room', Bool, self.greenRoomFlagCallback)
        self.rectangle_detection_sub = rospy.Subscriber('rectangle_topic', Bool, self.rectangleFlagCallback)
        self.bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumperHandler)

        # Publishers
        self.movement_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
        self.stop_circle_finder_pub = rospy.Publisher('turtle_bot_main_room_moving_topic', Bool, queue_size=10)

        self.rate = rospy.Rate(10)  # 10hz

        # simple movement to rotate anticlockwise around a point
        self.rotation_angle = 10  # change for more precision
        self.rotate = Twist()
        self.rotate.angular.z = math.radians(self.rotation_angle)

        self.move_forward = Twist()
        self.move_forward.linear.x = 0.2

        self.cam_adjust = Twist()
        self.cam_adjust.angular.z = - math.radians(90)

        # give ros time to initialise the subscribers and publishers
        rospy.sleep(2)

    # inside_green_room subscriber callback
    def greenRoomFlagCallback(self, data):
        self.in_green_room = data.data

    # rectangle_topic subscriber callback
    def rectangleFlagCallback(self, data):
        self.rectangle_flag = data.data

    # bumper handler subscriber callback
    # bumped flag reset in randomMovement function
    def bumperHandler(self, data):
        if (data.state == BumperEvent.PRESSED):
            self.bumped = True

    # main navigation function
    def startNavigation(self):
        moved = 0
        while (not rospy.is_shutdown()):
            if (self.in_green_room == True):
                # prevent circle_Finder from published Twist movements
                self.stop_circle_finder_pub.publish(True)
                # check for the portrait from initial position
                seen = self.lookForRectange()
                if seen:
                    print("I can see It!!!")
                    if moved <= 8:
                        status = self.moveTowardPortrait()
                        moved += 1
                        for i in range(10):
                            self.movement_pub.publish(self.cam_adjust)
                            self.rate.sleep()
                    else:
                        print("movement complete")

                else:
                    print("I cant see it :(")
                    status = 1
                    while status == 1:
                        status = self.randomMovement()

            else:
                # debugging print for main function idling
                print("waiting...")

    # rotates around current pose to look for character in the scene
    # CATION: does NOT reset to rotation from before function, exits with new rotation 
    # Return: True if portrait is found, False otherwise
    def lookForRectange(self):
        rotation = 0
        while (rotation < 360 and self.rectangle_flag == False):
            for i in range(10):
                self.movement_pub.publish(self.rotate)
                # sleep after issuing rotation, give chance for message and subsequent processing by other nodes
                self.rate.sleep()
            rospy.sleep(1)
            print("rotated 20 degrees?")
            rotation = rotation + self.rotation_angle

        if self.rectangle_flag == False:
            return False
        else:
            return True

    # Find the middle rotation for the portrait, then moves towards it
    # makes the assumption portrait is in view of turtlebot already AND that rotating anti-clockwise will move it into frame
    # Return: 0 for when turtle is in correct position for the photo, 1 for successful movement
    def moveTowardPortrait(self):
        rotation = 0
        while (self.rectangle_flag == True):
            for i in range(10):
                self.movement_pub.publish(self.rotate)
                self.rate.sleep()
            rospy.sleep(1)
            rotation = rotation + self.rotation_angle

        # find angle to focus the portrait
        middle = (rotation / 2) + 5

        # build same rotation message but in the clockwise direction
        rotation_msg = Twist()
        rotation_msg.angular.z = - math.radians(self.rotation_angle)

        # handles rotations above the rotation angle, rotating the rotation angle x times
        if (middle > self.rotation_angle):
            while (middle > self.rotation_angle):
                for i in range(10):
                    self.movement_pub.publish(rotation_msg)
                    self.rate.sleep()
                middle = middle - self.rotation_angle

        # handles rotations under the rotation angle, if they are needed
        if (middle > 0):
            rotation_msg.angular.z = - math.radians(middle)
            for i in range(10):
                self.movement_pub.publish(rotation_msg)
                self.rate.sleep()

        # Move Forward Code
        if (rotation < 30):
            return 0
        else:
            for i in range(10):
                self.movement_pub.publish(self.move_forward)
                self.rate.sleep()
            rospy.sleep(1)
            return 1

    def randomMovement(self):
        angle = random.randrange(0, 360, 1)

        # build same rotation message but in the clockwise direction
        rotation_msg = Twist()
        rotation_msg.angular.z = - math.radians(self.rotation_angle)

        if (angle > self.rotation_angle):
            while (angle > self.rotation_angle):
                for i in range(10):
                    self.movement_pub.publish(rotation_msg)
                    self.rate.sleep()
                angle = angle - self.rotation_angle

        # handles rotations under the rotation angle, if they are needed
        if (angle > 0):
            rotation_msg.angular.z = - math.radians(angle)
            for i in range(10):
                self.movement_pub.publish(rotation_msg)
                self.rate.sleep()

        # move forward set amount
        for i in range(10):
            self.movement_pub.publish(self.move_forward)
            self.rate.sleep()
        rospy.sleep(1)

        # check if the turtlebot has collided with the wall
        for i in range(40):
            self.movement_pub.publish(self.cam_adjust)
            self.rate.sleep()
        # give ros time to recieve and process the bumper response
        rospy.sleep(1)

        if self.bumped == True:
            # rotate 180 degrees and move back to original position
            for i in range(20):
                self.movement_pub.publish(self.cam_adjust)
                self.rate.sleep()
            for i in range(10):
                self.movement_pub.publish(self.move_forward)
                self.rate.sleep()
            rospy.sleep(1)

            self.bumped = False
            return 1
        else:
            return 0


def main(args):
    rospy.init_node('green_room_navigator', anonymous=True)
    print("init green_room_navigator")
    gNav = greenNavigation()
    try:
        gNav.startNavigation()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main(sys.argv)
