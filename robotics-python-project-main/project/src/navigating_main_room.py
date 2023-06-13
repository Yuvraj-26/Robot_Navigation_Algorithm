#!/usr/bin/env python
from __future__ import division
import cv2
import numpy as np
import rospy
import sys
import yaml
from os.path import expanduser
import math
from std_msgs.msg import Bool

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

green_discovered = False
green_discovered_at_least_once = False
red_discovered = False
moving = False


class Navigator():
    def __init__(self):
        # Subscribers
        self.red_circle_sub = rospy.Subscriber('red_circle_topic', Bool, self.callbackRedCircle)
        self.green_circle_sub = rospy.Subscriber('green_circle_topic', Bool, self.callbackGreenCircle)
        self.image_sub = rospy.Subscriber('image_topic', Image, self.imageCallback)
        # Publishers
        self.moving_pub = rospy.Publisher('turtle_bot_main_room_moving_topic', Bool, queue_size=10)
        self.in_room_pub = rospy.Publisher('turtle_bot_in_green_room', Bool, queue_size=10)

        # Bool describing if the turtlebot has entered the room
        self.in_room = False

        # Get home directory
        home = expanduser("~")
        # and open input_points.yaml file
        with open(home + "/catkin_ws/src/group_project/project/example/input_points.yaml", 'r') as stream:
            points = yaml.safe_load(stream)

        # Use file data to set our input points variables
        self.room1_entrance_x = points['room1_entrance_xy'][0]
        self.room1_entrance_y = points['room1_entrance_xy'][1]
        self.room2_entrance_x = points['room2_entrance_xy'][0]
        self.room2_entrance_y = points['room2_entrance_xy'][1]

        self.room1_centre_x = points['room1_centre_xy'][0]
        self.room1_centre_y = points['room1_centre_xy'][1]
        self.room2_centre_x = points['room2_centre_xy'][0]
        self.room2_centre_y = points['room2_centre_xy'][1]

        self.bridge = CvBridge()
        self.cv_image = np.zeros((480, 640, 3), np.uint8)

    def imageCallback(self, data):
        # Convert image message to cv image
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print("Image conversion failed")
            print(e)
            pass

    def callbackRedCircle(self, data):
        # Updates variable to indicate if the red circle is found
        global red_discovered
        global moving
        if not moving:
            red_discovered = data.data

        # For Debugging
        # if not moving:
        #     print("red found " + str(data.data))

    def callbackGreenCircle(self, data):
        # Updates variable to indicate if the green circle is found and takes pictures if this is the case
        global green_discovered
        global green_discovered_at_least_once
        global moving
        if green_discovered_at_least_once == False and data.data:
            cv2.imwrite('green_circle.png', self.cv_image)
            green_discovered_at_least_once = True
        if not moving:
            green_discovered = data.data

        # For Debugging
        # if not moving:
        #     print("green found " + str(data.data))

    def center_of_room_1(self):
        # Sents turtlebot to center of room 1
        room_1_navigator = GoToPose()

        x = self.room1_centre_x
        y = self.room1_centre_y
        theta = 0

        position = {'x': x, 'y': y}
        quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': np.sin(theta / 2.0), 'r4': np.cos(theta / 2.0)}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])

        # Publishing if the turtle bot is moving to not interfere with other nodes
        self.moving_pub.publish(True)
        success = room_1_navigator.goto(position, quaternion)

        # Turtle bot has reached goal or failed to reach the goal
        if success:
            rospy.loginfo("Hooray, reached the desired pose")
            self.in_room = True
        else:
            rospy.loginfo("The base has failed to reach the desired pose")
        self.moving_pub.publish(False)
        rospy.sleep(1)

    def center_of_room_2(self):
        # Sents turtlebot to center of room 2
        room_2_navigator = GoToPose()

        x = self.room2_centre_x
        y = self.room2_centre_y
        theta = 0

        position = {'x': x, 'y': y}
        quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': np.sin(theta / 2.0), 'r4': np.cos(theta / 2.0)}
        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        
        # Publishing if the turtle bot is moving to not interfere with other nodes
        self.moving_pub.publish(True)
        success = room_2_navigator.goto(position, quaternion)

        # Turtle bot has reached goal or failed to reach the goal
        if success:
            rospy.loginfo("Hooray, reached the desired pose")
            self.in_room = True
        else:
            rospy.loginfo("The base has failed to reach the desired pose")
        self.moving_pub.publish(False)
        rospy.sleep(1)

    def entrance_of_room_1(self):
        # Sents turtlebot to entrance of room 1
        navigator = GoToPose()
        x = self.room1_entrance_x
        y = self.room1_entrance_y
        
        # Create vector pointing from entrance to room center
        # Turtle bot faces the direction of this vector once it reaches the entrance
        theta = 0
        if x - self.room1_centre_x != 0:
            theta = math.atan((y - self.room1_centre_y) / (x - self.room1_centre_x))
        else:
            theta = math.atan((y - self.room1_centre_y))
        theta -= math.pi

        position = {'x': x, 'y': y}
        quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': np.sin(theta / 2.0), 'r4': np.cos(theta / 2.0)}
        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        
        # Publishing if the turtle bot is moving to not interfere with other nodes
        self.moving_pub.publish(True)
        success = navigator.goto(position, quaternion)

        # Turtle bot has reached goal or failed to reach the goal
        if success:
            rospy.loginfo("Hooray, reached the desired pose")
        else:
            rospy.loginfo("The base has failed to reach the desired pose")

        self.moving_pub.publish(False)
        rospy.sleep(1)

    def entrance_of_room_2(self):
        # Sents turtlebot to entrance of room 1
        room_2_navigator = GoToPose()
        x = self.room2_entrance_x
        y = self.room2_entrance_y

        # Create vector pointing from entrance to room center
        # Turtle bot faces the direction of this vector once it reaches the entrance
        theta = 0
        if x - self.room2_centre_x != 0:
            theta = math.atan((y - self.room2_centre_y) / (x - self.room2_centre_x))
        else:
            theta = math.atan((y - self.room2_centre_y))
        theta -= math.pi

        position = {'x': x, 'y': y}
        quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': np.sin(theta / 2.0), 'r4': np.cos(theta / 2.0)}
        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        
        # Publishing if the turtle bot is moving to not interfere with other nodes
        self.moving_pub.publish(True)
        success = room_2_navigator.goto(position, quaternion)

        # Turtle bot has reached goal or failed to reach the goal
        if success:
            rospy.loginfo("Hooray, reached the desired pose")
        else:
            rospy.loginfo("The base has failed to reach the desired pose")

        self.moving_pub.publish(False)
        rospy.sleep(1)


class GoToPose():  # x
    def __init__(self):
        self.goal_sent = False
        rospy.on_shutdown(self.shutdown)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")
        self.move_base.wait_for_server()

    def goto(self, pos, quat):  # x
        global moving
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))
        moving = True
        self.move_base.send_goal(goal)

        success = self.move_base.wait_for_result(rospy.Duration(60))

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False

        moving = False

        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)


def main(args):
    # Initiate node and set room flag variables
    rospy.init_node('navigating_main_room', anonymous=True)
    count_track = 0
    global red_discovered
    first_flag = False
    second_flag = False
    global green_discovered
    global moving
    navigator = Navigator()

    # Loop while ros is running and the turtlebot is not in a room
    while not rospy.is_shutdown() and not navigator.in_room:
        # Send to first room entrance
        if first_flag == False:
            print("sending to room 1")
            navigator.entrance_of_room_1()
            count_track += 1

        # If a green circle is discovered send to first room center
        if green_discovered == True and count_track == 1:
            print("moving to to room 1 center")
            navigator.center_of_room_1()

        # Send to second room entrance if not already in first room center
        if second_flag == False and green_discovered == False and not navigator.in_room:
            print("sending to to room 2")
            navigator.entrance_of_room_2()
            count_track += 1

        # If green circle is discovered at second room entrance then move to second room center, otherwise move to
        # first room center
        if not navigator.in_room:
            if green_discovered == True and count_track == 2:
                navigator.center_of_room_2()
            else:
                navigator.center_of_room_1()

        # publish if in room or not
        navigator.in_room_pub.publish(navigator.in_room)

    while not rospy.is_shutdown():
        # publish if in room or not
        navigator.in_room_pub.publish(navigator.in_room)

if __name__ == '__main__':
    main(sys.argv)
