#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, Int32MultiArray, String


def callbackRed(data):
    print("Red flag is " + str(data.data))


def callbackGreen(data):
    print("Green flag is " + str(data.data))


def callbackMustard(data):
    print("callbackMustard " + str(data.data))


def callbackPeacock(data):
    print("callbackPeacock " + str(data.data))


def callbackPlum(data):
    print("callbackPlum " + str(data.data))


def callbackScarlet(data):
    print("callbackScarlet " + str(data.data))


def callbackRectangle(data):
    print("callbackRectangle " + str(data.data))


def callbackRectangleInBounds(data):
    print("callbackRectangleInBounds " + str(data.data))


def callbackRectangleInts(data):
    print("callbackRectangleInts " + str(data.data))


def callbackShutdown(data):
    print("callbackShutdown " + str(data.data))


def callbackTimer(data):
    print("callbackTimer " + str(data.data))


def callbackInGreenRoom(data):
    print("callbackInGreenRoom " + str(data.data))


def callbackTurtleBotMainRoomMovingTopic(data):
    print("callbackTurtleBotMainRoomMovingTopic " + str(data.data))


def listener():
    rospy.init_node('listener', anonymous=True)
    sub = rospy.Subscriber('red_circle_topic', Bool, callbackRed)
    sub = rospy.Subscriber('green_circle_topic', Bool, callbackGreen)
    sub = rospy.Subscriber('mustard_topic', Bool, callbackMustard)
    sub = rospy.Subscriber('peacock_topic', Bool, callbackPeacock)
    sub = rospy.Subscriber('plum_topic', Bool, callbackPlum)
    sub = rospy.Subscriber('scarlet_topic', Bool, callbackScarlet)
    sub = rospy.Subscriber('rectangle_topic', Bool, callbackRectangle)
    sub = rospy.Subscriber('rectangle_in_bounds_topic', Bool, callbackRectangleInBounds)
    sub = rospy.Subscriber('rectangle_ints_topic', Int32MultiArray, callbackRectangleInts)
    sub = rospy.Subscriber('shutdown_topic', Bool, callbackShutdown)
    sub = rospy.Subscriber('timer_topic', String, callbackTimer)
    sub = rospy.Subscriber('turtle_bot_in_green_room', Bool, callbackInGreenRoom)
    sub = rospy.Subscriber('turtle_bot_main_room_moving_topic', Bool, callbackTurtleBotMainRoomMovingTopic)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
