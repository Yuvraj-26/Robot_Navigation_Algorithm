#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool
import sys


class Timer():
    def __init__(self):
        # Get system start time until clock actually starts
        self.start_time = rospy.get_time()
        while self.start_time == 0.0:
            self.start_time = rospy.get_time()

        self.end_time = rospy.get_time()
        self.ended = False
        self.sub_shutdown = rospy.Subscriber('shutdown_topic', Bool, self.printFinishedTimer)
        self.pub_timer = rospy.Publisher('timer_topic', String, queue_size=10)

    def printFinishedTimer(self, data):
        # If the program has finished running
        if data.data:
            # Calculate end time once
            if not self.ended:
                self.end_time = rospy.get_time()

            # Get total duration
            total_time = self.end_time - self.start_time

            # Publish duration as seconds or minutes
            if total_time < 60:
                self.pub_timer.publish("Completed in " + str(total_time) + " seconds")
            else:
                mins = int(total_time / 60)
                secs = total_time % 60
                self.pub_timer.publish("Completed in " + str(mins) + ":" + str(secs))
            self.ended = True


def main(args):
    rospy.init_node('timer', anonymous=True)
    t = Timer()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


# Check if the node is executing in the main path
if __name__ == '__main__':
    main(sys.argv)
