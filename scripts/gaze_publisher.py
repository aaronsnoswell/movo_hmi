#!/usr/bin/env python

"""
Simple ROS node to test publishing a gaze point for MOVO's eyes

Can also test using "rostopic pub -1 /gaze_point geometry_msgs/Point '{x: 1, y: 0}'"
"""

import rospy
import math

from geometry_msgs.msg import Point


def gaze_publisher(topic="/gaze_point"):
    """
    Publishes a geometry_msgs/Point object
    """

    pub = rospy.Publisher(topic, Point, queue_size=10)
    rospy.init_node('gaze_publisher', anonymous=True)
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        t = seconds = rospy.get_time()
        pub.publish(Point(
            math.cos(t),
            math.sin(t),
            0
        ))
        rate.sleep()


if __name__ == '__main__':
    try:
        gaze_publisher()
    except rospy.ROSInterruptException:
        pass
