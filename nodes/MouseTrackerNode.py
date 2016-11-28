#!/usr/bin/env python
from __future__ import division, print_function

import rospy
import numpy as np

from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped

from kpca_gesture_identifier.msg import Trajectory
from kpca_gesture_identifier.Tracker import Tracker

publisher = rospy.Publisher("trajectory", Trajectory, queue_size=5)


def callback(points):
    if rospy.is_shutdown():
        return

    trajectory = Trajectory()
    trajectory.points = []

    baseTime = rospy.Time.now() - rospy.Duration(points[-1].t)
    for point in points:
        pointStamped = PointStamped()
        pointStamped.header = Header()
        pointStamped.header.stamp = baseTime + rospy.Duration(point.t)
        pointStamped.point.x = point.x
        pointStamped.point.y = point.y
        trajectory.points.append(pointStamped)

    publisher.publish(trajectory)


if __name__ == '__main__':
    rospy.init_node("GestureTracker", anonymous=True)
    Tracker(callback, numSamples=-1)
