#!/usr/bin/env python
from __future__ import division, print_function

import rospy
from geometry_msgs.msg import PointStamped
from kpca_gesture_identifier.Tracker import Tracker
from kpca_gesture_identifier.msg import Trajectory

publisher = rospy.Publisher("trajectory", Trajectory, queue_size=5)


def callback(points):
    if rospy.is_shutdown():
        return

    trajectory = Trajectory()
    baseTime = rospy.Time.now() - rospy.Duration(points[-1].t)
    for p in points:
        pointStamped = PointStamped()
        pointStamped.header.stamp = baseTime + rospy.Duration(p.t)
        pointStamped.point.x = p.x
        pointStamped.point.y = p.y
        trajectory.points.append(pointStamped)

    publisher.publish(trajectory)


if __name__ == '__main__':
    rospy.init_node("MouseTracker", anonymous=True)
    Tracker(callback, numSamples=-1)
