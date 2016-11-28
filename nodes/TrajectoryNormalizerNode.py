#!/usr/bin/env python
from __future__ import division, print_function

import rospy
from geometry_msgs.msg import PointStamped
from kpca_gesture_identifier.Interpolater import normalize
from kpca_gesture_identifier.Point import Point as kpcaPoint
from kpca_gesture_identifier.msg import Trajectory

publisher = rospy.Publisher("trajectory_normalized", Trajectory, queue_size=5)


def callback(trajectory):
    # Convert to the point representation used by the KPCA algorithm
    points = [kpcaPoint(stampedPoint.point.x, stampedPoint.point.y, stampedPoint.header.stamp.to_sec())
              for stampedPoint in trajectory.points]

    pointsNormalized = normalize(points)

    # Convert to the point representation used by ROS
    trajectoryNormalized = Trajectory()
    for p in pointsNormalized:
        pointStamped = PointStamped()
        pointStamped.header.stamp = rospy.Time.from_sec(p.t)
        pointStamped.point.x = p.x
        pointStamped.point.y = p.y
        trajectoryNormalized.points.append(pointStamped)
    publisher.publish(trajectoryNormalized)


subscriber = rospy.Subscriber("trajectory", Trajectory, callback)

if __name__ == '__main__':
    rospy.init_node("TrajectoryNormalizer", anonymous=True)
    rospy.spin()
