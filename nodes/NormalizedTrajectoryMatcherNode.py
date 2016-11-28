#!/usr/bin/env python
from __future__ import division, print_function

import rospy
import numpy as np
from std_msgs.msg import String
from kpca_gesture_identifier.KPCA import Predictor
from kpca_gesture_identifier.Point import Point
from kpca_gesture_identifier.msg import Trajectory

import rospkg
rp = rospkg.RosPack()
path = rp.get_path("kpca_gesture_identifier")

publisher = rospy.Publisher("trajectory_shape", String, queue_size=5)


def callback(trajectory):
    # Convert to the point representation used by the KPCA algorithm
    points = [Point(stampedPoint.point.x, stampedPoint.point.y, stampedPoint.header.stamp.to_sec())
              for stampedPoint in trajectory.points]

    # Read in saved trajectories
    predictor = Predictor()
    for i in range(1, 21):
        predictor.addTrajectory(np.load(path + "/" + "data/L/{0}.npy".format(i)), "L")
    for i in range(1, 21):
        predictor.addTrajectory(np.load(path + "/" + "data/O/{0}.npy".format(i)), "O")
    for i in range(1, 21):
        predictor.addTrajectory(np.load(path + "/" + "data/R/{0}.npy".format(i)), "R")
    for i in range(1, 21):
        predictor.addTrajectory(np.load(path + "/" + "data/S/{0}.npy".format(i)), "S")
    for i in range(1, 21):
        predictor.addTrajectory(np.load(path + "/" + "data/W/{0}.npy".format(i)), "W")

    # Guess a new point
    A = np.array([[p.x for p in points], [p.y for p in points]]).T
    prediction, projection = predictor.classify(A)
    print(prediction)
    publisher.publish(prediction)


subscriber = rospy.Subscriber("trajectory_normalized", Trajectory, callback)

if __name__ == '__main__':
    rospy.init_node("NormalizedTrajectoryMatcher", anonymous=True)
    rospy.spin()
