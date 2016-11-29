#!/usr/bin/env python
from __future__ import division, print_function

import rospy
import numpy as np
import os
from std_msgs.msg import String
from kpca_gesture_identifier.KPCA import Predictor
from kpca_gesture_identifier.Point import Point
from kpca_gesture_identifier.msg import Trajectory

import rospkg

publisher = rospy.Publisher("trajectory_shape", String, queue_size=5)

# Read in saved trajectories
rp = rospkg.RosPack()
path = rp.get_path("kpca_gesture_identifier")
gestures = rospy.get_param("gestures")
predictor = Predictor()
count = 0
for gesture in gestures:
    for directory, subdirectories, filePaths in os.walk(os.path.join(path, gestures[gesture])):
        for filePath in filePaths:
            fullPath = os.path.join(directory, filePath)
            predictor.addTrajectory(np.load(fullPath), gesture)
            count += 1
rospy.logdebug("{0} samples loaded".format(count))


def callback(trajectory):
    # Convert to the point representation used by the KPCA algorithm
    points = [Point(stampedPoint.point.x, stampedPoint.point.y, stampedPoint.header.stamp.to_sec())
              for stampedPoint in trajectory.points]

    # Guess a new point
    A = np.array([[p.x for p in points], [p.y for p in points]]).T
    prediction, projection = predictor.classify(A)
    publisher.publish(prediction)


subscriber = rospy.Subscriber("trajectory_normalized", Trajectory, callback)

if __name__ == '__main__':
    rospy.init_node("NormalizedTrajectoryMatcher", anonymous=True)
    rospy.spin()
