#!/usr/bin/env python
from __future__ import division, print_function

import numpy as np
import rospkg
import rospy
import time
import os
from kpca_gesture_identifier.msg import Trajectory

rp = rospkg.RosPack()
path = rp.get_path("kpca_gesture_identifier")


def callback(trajectory):
    points = trajectory.points
    try:
        os.mkdir(os.path.join(path, "saved"), 0o755)
    except OSError:
        pass
    np.save(os.path.join(path, "saved", str(int((time.time() + 0.5) * 1000)) + ".npy"),
            np.array([[p.point.x for p in points], [p.point.y for p in points],
                      [p.header.stamp.to_sec() for p in points]]).T)


subscriber = rospy.Subscriber("trajectory", Trajectory, callback)

if __name__ == '__main__':
    rospy.init_node("TrajectorySaver", anonymous=True)
    rospy.spin()
