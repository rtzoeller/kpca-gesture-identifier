from __future__ import division, print_function

import numpy as np

from InterpolateTracker import InterpolateTracker


def main(points):
    A = np.array([[p.x for p in points], [p.y for p in points]]).T
    np.save("out.npy", A)


if __name__ == '__main__':
    InterpolateTracker(main)
