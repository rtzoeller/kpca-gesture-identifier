from __future__ import division, print_function

import numpy as np

from Tracker import Tracker

i = 1


def main(points):
    global i
    A = np.array([[p.x for p in points], [p.y for p in points], [p.t for p in points]]).T
    np.save(str(i) + ".npy", A)
    i += 1


if __name__ == '__main__':
    Tracker(main, 20)
