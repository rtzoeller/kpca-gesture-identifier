from __future__ import division, print_function

import numpy as np

from Tracker import Tracker
from Interpolater import scale


def main(points):
    scaled = scale(points)
    A = np.array([[p.x for p in scaled], [p.y for p in scaled]]).T
    np.save("out.npy", A)


if __name__ == '__main__':
    Tracker(main)
