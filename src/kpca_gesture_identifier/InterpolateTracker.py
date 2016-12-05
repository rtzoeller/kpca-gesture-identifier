from __future__ import division, print_function

from Tracker import Tracker
from Interpolater import linear


class InterpolateTracker(Tracker):
    def __init__(self, callback=None, numSamples=1):
        if not callback:
            callback = print
        super(InterpolateTracker, self).__init__(lambda points: callback(linear(points)), numSamples)


if __name__ == '__main__':
    InterpolateTracker()
