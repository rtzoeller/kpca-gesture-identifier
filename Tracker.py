from __future__ import division, print_function
import six.moves.tkinter as tk
from six.moves import filter, range

import time

from Point import Point

WIDTH = 640
HEIGHT = 480


class Tracker(object):
    def __init__(self, callback=None):
        # Initialize tkinter
        self.root = tk.Tk()
        self.canvas = tk.Canvas(width=WIDTH, height=HEIGHT)
        self.canvas.bind('<Button-1>', self.down)
        self.canvas.bind('<ButtonRelease-1>', self.up)
        self.canvas.bind('<B1-Motion>', self.update)
        self.canvas.pack()

        if callback is None:
            self.callback = print
        else:
            self.callback = callback

        self.points = None
        self.root.mainloop()

    def down(self, event):
        assert (self.points is None)
        self.points = []
        self.update(event)

    def update(self, event):
        self.points.append(Point(event.x, event.y, time.time()))

    def up(self, event):
        self.update(event)
        self.process()
        self.root.destroy()
        self.callback(self.points)
        self.points = None

    def process(self):
        initial_time = self.points[0].t

        # Remap to start at t = 0
        for p in self.points:
            p.t -= initial_time

        # Remove points that are out of range
        self.points = filter(lambda p: 0 <= p.x < WIDTH and 0 <= p.y < HEIGHT, self.points)

        # Drop down to 30fps to match Kinect
        points30 = []
        count = 0
        for p in self.points:
            r = p.t // (1 / 30)
            if r >= count:
                points30.append(p)
                count = r + 1
        self.points = points30


if __name__ == '__main__':
    Tracker()
