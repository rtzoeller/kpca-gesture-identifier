class Point(object):
    def __init__(self, x, y, t):
        self.x = x
        self.y = y
        self.t = t

    def __repr__(self):
        return "({0}, {1}, {2})".format(self.x, self.y, self.t)