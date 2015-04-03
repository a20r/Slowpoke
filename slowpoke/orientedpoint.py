
import math


class OrientedPoint(object):

    def __init__(self, x, y, theta=0):
        self.x = x
        self.y = y
        self.theta = theta

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_theta(self):
        return self.theta

    def euclid_dist(self, other):
        x_part = pow(self.x - other.get_x(), 2)
        y_part = pow(self.y - other.get_y(), 2)
        return math.sqrt(x_part + y_part)

    def angle_between(self, other):
        return self.theta - other.get_theta()

    def to_unit_vector(self):
        return OrientedPoint(math.cos(self.theta), math.sin(self.theta))

    def __sub__(self, other):
        new_x = self.x - other.get_x()
        new_y = self.y - other.get_y()
        new_theta = self.theta - other.get_theta()
        return OrientedPoint(new_x, new_y, new_theta)

    def __add__(self, other):
        new_x = self.x + other.get_x()
        new_y = self.y + other.get_y()
        new_theta = self.theta + other.get_theta()
        return OrientedPoint(new_x, new_y, new_theta)

    def __str__(self):
        pass

    def __repr__(self):
        pass


def make(x, y, theta=0):
    return OrientedPoint(x, y, theta)
