
import rospy
import freenect
from nav_msgs.msg import Odometry
import tf
import numpy as np
import math
import orientedpoint
import point


class Sensors(object):

    def __init__(self):
        rospy.Subscriber("odom", Odometry, self.set_position)
        self.x = 0
        self.y = 0
        self.theta = 0

    def kinect_get_minimum(self):
        depth, _ = freenect.sync_get_depth()
        min_val = depth[0:400, 0:640].min()
        return min_val

    def kinect_get_hps(self, num_hps=100):
        """
        Gets the absolute hit points
        """
        depth_org, _ = freenect.sync_get_depth()
        depth = np.fliplr(depth_org)
        height = 240
        hps = list()
        for i in np.linspace(20, 620, num_hps):
            alpha = math.radians(-28.5 + 57 * i / 639)
            dist = depth[height, i] / 1000
            h_x = self.x + dist * math.cos(alpha)
            h_y = self.y + dist * math.sin(alpha)
            hps.append(point.make(h_x, h_y))

        return hps

    def set_position(self, odom):
        self.x = odom.pose.pose.position.x
        self.y = odom.pose.pose.position.y
        quat = odom.pose.pose.orientation
        self.theta = tf.transformations.euler_from_quaternion((
            quat.x, quat.y, quat.z, quat.w
        ))[2]

    def get_position(self):
        return orientedpoint.make(self.x, self.y, self.theta)
