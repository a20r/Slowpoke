
import rospy
from geometry_msgs.msg import Twist
import numpy as np
import sensors as sns
import math
import point


class PF(object):

    REP = 100

    def __init__(self, goal, s_radius=0.4):
        rospy.init_node('slowpoke_pf', anonymous=False)
        self.sensors = sns.Sensors()
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('mobile_base/commands/velocity', Twist)
        self.s_radius = s_radius
        self.goal = goal

    def get_angle_to(self, pos):
        current_pos = self.sensors.get_position()
        return math.atan2(pos.y - current_pos.y,
                          pos.x - current_pos.x)

    def get_dist_to_goal(self, pos=None):
        if not pos:
            pos = self.sensors.get_position()
        return pos.euclid_dist(self.goal)

    def get_goal_potential(self, pos):
        return pow(pos.euclid_dist(self.goal), 2)

    def get_obstacle_potential(self, pos, hp):
        return self.REP / (pow(pos.euclid_dist(hp), 2) + 0.01)

    def get_potential(self, pos, hps):
        gp = self.get_goal_potential(pos)
        op = max(map(lambda hp: self.get_obstacle_potential(pos, hp), hps))
        return gp + op

    def get_best_sample(self, num_samples=100):
        current_pos = self.sensors.get_position()
        hps = self.sensors.kinect_get_hps()
        min_pot = None
        min_pos = None
        for alpha in np.linspace(math.radians(-28.5), math.radians(28.5),
                                 num_samples):
            x = current_pos.get_x() + self.s_radius * math.sin(alpha)
            y = current_pos.get_y() + self.s_radius * math.cos(alpha)
            p = point.make(x, y)
            pot = self.get_potential(p, hps)
            if min_pot is None or pot < min_pot:
                min_pot = pot
                min_pos = p

        return min_pos, min_pot

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.get_dist_to_goal() < 0.5:
                self.shutdown()
                break

            current_pos = self.sensors.get_position()
            b_s, p = self.get_best_sample()

            move_cmd = Twist()
            move_cmd.angular.z = -1.1 * math.sin(
                current_pos.theta - self.get_angle_to(b_s))
            move_cmd.linear.x = 0.2

            self.cmd_vel.publish(move_cmd)
            r.sleep()

    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
