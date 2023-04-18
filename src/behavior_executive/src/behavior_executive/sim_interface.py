import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import UInt8
from simple_mapf_sim.msg import Plan, Waypoint, PoseStampedArray, BatteryArray


class SimInterface(object):
    def __init__(self):

        self.covered = -1.0
        self.remaining_covered = -1.0

        self._coverage_sub = rospy.Subscriber(
            "/sim/coverage_grid", OccupancyGrid, self.coverage_callback
        )

    def coverage_callback(self, msg):
        self.covered = (np.array(msg.data) == 25.0).sum()
        self.remaining_covered = (np.array(msg.data) == 1.0).sum()

    def get_coverage(self):
        return self.covered, self.remaining_covered
