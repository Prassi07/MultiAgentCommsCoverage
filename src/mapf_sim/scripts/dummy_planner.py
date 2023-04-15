#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from simple_mapf_sim.msg import Plan, Waypoint, PoseStampedArray

class WaypointPub(object):
    def __init__(self):
        self._waypoint_pub = rospy.Publisher(
            "/global_path", Plan, queue_size=10
        )

        self._odom_sub = rospy.Subscriber(
            "/drone_sim/vehicle_poses", PoseStampedArray, self.vehicle_odom_callback
        )

        x_points = [800., 850., 800., 850.]
        y_points = [500., 500., 550., 500.]
        z_points = [20., 25., 35., 25.]

        self.plan = Plan()

        self.veh_id = 0

        self.plan.vehicle_id = self.veh_id

        for x, y, z in zip(x_points, y_points, z_points):
            wp = Waypoint()
            wp.position.position.x = x
            wp.position.position.y = y
            wp.position.position.z = z
            wp.waypoint_type = 1
            self.plan.plan.append(wp)

    def run(self):
        first = self.plan.plan[0]
        self.plan.plan = self.plan.plan[1:]
        self.plan.plan.append(first)
        self._waypoint_pub.publish(self.plan)


    def vehicle_odom_callback(self, msg):
        self.odom = msg.poses[self.veh_id]


if __name__ == "__main__":
    rospy.init_node("dummy_planner")
    node_ = WaypointPub()
    rate = rospy.Rate(0.3)

    while not rospy.is_shutdown():
        node_.run()
        rate.sleep()

    rospy.spin()
