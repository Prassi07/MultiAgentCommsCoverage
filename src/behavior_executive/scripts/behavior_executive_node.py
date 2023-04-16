#!/usr/bin/env python3

import rospy
from behavior_executive.behavior_executive import BehaviorExecutive


if __name__ == "__main__":
    rospy.init_node("dummy_planner")
    node_ = BehaviorExecutive()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        node_.run()
        rate.sleep()

    rospy.spin()
