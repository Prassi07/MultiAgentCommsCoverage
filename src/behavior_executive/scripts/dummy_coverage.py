#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from simple_mapf_sim.msg import CommsNodeMsg, CommsNodeArray


class CoveragePlanner(object):
    def __init__(self):
        
        self._target_pub = rospy.Publisher("/set_coverage/targets", CommsNodeArray, queue_size=1, latch=True)
        self.sim_example = rospy.get_param('~sim_file')
        

        test1 = CommsNodeArray()
        test1.nodes.append(CommsNodeMsg(x = 10, y = 0, id = 0))
        test1.nodes.append(CommsNodeMsg(x = 0, y = -10, id = 1))
        test1.nodes.append(CommsNodeMsg(x = 10, y = 10, id = 2))
        test1.nodes.append(CommsNodeMsg(x = -10, y = -10, id = 3))
        test1.nodes.append(CommsNodeMsg(x = -10, y = 10, id = 4))
        test1.nodes.append(CommsNodeMsg(x = 10, y = -10, id = 5))
        test1.nodes.append(CommsNodeMsg(x = -10, y = 0, id = 6))
        test1.nodes.append(CommsNodeMsg(x = 0, y = 10, id = 7))
        
        test2 = CommsNodeArray()
        test2.nodes.append(CommsNodeMsg(x = 10, y = 0, id = 0))
        test2.nodes.append(CommsNodeMsg(x = 0, y = -10, id = 1))
        test2.nodes.append(CommsNodeMsg(x = 10, y = 10, id = 2))
        test2.nodes.append(CommsNodeMsg(x = -10, y = -10, id = 3))
        test2.nodes.append(CommsNodeMsg(x = -10, y = 10, id = 4))
        test2.nodes.append(CommsNodeMsg(x = 10, y = -10, id = 5))
        test2.nodes.append(CommsNodeMsg(x = -10, y = 0, id = 6))
        test2.nodes.append(CommsNodeMsg(x = 0, y = 10, id = 7))
        
        self.sent = 0

        self.targets_dict = {'sim.yaml': test1, 'sim32.yaml': test2}

    def run(self):
        if self.sent == 0:
            rospy.sleep(10.0)
            self.sent = self.sent + 1
        elif self.sent == 1:
            self._target_pub.publish(self.targets_dict[self.sim_example])
            self.sent = self.sent + 1

if __name__ == "__main__":
    rospy.init_node("dummy_planner")
    node_ = CoveragePlanner()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        node_.run()
        rate.sleep()

    rospy.spin()
