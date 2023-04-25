import math
from enum import Enum
import rospy
import time
# msgs
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Float32, Bool
from planner_pkg.msg import PlannerType
from simple_mapf_sim.msg import Plan, PoseStampedArray, CommsNodeArray, RobotArray, RobotInfo, CommsNodeMsg, MultiRobotPlan
from jsk_rviz_plugins.msg import OverlayText
from itertools import zip_longest
# application
from behavior_executive.sim_interface import SimInterface

class BehaviorStates(Enum):
    INIT = 0
    COMPUTE_ASSIGNMENTS = 1
    TRIGGER_PLANNER = 2
    WAIT_EXECUTION = 3
    COLLECTING_NODE_INFO = 4
    ALL_NODES_PLACED = 5

class BehaviorExecutive(object):
    def __init__(self):
        self.sim_interface = SimInterface()

        self.reached_goal = False
        self.all_targets_inited = False
        self.plan_sent = False
        self.executing_plan = False
        self.plan_received = False
        
        self.got_start_states = False
        self.robots_info = []
        self.all_targets = []
        self.sent_targets = []
        
        self.collision = False
        
        self.state = BehaviorStates.INIT
        self.text = "Initializing..."
        
        self.state_counter = 0
        self.start_states = PoseStampedArray()
        self.goal_locations = CommsNodeArray()
         
        self.assignment_type = int(rospy.get_param("~assignment_type")) # 1 - ECBS with sequential, 2 - ECBSTA, 3 - CBS - explicit TA based on distance
        self.suboptimal_w = rospy.get_param("~planner_w")
        
        self.planner_assign_map = {1: 1, 2: 2, 3: 3}
        self.compute_plan_msg = PlannerType(planner_type = self.planner_assign_map[self.assignment_type], suboptimality_bound = self.suboptimal_w)
        
        self._covered_pub = rospy.Publisher( "/executive/covered_voxels", Float32, queue_size=5)
        self._remaining_covered_pub = rospy.Publisher("/executive/remaining_voxels", Float32, queue_size=5)
        self.viz_text_pub = rospy.Publisher("/executive/viz/text", OverlayText, queue_size=5)
        self.assignment_time_pub = rospy.Publisher("/executive/assignment_time", Float32, queue_size=5)
        
        self.planner_starts_pub = rospy.Publisher("/executive/starts", PoseStampedArray, queue_size=5)
        self.planner_targets_pub = rospy.Publisher("/executive/targets", CommsNodeArray, queue_size=5)
        self.planner_compute_pub = rospy.Publisher("/planner/compute", PlannerType, queue_size=5)
        
        self.robot_info_sub = rospy.Subscriber("/sim/vehicle_poses", RobotArray, self.robots_info_callback)
        self.goal_reached_feedback = rospy.Subscriber("/sim/goals_reached", Bool, self.goal_reached_feedback)
        self.execution_feedback = rospy.Subscriber("/sim/executing_plans", Bool, self.execution_feedback)
        self.planner_feedback = rospy.Subscriber("/planner/paths", MultiRobotPlan, self.planner_feedback)
        self.full_target_list = rospy.Subscriber("/set_coverage/targets", CommsNodeArray, self.all_targets_callback)
        self.vehicle_in_collision_sub = rospy.Subscriber('/sim/collision_detected', Bool, self.collision_feedback)
        self.comms_node_sub =  rospy.Subscriber('/sim/comms_nodes', CommsNodeArray, self.comms_node_callback)

    def all_targets_callback(self, msg):
        self.all_targets = msg.nodes
        self.all_targets_inited = True

    def collision_feedback(self, msg):
        self.collision = msg.data

    def robots_info_callback(self, msg):
        self.robots_info = msg.robots
        self.got_start_states = True
    
    def goal_reached_feedback(self, msg):
        self.reached_goal = msg.data
        
    def planner_feedback(self, msg):
        self.plan_received = True
        
    def execution_feedback(self, msg):
        self.executing_plan = msg.data

    def comms_node_callback(self, msg):
        if(self.state == BehaviorStates.COLLECTING_NODE_INFO and self.state_counter > 5):
            new_targets = []
            
            for node1 in self.all_targets:
                found = False
                for node2 in msg.nodes:
                    if node1.x == node2.x and node1.y == node2.y:
                        found = True
                        break
                if not found:
                    new_targets.append(node1)
            
            self.all_targets = new_targets                
            self.updatedTargetList = True
        
    def update_viz(self):
        covered, remaining_covered = self.sim_interface.get_coverage()
        total_voxels = covered + remaining_covered
        percent_covered = covered / total_voxels
        percent_remaining = remaining_covered / total_voxels
        self._covered_pub.publish(Float32(percent_covered))
        self._remaining_covered_pub.publish(Float32(percent_remaining))
        
        self.viz_text_pub.publish(OverlayText(text=self.text))

    def get_manhattan_dist(self, x1, y1, x2, y2):
        return (abs(x2 - x1) + abs(y2 - y1))
    
    def compute_assignments(self):
        
        valid_robots_list = []
        for robot in self.robots_info:
            if robot.num_nodes_left > 0:
                valid_robots_list.append(robot)

        self.start_states.poses.clear()
        self.goal_locations.nodes.clear()
        
        if len(self.all_targets) == 0:
            return False
            
        if self.assignment_type == 1:
            
            self.updatedTargetList = False
            
            for idx, robot in enumerate(self.robots_info):
                
                if robot.num_nodes_left > 0:
                    r = PoseStamped()
                    r.pose.position.x = robot.pose.position.x
                    r.pose.position.y = robot.pose.position.y
                    r.pose.position.z = robot.pose.position.z
                    
                    self.start_states.poses.append(r)
                    if(idx < len(self.all_targets)):
                        n = CommsNodeMsg()
                        n.x = self.all_targets[idx].x
                        n.y = self.all_targets[idx].y
                        
                        self.goal_locations.nodes.append(n)
                    else:
                        n = CommsNodeMsg()
                        n.x = robot.pose.position.x
                        n.y = robot.pose.position.y
                        
                        self.goal_locations.nodes.append(n)
                else:
                    r = PoseStamped()
                    r.pose.position.x = robot.pose.position.x
                    r.pose.position.y = robot.pose.position.y
                    r.pose.position.z = robot.pose.position.z
                    
                    self.start_states.poses.append(r)
                    n = CommsNodeMsg()
                    n.x = robot.pose.position.x
                    n.y = robot.pose.position.y
                    
                    self.goal_locations.nodes.append(n)
            
            return True
        elif self.assignment_type == 2 or self.assignment_type == 3:
            
            self.updatedTargetList = False
            
            for node in self.all_targets:
                n = CommsNodeMsg()
                n.x = node.x
                n.y = node.y
                
                self.goal_locations.nodes.append(n)
                
            for idx, robot in enumerate(self.robots_info):
                
                if robot.num_nodes_left > 0:
                    r = PoseStamped()
                    r.pose.position.x = robot.pose.position.x
                    r.pose.position.y = robot.pose.position.y
                    r.pose.position.z = robot.pose.position.z
                    
                    self.start_states.poses.append(r)
                    if(idx < len(self.all_targets)):
                        # print("Here")
                        pass
                    else:
                        n = CommsNodeMsg()
                        # print("Here2")
                        n.x = robot.pose.position.x
                        n.y = robot.pose.position.y
                        
                        self.goal_locations.nodes.append(n)
                else:
                    r = PoseStamped()
                    r.pose.position.x = robot.pose.position.x
                    r.pose.position.y = robot.pose.position.y
                    r.pose.position.z = robot.pose.position.z
                    
                    self.start_states.poses.append(r)
                    n = CommsNodeMsg()
                    n.x = robot.pose.position.x
                    n.y = robot.pose.position.y
                    
                    self.goal_locations.nodes.append(n)
            
            
            return True
        # elif self.assignment_type == 3:
        #     print("Should not be here")
        #     self.updatedTargetList = False
        #     assigned_indices = []
        #     for idx, robot in enumerate(self.robots_info):
                
        #         if robot.num_nodes_left > 0:
                    
        #             r = PoseStamped()
        #             r.pose.position.x = robot.pose.position.x
        #             r.pose.position.y = robot.pose.position.y
        #             r.pose.position.z = robot.pose.position.z
                    
        #             self.start_states.poses.append(r)
                    
        #             if(idx < len(self.all_targets)):
        #                 best_dist = 1000
        #                 best_idx = -1
        #                 # print("R: {} {}".format(robot.pose.position.x, robot.pose.position.y))
        #                 for n_i,node in enumerate(self.all_targets):
        #                    dist = self.get_manhattan_dist(node.x, node.y, robot.pose.position.x, robot.pose.position.y) 
        #                    if ((dist <= best_dist) and (n_i not in assigned_indices)):
        #                     #    print("N : {} {}".format(node.x, node.y))
        #                        best_dist = dist
        #                        best_idx = n_i
                               
        #                 if best_idx == -1:
        #                     rospy.logerr('ERRORRR in assignments')
        #                     return False
                        
        #                 assigned_indices.append(best_idx)

        #                 n = CommsNodeMsg()
        #                 n.x = self.all_targets[best_idx].x
        #                 n.y = self.all_targets[best_idx].y
                        
        #                 # print("C : {} {}".format(n.x, n.y))
        #                 self.goal_locations.nodes.append(n)
        #             else:
        #                 n = CommsNodeMsg()
        #                 n.x = robot.pose.position.x
        #                 n.y = robot.pose.position.y
                        
        #                 self.goal_locations.nodes.append(n)
        #         else:
        #             r = PoseStamped()
        #             r.pose.position.x = robot.pose.position.x
        #             r.pose.position.y = robot.pose.position.y
        #             r.pose.position.z = robot.pose.position.z
                    
        #             self.start_states.poses.append(r)
        #             n = CommsNodeMsg()
        #             n.x = robot.pose.position.x
        #             n.y = robot.pose.position.y
                    
        #             self.goal_locations.nodes.append(n)
                    
        #    return True

    def trigger_planner(self):
        if not self.plan_sent:
            self.planner_starts_pub.publish(self.start_states)
            self.planner_targets_pub.publish(self.goal_locations)
            self.planner_compute_pub.publish(self.compute_plan_msg)
            self.plan_sent = True

    def run(self):

        if self.state == BehaviorStates.INIT:
            if (self.reached_goal) and (self.all_targets_inited) and (self.got_start_states):
                self.state = BehaviorStates.COMPUTE_ASSIGNMENTS
                self.text = "Received Inputs"
        elif self.state == BehaviorStates.COMPUTE_ASSIGNMENTS:
            time1 = time.time()
            ret = self.compute_assignments()
            duration = time.time() - time1
            if ret:
                self.state = BehaviorStates.TRIGGER_PLANNER
                self.text = "Computing Assignments"
                self.assignment_time_pub.publish(Float32(duration))
            else:
                self.state = BehaviorStates.ALL_NODES_PLACED
        elif self.state == BehaviorStates.TRIGGER_PLANNER:
            if not self.plan_sent:
                self.trigger_planner()
                self.plan_received = False
                self.plan_sent = True    
                self.text = "Triggering Planner"
            else:
                if self.executing_plan and self.plan_received:
                    self.state = BehaviorStates.WAIT_EXECUTION
                    self.text = "Executing Plans"
        elif self.state == BehaviorStates.WAIT_EXECUTION:
            if (self.plan_received) and (self.reached_goal) and (not self.executing_plan):
                self.state = BehaviorStates.COLLECTING_NODE_INFO 
                self.plan_sent = False
                self.text = "Goals Reached, Updating Node list"
        elif self.state == BehaviorStates.COLLECTING_NODE_INFO:
            self.state_counter += 1
            if (self.updatedTargetList):
                self.state = BehaviorStates.INIT
                self.updatedTargetList = False
                self.text = "Updated Node List"
                self.state_counter = 0
        elif self.state == BehaviorStates.ALL_NODES_PLACED:
            self.text = "All Nodes Placed Successfully"
            
        self.update_viz()