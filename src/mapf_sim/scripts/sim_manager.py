#!/usr/bin/env python

import os
import rospy
import numpy as np
import time


from rospkg import RosPack
from simple_mapf_sim.msg import Plan, Waypoint, ObstaclePose, ObstacleArray, PoseStampedArray, OdometryArray
from simple_mapf_sim.msg import CommsNodeMsg, CommsNodeArray, MultiRobotPlan, RobotInfo, RobotArray
from simple_mapf_sim.environment import *
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion, PoseArray
from std_msgs.msg import ColorRGBA, Bool
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import UInt8
from tf.transformations import quaternion_from_euler
from math import ceil, floor
import copy
from visualization_msgs.msg import Marker, MarkerArray

package = RosPack()
package_path = package.get_path("simple_mapf_sim")

ROBOT_COLORS = [[0.9, 0.6, 0.1], [1, 0, 0], [0, 0, 1], [1, 1, 0],
                [0, 1, 1]]

def get_obstacle_color():
    ros_color = ColorRGBA()
    ros_color.r = 0.15
    ros_color.g = 0.15
    ros_color.b = 0.15
    ros_color.a = 0.8
    return  ros_color

def get_color_robot(index):
    total_colors = len(ROBOT_COLORS) 
    ros_color = ColorRGBA()
    ros_color.r = ROBOT_COLORS[int(index % total_colors)][0]
    ros_color.g = ROBOT_COLORS[int(index % total_colors)][1]
    ros_color.b = ROBOT_COLORS[int(index % total_colors)][2]
    ros_color.a = 1.0
    return  ros_color

class SimManager:
    def __init__(self):

        self.sim_env = self.env_setup()
        self.vehicle_traj_list = [[] for v in range(self.sim_env.vehicle_num)]
        
        self.map_size = rospy.get_param("/map_size") # meters
        self.map_resolution = rospy.get_param("/map_resolution") # meters
        self.coverage_size = rospy.get_param("/coverage_size") # meters
        self.covered_value = 25.0
        self.map_voxels = int(self.map_size / self.map_resolution)
        self.coverage_map = np.ones(self.map_voxels**2, dtype=int)

        self.new_plan_in = False
        self.execution_in_progress = False

    def env_setup(self):

        # obstacles
        obst_list = rospy.get_param("obstacles", [])

        # basestation state
        init_x = rospy.get_param("/basestation_x")
        init_y = rospy.get_param("/basestation_y")
        init_z = rospy.get_param("/basestation_z")

        vehicle_num = rospy.get_param("/vehicle_num")
        del_t = rospy.get_param("/del_t")

        nodes_per_robot = rospy.get_param("/nodes_per_robot")
        return Environment( init_x,
                            init_y,
                            init_z,
                            vehicle_num,
                            del_t,
                            obst_list, 
                            nodes_per_robot)

    def get_vehicle_position(self, time, frame):
        v_poses = RobotArray()
        v_poses.header.frame_id = frame
        v_poses.header.stamp = time

        for id_num in range(self.sim_env.vehicle_num):
            vehicle_pose = RobotInfo()
            vehicle_pose.pose.position.x = self.sim_env.vehicles[id_num].x
            vehicle_pose.pose.position.y = self.sim_env.vehicles[id_num].y
            vehicle_pose.pose.position.z = self.sim_env.vehicles[id_num].z

            quat = quaternion_from_euler(0, 0, 0)
            vehicle_pose.pose.orientation.x = quat[0]
            vehicle_pose.pose.orientation.y = quat[1]
            vehicle_pose.pose.orientation.z = quat[2]
            vehicle_pose.pose.orientation.w = quat[3]
            
            vehicle_pose.num_nodes_left = self.sim_env.vehicles[id_num].num_nodes_left

            v_poses.robots.append(vehicle_pose)

        return v_poses
    
    def get_comms_positions(self, time, frame):
        comms_poses = CommsNodeArray()
        comms_poses.header.frame_id = frame
        comms_poses.header.stamp = time
        
        # comms_poses.nodes.append(CommsNodeMsg(id = 0, x = self.sim_env.basestation.x, y = self.sim_env.basestation.y))
        for i in range(len(self.sim_env.comms_nodes)):
            comms = CommsNodeMsg()
            comms.x = self.sim_env.comms_nodes[i].x
            comms.y = self.sim_env.comms_nodes[i].y
            comms.id = self.sim_env.comms_nodes[i].id
            
            comms_poses.nodes.append(comms)
            
        return comms_poses
    
    def get_coverage_grid(self, time, frame):
        
        new_coverage_map = copy.deepcopy(self.coverage_map)
        # Basestation Coverage
        bst = self.sim_env.basestation
        start_y = int((bst.y - (self.coverage_size / 2) + (self.map_size / 2)) / self.map_resolution)
        end_y = int((bst.y + (self.coverage_size / 2) + (self.map_size / 2)) / self.map_resolution)
        i = start_y
        while i < end_y:
            start_x = int((bst.x - (self.coverage_size / 2) + (self.map_size / 2)) / self.map_resolution)
            end_x = int((bst.x + (self.coverage_size / 2) + (self.map_size / 2)) / self.map_resolution)
            start = i * self.map_voxels + start_x
            end = i * self.map_voxels + end_x
            new_coverage_map[start:end][new_coverage_map[start:end] <= self.covered_value] = self.covered_value
            i = i + 1 
            
        for id_num in range(len(self.sim_env.comms_nodes)):
            x = self.sim_env.comms_nodes[id_num].x
            y = self.sim_env.comms_nodes[id_num].y
            start_y = int((y - self.coverage_size / 2. + self.map_size / 2)  / self.map_resolution)
            end_y = int((y + self.coverage_size / 2. + self.map_size / 2)  / self.map_resolution)
            i = start_y
            while i < end_y:
                start_x = int((x - self.coverage_size / 2. + self.map_size / 2) / self.map_resolution)
                end_x = int((x + self.coverage_size / 2. + self.map_size / 2) / self.map_resolution)

                start = i * self.map_voxels + start_x
                end = i * self.map_voxels + end_x
                new_coverage_map[start:end][new_coverage_map[start:end] <= self.covered_value] = self.covered_value
                i = i + 1

        grid = OccupancyGrid()
        grid.header.frame_id = frame
        grid.header.stamp = time
        grid.info.height = self.map_voxels
        grid.info.width = self.map_voxels
        grid.info.resolution = self.map_resolution
        grid.info.origin.position.x = -self.map_size / 2
        grid.info.origin.position.y = -self.map_size / 2
        grid.data = new_coverage_map.tolist()

        return grid

    def get_obstacle_positions(self, time, frame):
        obstacles = ObstacleArray()
        obstacles.header.frame_id = frame
        obstacles.header.stamp = time

        for obst in self.sim_env.obstacles:
            obst_pose = ObstaclePose()
            obst_pose.id = obst.id
            obst_pose.x = obst.x
            obst_pose.y = obst.y
            obst_pose.width = obst.width
            obst_pose.length = obst.length
            obst_pose.height = obst.height
            obstacles.obstacles.append(obst_pose)

        return obstacles

    def get_occupancy_grid(self, time, frame):
        
        # Obstacles in the occupancy grid
        for obst in self.sim_env.obstacles:
            start_y = int((obst.y - (obst.width / 2) + (self.map_size / 2))  / self.map_resolution)
            end_y = int((obst.y + (obst.width / 2) + (self.map_size / 2))  / self.map_resolution)
            i = start_y
            while i < end_y:
                start_x = int((obst.x - (obst.length / 2) + (self.map_size / 2)) / self.map_resolution)
                end_x = int((obst.x + (obst.length / 2) + (self.map_size / 2)) / self.map_resolution)
                start = i * self.map_voxels + start_x
                end = i * self.map_voxels + end_x
                self.coverage_map[start:end] = 100
                i = i + 1

        # Basestation in the occupancy grid
        bst = self.sim_env.basestation
        start_y = int((bst.y - 1 + (self.map_size / 2)) / self.map_resolution)
        end_y = int((bst.y + 1 + (self.map_size / 2)) / self.map_resolution)
        i = start_y
        while i < end_y:
            start_x = int((bst.x - 1 + (self.map_size / 2)) / self.map_resolution)
            end_x = int((bst.x + 1 + (self.map_size / 2)) / self.map_resolution)
            start = i * self.map_voxels + start_x
            end = i * self.map_voxels + end_x
            self.coverage_map[start:end] = 100
            i = i + 1 
            
        grid = OccupancyGrid()
        grid.header.frame_id = frame
        grid.header.stamp = time
        grid.info.height = self.map_voxels
        grid.info.width = self.map_voxels
        grid.info.resolution = self.map_resolution
        grid.info.origin.position.x = -self.map_size / 2
        grid.info.origin.position.y = -self.map_size / 2
        grid.data = self.coverage_map.tolist()

        return grid

    def planner_callback(self, msg):
        if not self.execution_in_progress:
            ret = self.sim_env.update_waypts(msg)
            if ret:
                self.new_plan_in = True

    def get_basestation_marker(self, time, frame):
        basest_marker = MarkerArray()
        
        bst = self.sim_env.basestation
        bst_marker = Marker()
        bst_marker.header.frame_id = frame
        bst_marker.header.stamp = time
        bst_marker.ns = "basestation"
        bst_marker.id = bst.id
        bst_marker.type = Marker.CUBE
        bst_marker.action = Marker.ADD
        bst_marker.lifetime = rospy.Duration()
        bst_marker.color.r = 0
        bst_marker.color.g = 1
        bst_marker.color.b = 0
        bst_marker.color.a = 1
        bst_marker.scale.x = 2
        bst_marker.scale.y = 2
        bst_marker.scale.z = 2

        bst_marker.pose.position.x = bst.x
        bst_marker.pose.position.y = bst.y
        bst_marker.pose.position.z = bst.z

        bst_marker.pose.orientation.x = 0
        bst_marker.pose.orientation.y = 0
        bst_marker.pose.orientation.z = 0
        bst_marker.pose.orientation.w = 1
        basest_marker.markers.append(bst_marker)
        
        return basest_marker
        
    def get_vehicle_marker(self, time, frame):
        veh_markers = MarkerArray()

        for veh in self.sim_env.vehicles:
            vehicle_marker = Marker()
            vehicle_marker.header.frame_id = frame
            vehicle_marker.header.stamp = time
            vehicle_marker.ns = "vehicle_pose"
            vehicle_marker.id = veh.id_num
            vehicle_marker.type = Marker.CYLINDER
            vehicle_marker.action = Marker.ADD
            vehicle_marker.lifetime = rospy.Duration()
            vehicle_marker.color = get_color_robot(veh.id_num)
            vehicle_marker.scale.x = 0.9
            vehicle_marker.scale.y = 0.9
            vehicle_marker.scale.z = 1


            vehicle_marker.pose.position.x = veh.x + 0.5
            vehicle_marker.pose.position.y = veh.y + 0.5
            vehicle_marker.pose.position.z = veh.z + 0.5

            vehicle_marker.pose.orientation.x = 0
            vehicle_marker.pose.orientation.y = 0
            vehicle_marker.pose.orientation.z = 0
            vehicle_marker.pose.orientation.w = 1

            veh_markers.markers.append(vehicle_marker)

        return veh_markers

    def get_vehicle_trajectory_marker(self, time, frame):
        markers = MarkerArray()

        for veh in self.sim_env.vehicles:
            trajectory_marker = Marker()
            trajectory_marker.header.frame_id = frame
            trajectory_marker.header.stamp = time
            trajectory_marker.ns = "vehicle_trajectory"
            trajectory_marker.id = veh.id_num
            trajectory_marker.type = Marker.LINE_STRIP
            trajectory_marker.action = Marker.ADD
            trajectory_marker.lifetime = rospy.Duration()


            trajectory_marker.pose.position.x = 0
            trajectory_marker.pose.position.y = 0
            trajectory_marker.pose.position.z = 0

            trajectory_marker.pose.orientation.x = 0
            trajectory_marker.pose.orientation.y = 0
            trajectory_marker.pose.orientation.z = 0
            trajectory_marker.pose.orientation.w = 1

            for p in veh.plan:
                trajectory_marker.points.append(Point(p.x + 0.5, p.y + 0.5, 0.5))

            trajectory_marker.color = get_color_robot(veh.id_num)
            
            trajectory_marker.scale.x = 0.2
            trajectory_marker.scale.y = 0.2
            trajectory_marker.scale.z = 0.2

            markers.markers.append(trajectory_marker)

        return markers
    
    def get_obstacle_markers(self, time, frame):
        obstacles_marker_array = MarkerArray()

        for idx, obstacle in enumerate(self.sim_env.obstacles):
            obstacle_marker = Marker()
            obstacle_marker.header.frame_id = frame
            obstacle_marker.header.stamp = time
            obstacle_marker.ns = "obstacle_pose"
            obstacle_marker.id = idx
            obstacle_marker.type = Marker.CUBE
            obstacle_marker.action = Marker.ADD
            obstacle_marker.lifetime = rospy.Duration()
            obstacle_marker.pose = Pose(Point(obstacle.marker_x,
                                              obstacle.marker_y,
                                              2.5),
                                              Quaternion(0, 0, 0, 1))

            obstacle_marker.color = get_obstacle_color()
            obstacle_marker.scale.x = obstacle.length
            obstacle_marker.scale.y = obstacle.width
            obstacle_marker.scale.z = 5
            obstacles_marker_array.markers.append(obstacle_marker)

        return obstacles_marker_array
    
    def get_comms_nodes_markers(self, time, frame):
        comms_marker_array = MarkerArray()

        for idx, node in enumerate(self.sim_env.comms_nodes):
            comms_marker = Marker()
            comms_marker.header.frame_id = frame
            comms_marker.header.stamp = time
            comms_marker.ns = "comms_node"
            comms_marker.id = node.id
            comms_marker.type = Marker.ARROW
            comms_marker.action = Marker.ADD
            comms_marker.lifetime = rospy.Duration()
            
            quat = quaternion_from_euler(0, -1.57, 0)
            comms_marker.pose.position.x = node.x
            comms_marker.pose.position.y = node.y
            comms_marker.pose.position.z = 0.1
            comms_marker.pose.orientation.x = quat[0]
            comms_marker.pose.orientation.y = quat[1]
            comms_marker.pose.orientation.z = quat[2]
            comms_marker.pose.orientation.w = quat[3]

            comms_marker.color = ColorRGBA(1, 0, 1, 1)
            comms_marker.scale.x = 2
            comms_marker.scale.y = 0.25
            comms_marker.scale.z = 0.25
            comms_marker_array.markers.append(comms_marker)

        return comms_marker_array
        
    def check_all_goals_reached(self):
        reached = True
        for idx, robot in enumerate(self.sim_env.vehicles):
            if (robot.reached_goal) and (not robot.recently_dropped_node) and (robot.num_nodes_left > 0):
                self.sim_env.drop_comms_nodes(robot.x, robot.y)
                self.sim_env.vehicles[idx].setCommsNodeDropped(True)
            reached = reached & robot.reached_goal

        return reached

    def main(self):
        
        vehicle_pose_pub = rospy.Publisher('/sim/vehicle_poses', RobotArray, queue_size=10)
        obstacle_pose_pub = rospy.Publisher('/sim/obstacles', ObstacleArray, queue_size=10)
        comms_pose_pub = rospy.Publisher('/sim/comms_nodes', CommsNodeArray, queue_size=10)
        
        occ_grid_pub = rospy.Publisher('/sim/occupancy_grid', OccupancyGrid, queue_size=10, latch=True)
        coverage_grid_pub = rospy.Publisher('/sim/coverage_grid', OccupancyGrid, queue_size=10)
        
        vehicle_in_collision_pub = rospy.Publisher('/sim/collision_detected', Bool, queue_size=10)

        # Marker Publishers
        vehicle_marker_pub = rospy.Publisher('/sim/markers/vehicle_pose', MarkerArray, queue_size=10)
        comms_marker_pub = rospy.Publisher('/sim/markers/comms_marker', MarkerArray, queue_size=10)
        basestation_marker_pub = rospy.Publisher('/sim/markers/basestation', MarkerArray, queue_size=10, latch=True)
        vehicle_trajectory_pub = rospy.Publisher('/sim/markers/vehicle_trajectory', MarkerArray, queue_size=10)
        obstacle_marker_pub = rospy.Publisher('/sim/markers/obstacles', MarkerArray, queue_size=10, latch=True)

        waypt_sub = rospy.Subscriber('/planner/paths', MultiRobotPlan, self.planner_callback)
        goals_reached_pub = rospy.Publisher('/sim/goals_reached', Bool, queue_size = 1)
        plan_execution_pub = rospy.Publisher('/sim/executing_plans', Bool, queue_size = 1)
        rate = rospy.Rate(1/0.25)
        counter = 0

        collision_detected = False
        published_grid = False

        while not rospy.is_shutdown():
            time = rospy.Time.now()
            frame = "local_enu"
            
            
            vehicle_pose_pub.publish(self.get_vehicle_position(time, frame))
            comms_pose_pub.publish(self.get_comms_positions(time, frame))
            obstacle_pose_pub.publish(self.get_obstacle_positions(time, frame))
            
            if not published_grid:
                occ_grid_pub.publish(self.get_occupancy_grid(time, frame))
                basestation_marker_pub.publish(self.get_basestation_marker(time, frame))
                obstacle_marker_pub.publish(self.get_obstacle_markers(time, frame))
                published_grid = True

            vehicle_marker_pub.publish(self.get_vehicle_marker(time, frame))
            comms_marker_pub.publish(self.get_comms_nodes_markers(time, frame))
            
            if counter % 10 == 0:
                vehicle_trajectory_pub.publish(self.get_vehicle_trajectory_marker(time, frame))
                coverage_grid_pub.publish(self.get_coverage_grid(time, frame))

            counter += 1

            if self.new_plan_in:
                self.sim_env.update_states()
                self.new_plan_in = False
                self.execution_in_progress = True
            else:
                if(self.check_all_goals_reached()):
                    goals_reached_pub.publish(Bool(True))
                    self.execution_in_progress = False
                else:
                    goals_reached_pub.publish(Bool(False))
                    self.sim_env.update_states()
                    
            plan_execution_pub.publish(Bool(self.execution_in_progress))
             
            if self.sim_env.is_in_collision():
                collision_detected = True

            msg_collision = Bool()
            msg_collision.data = collision_detected
            vehicle_in_collision_pub.publish(msg_collision)

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("sim_manager_node", anonymous=True)
    obj = SimManager()
    obj.main()


