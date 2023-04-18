import math
import random
from turtle import heading
import numpy as np
from simple_mapf_sim.vehicle import *
from simple_mapf_sim.obstacle import *
from simple_mapf_sim.basestation import *
from simple_mapf_sim.comms_node import *
from geographic_msgs.msg import GeoPose


class Environment:
    def __init__(self, init_x=None, init_y=None, init_z=None,
                 vehicle_num=3, del_t=1,
                list_of_obstacle_dicts=[], nodes_per_robot=10):
        '''
        Setup simulation environment
        '''
        # Basestation pose
        self.init_x = init_x
        self.init_y = init_y
        self.init_z = init_z

        self.vehicle_num = vehicle_num
        self.del_t = del_t
        self.current_timestep = 0
        
        self.basestation = Basestation(self.init_x, self.init_y, self.init_z, self.vehicle_num)
        
        self.obstacles = self.generate_obstacles(list_of_obstacle_dicts)
        
        self.comms_nodes = []
        self.nodes_per_robot = nodes_per_robot
        
        vehicle_positions = self.basestation.getInitialRobotPositions()
        
        self.vehicles = []
        for robot in range(self.vehicle_num):
            self.vehicles.append(self.init_vehicle(robot, vehicle_positions[robot]))


    def is_in_collision(self):
        for i in range(self.vehicle_num):
            for obst in self.obstacles:
                if obst.is_in_collision(self.vehicles[i].x, self.vehicles[i].y, self.vehicles[i].z):
                    return True
        return False

    def generate_obstacles(self, obsts):
        obstacles = [
            Obstacle(
                id=obst["id"],
                init_x=obst["x"], init_y=obst["y"],
                width=obst["width"],
                length=obst["length"],
                height=obst["height"]
            ) for obst in obsts
        ]
        return obstacles

    def init_vehicle(self, id_num, position):
        return Vehicle(id_num = id_num, init_x = position[0], init_y = position[1], init_z = position[2], comms_nodes_inventory = self.nodes_per_robot, del_t=self.del_t)

    def update_waypts(self, new_wpts):
        '''
        Receive new waypoints and send them to waypoint manager
        '''

        for idx,robot in enumerate(self.vehicles):
            ret = robot.update_plan(new_wpts.plans[idx])
            if not ret:
                rospy.logwarn("Incoming Plan not starting from current pos... ignoring")
                return False
        
        return True

    def update_states(self):
        '''
        Updates the environment states
        '''
        for robot in self.vehicles:
            robot.move_one_time_step()

    def drop_comms_nodes(self, x, y):
        self.comms_nodes.append(CommsNode(len(self.comms_nodes) + 1, x, y))