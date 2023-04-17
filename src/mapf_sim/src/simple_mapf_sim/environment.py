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
                 waypt_threshold=5, list_of_obstacle_dicts=[], nodes_per_robot=10):
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
        
        self.comms_nodes = [CommsNode(1, 10, 20)]
        self.nodes_per_robot = nodes_per_robot
        
        vehicle_positions = self.basestation.getInitialRobotPositions()
        
        self.vehicles = []
        for robot in range(self.vehicle_num):
            self.vehicles.append(self.init_vehicle(robot, vehicle_positions[robot]))
        
        self.global_waypt_list = [[] for i in range(vehicle_num)]

        self.waypt_threshold = waypt_threshold
        self.curr_waypt_num = 0

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


    def traverse(self):
        '''
        Waypoint manager and vehicle state update- moves vehicle towards waypoints as long as waypoints exist in global_waypt_list
        '''
        for veh_wps in self.global_waypt_list:
            # if not veh_wps or len(veh_wps.plan) == 0:
            #     pass
            # else:
            #     i = veh_wps.vehicle_id
            #     if (self.vehicle[i].battery == 0 or self.vehicle[i].in_flight_cond == False):
            #         next_position = np.array(
            #         [self.vehicle[i].x,
            #          self.vehicle[i].y,
            #          0])
            #     else:
            #         next_position = np.array(
            #         [veh_wps.plan[0].position.position.x,
            #          veh_wps.plan[0].position.position.y,
            #          veh_wps.plan[0].position.position.z])
            #     dist_to_waypt = np.linalg.norm(
            #         [self.vehicle[i].x, self.vehicle[i].y, self.vehicle[i].z] - next_position)

            #     # update waypoint list if reached waypoint
            #     if dist_to_waypt < self.waypt_threshold:
            #         # print("Reached waypoint -> ", next_position)
            #         self.curr_waypt_num += 1
            #         self.global_waypt_list[i].plan.pop(0)
            #     # else keep trying to navigate to next waypoint
                
            #     else:
            #         omega, z_d, dx, dy = self.vehicle[i].go_to_goal(self.max_omega, self.max_zvel,
            #                                             next_position, self.K_p,
            #                                             self.K_p_z)
            #         self.vehicle[i].x += self.del_t * dx
            #         self.vehicle[i].y += self.del_t * dy
            #         self.vehicle[i].z += self.del_t * z_d
            pass


    def update_waypts(self, new_wpts):
        '''
        Receive new waypoints and send them to waypoint manager
        '''
        # self.global_waypt_list.append(new_wpts)
        self.global_waypt_list[new_wpts.vehicle_id] = new_wpts
        self.curr_waypt_num = 0

    def update_states(self):
        '''
        Updates the environment states
        '''
        self.traverse()

