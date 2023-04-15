import math
import random
from turtle import heading
import numpy as np
from simple_mapf_sim.vehicle import *
from simple_mapf_sim.target import *
from simple_mapf_sim.obstacle import *
from geographic_msgs.msg import GeoPose


class Environment:
    def __init__(self, init_x=None, init_y=None, init_z=None,
                 vehicle_num=3, n_rand_obst=-1, del_t=1,
                 waypt_threshold=5, list_of_obstacle_dicts=[]):
        '''
        Setup simulation environment
        '''
        # if initial position not specified, randomly spawn vehicle between (50, 1000)
        init_x = random.randrange(50, 1000) if init_x is None else init_x
        init_y = random.randrange(50, 1000) if init_y is None else init_y
        init_z = random.randrange(20, 120,
                                  20) if init_z is None else init_z  # discretized by step-size 20
        # drone pose
        self.init_x = init_x
        self.init_y = init_y
        self.init_z = init_z

        self.vehicle_num = vehicle_num
        self.del_t = del_t

        # if targets not specified, randomly generate between 1-10 targets
        self.n_rand_obst = random.randrange(1, 10) if not list_of_obstacle_dicts and n_rand_obst == -1 else n_rand_obst

        self.obstacles = self.generate_obstacles(list_of_obstacle_dicts, self.n_rand_obst)

        self.global_waypt_list = [[] for i in range(vehicle_num)]

        self.waypt_threshold = waypt_threshold
        self.vehicle = [self.init_vehicle(i) for i in range(self.vehicle_num)]


        self.curr_waypt_num = 0

    def is_in_collision(self):
        for i in range(self.vehicle_num):
            for obst in self.obstacles:
                if obst.is_in_collision(self.vehicle[i].x, self.vehicle[i].y, self.vehicle[i].z):
                    return True
        return False

    def generate_obstacles(self, obsts, num_obst):
        if obsts is None or len(obsts) == 0:
            obstacles = []
            idx = num_obst
            while idx > 0:
                obst = Obstacle(
                    id=idx,
                    init_x=np.random.uniform(-220, 220),
                    init_y=np.random.uniform(-220, 220),
                    width=5.0,
                    length=5.0,
                    height=100.0,
                    speed=0.0,
                    hold_heading_time=1.
                )

                if not obst.is_in_collision(self.init_x, self.init_y, 40.0):
                    obstacles.append(obst)
                    idx = idx - 1
            for o in obstacles:
                print(str(o))
            return obstacles
        else:
            obstacles = [
                Obstacle(
                    id=obst["id"],
                    init_x=obst["x"], init_y=obst["y"],
                    width=obst["width"],
                    length=obst["length"],
                    height=obst["height"],
                    speed=obst["speed"],
                    hold_heading_time=obst["hold_heading_time"]
                ) for obst in obsts
            ]
            return obstacles

    def init_vehicle(self, id_num):
        return Vehicle(id_num=id_num,
                       init_x=self.init_x,
                       init_y=self.init_y+(id_num*200),
                       init_z=self.init_z + (id_num*20),
                       comms_nodes_inventory = 10,
                       del_t=self.del_t)


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

        # update the states for all ships in the environment
        for obstacle in self.obstacles:
            obstacle.update()

    def get_vehicle_uncertainty(self, id_num):
        return self.vehicle[id_num].position_uncertainty()

    # function that gets target heading and return heading with gaussian noise
    def get_target_heading_noise(self, heading):
        # gaussian noise model for now
        return heading + np.random.normal(0, 0.05)
