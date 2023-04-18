import math
import numpy as np
import rospy
from geometry_msgs.msg import Point
from simple_mapf_sim.msg import Waypoint

class Vehicle:
    '''
    Vehicle frame is ENU
    '''
    def __init__(self, id_num, init_x, init_y, init_z, comms_nodes_inventory, del_t):
        
        self.id_num = id_num
        self.x = init_x
        self.y = init_y
        self.z = init_z
        self.del_t = del_t

        self.num_nodes_left = int(comms_nodes_inventory)
        self.curr_t = 0
        self.reached_goal = True

        self.last_discrete_pos = Waypoint(x = self.x, y = self.y, z = self.z, t = 0) 
        self.plan = []

        self.recently_dropped_node = True
        
    def update_plan(self, new_plan):
        self.plan = new_plan.plan
        self.reached_goal = False
        self.curr_t = 0
        self.recently_dropped_node = False # Set Comms Node Dropped as False

        # print("P1: {} {} {} , P2: {} {} {}".format(self.plan[0].x , self.plan[0].y, self.plan[0].z, self.x, self.y, self.z))
        if(self.plan[0].x == self.x and self.plan[0].y == self.y and self.plan[0].z == self.z):
            self.last_discrete_pos = Waypoint(x = self.x, y = self.y, z = self.z, t = 0) 
            return True
        else:
            return False

    def setCommsNodeDropped(self, msg):
        self.recently_dropped_node = msg
        self.num_nodes_left -= 1
    
    def move_one_time_step(self):
        '''
        Returns angular velocity and velocity in z-axis towards desired direction
        '''
        if(len(self.plan) > 0):
            self.x = self.x + (self.plan[0].x - self.last_discrete_pos.x) * self.del_t
            self.y = self.y + (self.plan[0].y - self.last_discrete_pos.y) * self.del_t
            self.z = self.z + (self.plan[0].z - self.last_discrete_pos.z) * self.del_t

            self.curr_t += self.del_t

            if(self.curr_t >= 1):
                self.curr_t = 0
                self.plan.pop(0)
                self.last_discrete_pos = Waypoint(x = self.x, y = self.y, z = self.z, t = 0) 
                if len(self.plan) == 0:
                    self.reached_goal = True
        else:
            self.reached_goal = True    
            self.curr_t = 0


