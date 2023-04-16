import math
import numpy as np
import rospy
from geometry_msgs.msg import Point

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

        self.num_nodes_left = comms_nodes_inventory
        
    def go_to_goal(self, next_waypt):
        '''
        Returns angular velocity and velocity in z-axis towards desired direction
        '''
        
        e = next_waypt - [self.x, self.y, self.z]  # dist to desired position
    
        return e


