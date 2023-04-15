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
        
        # # battery status
        # self.battery = battery
        # self.battery_time = rospy.Time.now()

        # # flight status
        # self.in_flight_cond = in_flight_cond

    def go_to_goal(self, next_waypt):
        '''
        Returns angular velocity and velocity in z-axis towards desired direction
        '''
        
        e = next_waypt - [self.x, self.y, self.z]  # dist to desired position
    
        return e

    # def update_battery(self, time):
    #     updated_battery = self.battery
    #     if self.in_flight_cond:
    #         battery_delta = -0.4
    #         elapsed_time = (time - self.battery_time).to_sec()

    #         updated_battery = self.battery + battery_delta * elapsed_time
    #         if updated_battery <= 0.0:
    #             updated_battery = 0.0
    #             print("battery is out of charge")
    #             self.in_flight_cond = False

    #         self.battery = updated_battery
    #         self.battery_time = time
    #     else:
    #         self.battery_time = rospy.Time.now()

    #     return updated_battery

    # def reduce_battery(self, percent, time):
    #     if self.battery > percent:
    #         self.battery = percent
    #         self.battery_time = time

