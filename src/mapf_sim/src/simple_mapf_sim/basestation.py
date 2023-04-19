import math
import numpy as np
import rospy
from geometry_msgs.msg import Point



class Basestation:
    def __init__(self, id, x, y, z, num_robots):
        self.id = id
        self.x = x 
        self.y = y
        self.z = z
        
        self.bst_indices = [(self.x, self.y, 0), (self.x - 1, self.y - 1, 0), (self.x - 1, self.y , 0), (self.x, self.y - 1, 0)]
        self.num_robots = num_robots
        if self.num_robots <= 12:
            self.index_offset = 2
        elif self.num_robots <= 32:
            self.index_offset = 3
        elif self.index_offset <= 60:
            self.index_offset = 4
        else:
            self.index_offset = 5
    
    def getInitialRobotPositions(self):
        pos_list = []
        start_x = self.x - self.index_offset
        end_x = self.x + self.index_offset
        start_y = self.y - self.index_offset
        end_y = self.y + self.index_offset
        
        print("{} {} {} {} ".format(start_x, end_x, start_y, end_y))
        print(self.bst_indices)
        direction_x = 1 if end_x > start_x else -1
        direction_y = 1 if end_y > start_y else -1
        robots = self.num_robots
        for i in range(start_x, end_x, direction_x):
            for j in range(start_y, end_y, direction_y):
                if(robots > 0):
                    cur_pos =  (i, j, 0)
                    if (cur_pos not in pos_list) and (cur_pos not in self.bst_indices):
                        pos_list.append(cur_pos)
                        robots = robots - 1                        
        
        if len(pos_list) != self.num_robots:
            print("Error getting initial positions")
        return pos_list
            
    def __str__(self):
        return "Basestation at: {}, x: {},  y: {} has {} robots in authority".format(self.id, self.x, self.y, self.num_robots) 