import math
import numpy as np
import rospy
from geometry_msgs.msg import Point



class Basestation:
    def __init__(self, id, x, y, num_robots):
        self.id = id
        self.x = x 
        self.y = y
        self.z = 0
                
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
        end_y = self.x + self.index_offset
        
        robots = self.num_robots
        for i in range(start_x, end_x):
            for j in range(start_y, end_y):
                if(robots > 0):
                    cur_pos =  (i, j, 0)
                    if (cur_pos not in pos_list) and (cur_pos not in self.bst_indices):
                        pos_list.append(cur_pos)
                        robots = robots - 1
                        
        return pos_list
            
    def __str__(self):
        return "Basestation at: {}, x: {},  y: {} has {} robots in authority".format(self.id, self.x, self.y, self.num_robots) 