import rospy
import math
import numpy as np


class CommsNode:
    def __init__(self, id, x, y):
        self.id = id
        self.x = x 
        self.y = y
        
        self.bst_snr = 90
    
    def __str__(self):
        return "id: {}, x: {},  y: {}".format(self.id, self.x, self.y) 