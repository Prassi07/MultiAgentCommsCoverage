#!/usr/bin/env python3
import random
import math


class Obstacle(object):
    """
    Obstacles are defined as rectangular regions with a center
    located on the ground and a width, length, and height.
    """

    def __init__(
        self,
        id,
        init_x,
        init_y,
        width,
        length,
        height,
        dt=0.02,
    ):
        self.id = id
        self.x = init_x
        self.y = init_y
        self.width = width
        self.length = length
        self.height = height

        self.dt = dt

        # coordinates of bottom left and top right points
        self.bl = (self.x - self.length / 2.0, self.y - self.width / 2.0)
        self.tr = (self.x + self.length / 2.0, self.y + self.width / 2.0)

    def is_in_collision(self, x, y, z, size=1.0):
        """
        Is the point provided inside the obstacle
        treats object as a cube with side length `size`
        """
        bl_vehicle = (x - size, y - size)
        tr_vehicle = (x + size, y + size)

        no_collision = (
            (bl_vehicle[0] >= self.tr[0])
            or (tr_vehicle[0] <= self.bl[0])
            or (tr_vehicle[1] <= self.bl[1])
            or (bl_vehicle[1] >= self.tr[1])
        )

        if not no_collision:
            # if below height of obstacle, plus buffer
            return z < (self.height + size)

        return False

    def __str__(self):
        return "id: {}, x: {},  y: {},  width: {}, length: {}, height: {}".format(
            self.id,
            self.x,
            self.y,
            self.width,
            self.length,
            self.height,
        )

