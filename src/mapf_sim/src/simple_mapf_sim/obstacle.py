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
        speed,
        hold_heading_time,
        dt=0.02,
    ):
        self.id = id
        self.x = init_x
        self.y = init_y
        self.speed = speed
        self.width = width
        self.length = length
        self.height = height

        self.speed_heading = None
        self.hold_heading_time = hold_heading_time
        self.last_heading_update = hold_heading_time * int(1 / dt)
        self.dt = dt

        # coordinates of bottom left and top right points
        self.bl = (self.x - self.length / 2.0, self.y - self.width / 2.0)
        self.tr = (self.x + self.length / 2.0, self.y + self.width / 2.0)

    def update(self):
        # update position in a random heading, updated every second
        self.last_heading_update = self.last_heading_update - 1
        if self.speed_heading is None or self.last_heading_update == 0:
            self.last_heading_update = self.hold_heading_time * int(1 / self.dt)
            self.speed_heading = float(random.randrange(0, 360) / 180.0 * math.pi)

        vel_x = math.cos(self.speed_heading) * self.speed
        vel_y = math.sin(self.speed_heading) * self.speed

        self.x = self.x + vel_x * self.dt
        self.y = self.y + vel_y * self.dt

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
        return "id: {}, x: {},  y: {},  width: {}, length: {}, height: {}, speed: {}, hold_heading_time: {}".format(
            self.id,
            self.x,
            self.y,
            self.width,
            self.length,
            self.height,
            self.speed,
            self.hold_heading_time,
        )


def run_test():
    #               id   x    y    w    l    h    speed heading time
    obst = Obstacle(123, 0.0, 0.0, 2.0, 2.0, 4.0, 0.0, 0.0)

    # right in it
    print("Should be True: {}".format(obst.is_in_collision(1.0, 1.0, 1.0)))
    # definitely not in collision
    print("Should be False: {}".format(obst.is_in_collision(10.0, 10.0, 3.0)))

    # on edge in z
    print("Should be True: {}".format(obst.is_in_collision(1.0, 1.0, 4.9)))
    # on edge in z, but not in collision
    print("Should be False: {}".format(obst.is_in_collision(1.0, 1.0, 5.0)))

    # on edge in x, front
    print("Should be True: {}".format(obst.is_in_collision(1.9, 1.0, 1.0)))
    # on edge in x, front, but not in collision
    print("Should be False: {}".format(obst.is_in_collision(2.0, 1.0, 1.0)))

    # on edge in x, rear
    print("Should be True: {}".format(obst.is_in_collision(-1.9, 1.0, 1.0)))
    # on edge in x, rear, but not in collision
    print("Should be False: {}".format(obst.is_in_collision(-2.0, 1.0, 1.0)))

    # on edge in y, right
    print("Should be True: {}".format(obst.is_in_collision(1.0, 1.99, 1.0)))
    # on edge in y, right, but not in collision
    print("Should be False: {}".format(obst.is_in_collision(1.0, 2.0, 1.0)))

    # on edge in y, left
    print("Should be True: {}".format(obst.is_in_collision(1.0, -1.99, 1.0)))
    # on edge in y, left, but not in collision
    print("Should be False: {}".format(obst.is_in_collision(1.0, -2.0, 1.0)))


if __name__ == "__main__":
    run_test()
