from tree import Tree
from occupancygrid import OccupancyGrid
from Simulation import Simulation, TreePoint
import matplotlib.pyplot as plt
import numpy as np
import math
import random
import time
from robot import Robot
import matplotlib
import time

start = (0,0,0)
# xmin, ymin, xmax, ymax
world_bounds = (0,0,30,30)
robot = Robot(start)
occupancy = OccupancyGrid(world_bounds, robot.sensing_radius, robot.sensing_angle)

for i in range(30):
    # set some obstacles
    occupancy.grid[int(random.random() * 30), int(random.random() * 30), 0] = True
    
rrtree = Tree((robot.position.x, robot.position.y), 3, world_bounds, occupancy)

fig, ax = plt.subplots()
fig.set_figwidth(8)
fig.set_figheight(8)
fig.show()

while True:
    t = time.time()
    rrtree.build_tree(10)
    print(time.time() - t)
    ax.cla()
    occupancy.display(ax,TreePoint(0,0),0)
    rrtree.display(ax)

    fig.canvas.draw()
    fig.canvas.flush_events()