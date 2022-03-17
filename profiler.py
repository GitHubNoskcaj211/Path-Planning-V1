import cProfile
import math
import random
from tree import Tree
from occupancygrid import OccupancyGrid

start = (2.5,2.5,0)
goal = (25.5,28.5)
# xmin, ymin, xmax, ymax
world_bounds = (0,0,32,32)

occupancy = OccupancyGrid(world_bounds, 6, math.pi / 4)
for i in range(20):
    x_pos = math.floor(random.random() * 32)
    y_pos = math.floor(random.random() * 32)
    occupancy.grid[y_pos, x_pos, 0] = True

rrtree = Tree((start[0], start[1]), 3, world_bounds, occupancy)
# rrtree.build_tree(4000)

cProfile.run('rrtree.build_tree(1000)')


# new_node = rrtree.generate_random_node()
# while rrtree.node_in_obstacle(new_node):
#     new_node = rrtree.generate_random_node()
# cProfile.run('rrtree.add_node_to_tree(new_node)')