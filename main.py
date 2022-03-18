from Simulation import Simulation
import random
import math

# xmin, ymin, xmax, ymax
world_bounds = (0,0,32,32)
start = (random.random() * world_bounds[2], random.random() * world_bounds[3], random.random() * math.pi * 2 - math.pi)
goal = (random.random() * world_bounds[2], random.random() * world_bounds[3])
sim = Simulation(start, goal, world_bounds)

sim.generate_obstacles(5)

sim.build_initial_rrt()
for i in sim.path:
    print(i)
sim.display()

while not sim.step():
    sim.display()
    
sim.display()

input('Press enter to close the program.')