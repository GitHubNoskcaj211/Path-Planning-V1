from simulation import Simulation

start = (0,2,3.14)
goal = (10,10)
# xmin, ymin, xmax, ymax
world_bounds = (0,0,10,10)
sim = Simulation(start, goal, world_bounds)
sim.build_initial_rrt()
for i in sim.path:
    print(i)
sim.display()

while not sim.step():
    sim.display()
    
sim.display()

input('Press enter to close the program.')