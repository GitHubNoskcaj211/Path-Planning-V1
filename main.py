from simulation import Simulation

start = (2.5,2.5,0)
goal = (25.5,28.5)
# xmin, ymin, xmax, ymax
world_bounds = (0,0,30,30)
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