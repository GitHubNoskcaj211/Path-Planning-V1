from simulation import Simulation

start = (0,2,3.14)
goal = (30,30)
# xmin, ymin, xmax, ymax
world_bounds = (0,0,30,30)
sim = Simulation(start, goal, world_bounds)

sim.generate_obstacles(5)
print(sim.obstacles)

sim.build_initial_rrt()
for i in sim.path:
    print(i)
sim.display()

while not sim.step():
    sim.display()
    
sim.display()

input('Press enter to close the program.')