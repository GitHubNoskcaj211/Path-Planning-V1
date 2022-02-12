import numpy as np
import math

# Holds x and y position. supports equality; set(x,y); set(point); distance(point)
class Point():
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __init__(self, point):
        self.x = point.x
        self.y = point.y

    def __eq__(self, point):
        return self.x == point.x and self.y == point.y

    def set(self, x, y):
        self.x = x
        self.y = y

    def set(self, point):
        self.x = point.x
        self.y = point.y

    def distance(self, point):
        return math.sqrt((self.x - point.x) ** 2 + (self.y - point.y) ** 2)

class Simulation():
    def __init__(self, p_start, p_goal, obstacles):
        # Parameters
        self.max_iterations = 1000 # max iterations (number of points) to attempt initial graph creation
        self.sensor_theta = 0.78 # the sensing arc angle in radians
        self.sensor_max_distance = 3 #the sensing distance

        self.p_goal = p_goal
        self.p_robot = p_start
        self.true_obstacles = obstacles # python list of Points
        # (x,y,data) first element is obstacle T/F, second is confidence (T-known or F-unknown). initializes to free and unknown.
        self.occupancy_grid = np.full((30,30,2), False) 
        # graph structure. List of nodes. A node has element 0 as its position and element 1 as its edges.
        # Edges are a tuple (node_index, distance)
        self.G = []
        self.path = None


    # Returns T/F whether or not an initial graph was found
    def graph_creation(self):
        self.G.insert(0, [Point(self.p_robot), []])

        # Initial graph formulation
        i = 0
        while path is None and i < self.max_iterations:
            path = [Point(2,5), Point(12,5)]

            i += 1
        
        # No path was found in our maximum allowed iterations
        if path is None:
            print('No path was found in', self.max_iterations, 'iterations.')
            return False

        return True

    def step(self):
        for i in range(self.path.shape[0]):
            p_robot = self.path[i]
            # Get distance of the robot

            # Update occupancy grid

            # Update weights in the graph

            # Generate refinement points into the graph

            # Get new path for the robot
            print(p_robot)


p_start = Point(5,2)
p_goal = Point(5,18)
obstacles = [Point(30,30), Point(15.8,19.3)] # list of points
sim = Simulation(p_start, p_goal, obstacles)

sim.graph_creation()

# while the sim has a path and it hasn't finished the path
while sim.path is not None and len(sim.path) != 0:
    sim.step()

    # if our path becomes invalid (obstacle blocked the path)
    if sim.path is None:
        # attempt to create a new graph
        sim.graph_creation()