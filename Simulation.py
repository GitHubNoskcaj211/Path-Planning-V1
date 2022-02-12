import numpy as np
import math

# Holds x, y, and theta position. supports equality; set(x,y); set(point); distance(point)
class Point():
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.theta = 0

    def __init__(self, point):
        self.x = point.x
        self.y = point.y
        self.theta = point.theta

    def __eq__(self, point):
        return self.x == point.x and self.y == point.y and self.theta == point.theta

    def set(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def set(self, x, y):
        self.x = x
        self.y = y

    def set(self, point):
        self.x = point.x
        self.y = point.y
        self.theta = point.theta

    def distance(self, point):
        return math.sqrt((self.x - point.x) ** 2 + (self.y - point.y) ** 2)

class Graph():
    def __init__(self):
        self.graph = []

    def add_node(self, node):
        self.graph.append(node)

    def add_node(self, node, index):
        self.graph.insert(index, node)

class Node():
    def __init__(self, point, edges):
        self.pos = point
        self.edges = edges

    def __init__(self, point):
        self.pos = point
        self.edges = []

    def add_edge(self, edge):
        self.edges.append(edge)

class Edge():
    def __init__(self, node_1_index, node_2_index, distance, active):
        self.node_1_index = node_1_index
        self.node_2_index = node_2_index
        self.weight = distance
        self.active = active

    def set(self, node_1_index, node_2_index, distance, active):
        self.node_1_index = node_1_index
        self.node_2_index = node_2_index
        self.weight = distance
        self.active = active


class Simulation():
    def __init__(self, p_start, p_goal, obstacles):
        # Parameters
        self.max_iterations = 1000 # max iterations (number of points) to attempt initial graph creation
        self.sensor_theta = 0.78 # the sensing arc angle in radians
        self.sensor_max_distance = 3 #the sensing distance
        self.goal_radius = 1
        self.connection_radius = 2

        self.p_goal = p_goal
        self.p_robot = p_start
        self.true_obstacles = obstacles # python list of Points
        # (x,y,data) first element is obstacle T/F, second is confidence (T-known or F-unknown). initializes to free and unknown.
        self.occupancy_grid = np.full((30,30,2), False) 
        self.graph = Graph()
        self.path = None


    # Returns T/F whether or not an initial graph was found
    def graph_creation(self):
        self.graph.add_node(Node(self.p_robot), 0)
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
        # move the robot forward 1 step and remove it from the path
        self.p_robot.set(self.path.pop(0))
        print(self.p_robot)
        # Get distance of the sensor

        # Update occupancy grid

        # Update edges in the graph

        # Generate refinement points into the graph

        # Get new path for the robot


p_start = Point(5,2,1.56)
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