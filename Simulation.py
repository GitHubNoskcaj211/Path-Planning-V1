import numpy as np
from graph import Graph, Edge, Node, Point
import time

class Simulation():
    def __init__(self, p_start, p_goal, obstacles):
        # Parameters
        self.max_iterations = 30 # max iterations (number of points) to attempt initial graph creation
        self.sensor_theta = 0.78 # the sensing arc angle in radians
        self.sensor_max_distance = 3 #the sensing distance

        self.p_goal = p_goal
        self.p_robot = p_start
        self.true_obstacles = obstacles # python list of Points
        # (x,y,data) first element is obstacle T/F, second is confidence (T-known or F-unknown). initializes to free and unknown.
        self.occupancy_grid = np.full((30,30,2), False) 
        self.graph = Graph()
        
        self.goal_node = Node(Point(self.p_goal.x, self.p_goal.y))
        self.graph.add_node(self.goal_node)
        
        self.path = None

    # Returns T/F whether or not an initial graph was found
    def graph_creation(self):
        self.start_node = Node(Point(self.p_robot.x, self.p_robot.y))
        self.graph.add_node(self.start_node)
        
        self.path = self.graph.find_shortest_path(self.start_node, self.goal_node)
        
        # Initial graph formulation
        i = 0
        while self.path is None and i < self.max_iterations:
            # create an exploratory point
            node = self.graph.generate_exploratory_node(self.start_node, self.goal_node)
            
            # add the node to the graph
            self.graph.add_node(node)
            
            # check if a path reaches goal
            self.path = self.graph.find_shortest_path(self.start_node, self.goal_node)
            sim.graph.display()
            i += 1
        
        # No path was found in our maximum allowed iterations
        if self.path is None:
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


p_start = Point(5.0,2.0,1.56)
p_goal = Point(12.0,18.0)
obstacles = [Point(30,30), Point(15.8,19.3)] # list of points
sim = Simulation(p_start, p_goal, obstacles)

sim.graph_creation()
print('Num Points:', len(sim.graph.graph))
sim.graph.display()


# while the sim has a path and it hasn't finished the path
# while sim.path is not None and len(sim.path.path) != 0:
#     sim.step()
#     time.sleep(1)

#     # if our path becomes invalid (obstacle blocked the path)
#     if sim.path is None:
#         # attempt to create a new graph
#         sim.graph_creation()