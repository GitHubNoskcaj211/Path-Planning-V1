import math
import random
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, LineString
import numpy as np
from rtree import index

class Point():        
    def __init__(self, x, y, theta = 0):
        self.x = x
        self.y = y
        self.theta = theta

    def __eq__(self, point):
        return self.x == point.x and self.y == point.y and self.theta == point.theta
    
    def __str__(self):
        return str(self.x) + ' ' + str(self.y) + ' ' + str(self.theta)
    
    def add(self, point):
        self.x += point.x
        self.y += point.y
        self.theta += point.theta
        
    def mag(self):
        return math.sqrt((self.x) ** 2 + (self.y) ** 2)
    
    def get_angle_towards(self, point):
        return math.atan2(point.y - self.y, point.x - self.x)

    def distance(self, point):
        return math.sqrt((self.x - point.x) ** 2 + (self.y - point.y) ** 2)

class Node():
    def __init__(self, point, cost = None):
        self.pos = point
        self.cost = cost
        self.parent = None
        self.children = []
        self.rtree_index = -1
        self.nodes_index = -1
        
    def __str__(self):
        s = 'Node ' + str(self.index) + ': ' + str(self.pos) + ' | Cost: ' + str(self.cost) +(' | Parent: ' + str(self.parent.index) if self.parent != None else ' ') + 'Children: '
        # for n in self.children:
        #     s += ' ' + str(n.index)
            
        return s
        
    def distance(self, node2):
        return self.pos.distance(node2.pos)

class Tree():
    def __init__(self, start, radius, world_bounds, occupancy_grid):
        self.occupancy = occupancy_grid
        self.nodes = []
        self.rtree = index.Index()
        self.rtree_size = 0
        self.num_obstacle_squares = 0
        self.fill_rtree_with_occupancy(occupancy_grid)
        
        self.radius = radius
        
        self.world_x_min = world_bounds[0]
        self.world_x_max = world_bounds[2]
        self.world_y_min = world_bounds[1]
        self.world_y_max = world_bounds[3]
        
        self.world_area = (self.world_x_max - self.world_x_min) * (self.world_y_max - self.world_y_min)
        
        self.random_point_x_min = max(self.world_x_min, start[0] - self.radius)
        self.random_point_x_max = min(self.world_x_max, start[0] + self.radius)
        self.random_point_y_min = max(self.world_y_min, start[1] - self.radius)
        self.random_point_y_max = min(self.world_y_max, start[1] + self.radius)
        
        self.goal_node = None
        
        self.add_node(Node(Point(start[0], start[1]), 0))
        
    def fill_rtree_with_occupancy(self, occupancy_grid):
        for i in occupancy_grid.get_all_obstacle_polygons():
            x, y = i.exterior.coords.xy
            self.rtree.insert(self.rtree_size, (min(x), min(y), max(x), max(y)), i)
            self.rtree_size += 1
            
        self.num_obstacle_squares = self.rtree_size
            
    def display(self, ax):
        x = []
        y = []
        # for n in self.nodes:
        #     if n != self.goal_node:
        #         x.append(n.pos.x)
        #         y.append(n.pos.y)
        
        # grid = sns.JointGrid(df['x'], df['y'], space=0, height=8, ratio=100)
        # grid.plot_joint(plt.scatter, color="g")
        ax.scatter(x,y, c='g')
        if self.goal_node != None:
            ax.scatter(self.goal_node.pos.x,self.goal_node.pos.y, marker="P", c='purple')
        # for n in self.nodes:
        #     for child in n.children:
        #         #plt.arrow(x=n.pos.x, y=n.pos.y, dx=(child.pos.x - n.pos.x), dy=(child.pos.y - n.pos.y), width=0.05) 
        #         ax.plot([n.pos.x, child.pos.x], [n.pos.y, child.pos.y], linewidth=0.25, color='b')
        
    def get_path_to_goal(self):
        if self.goal_node is None:
            return None
        else:
            path_list = []
            n = self.goal_node
            while n.parent != None:
                path_list.insert(0, n.pos)
                n = n.parent
                
            return path_list
        
    def add_goal_node(self, goal_x, goal_y):
        goal_node = Node(Point(goal_x, goal_y))
        if self.add_node_to_tree(goal_node):
            self.goal_node = goal_node
            return True
        else:
            return False
        
    # build the rrt tree
    def build_tree(self, iterations):
        for i in range(iterations):
            new_node = self.generate_random_node()
            if self.node_in_obstacle(new_node):
                continue
            
            self.add_node_to_tree(new_node)
            
    # adds the new_node to the tree with the re-routing. Returns true if it was added and false if it was not.
    def add_node_to_tree(self, new_node):
        # get the nearest nodes that are not blocked by an obstacle to the new_node
        nearest_nodes = self.get_nearest_nodes(new_node)
        
        if len(nearest_nodes) == 0:
            # no near nodes with a clear path
            return False
        
        # add a node to our node list
        self.add_node(new_node)
        
        self.random_point_x_min = max(self.world_x_min, min(self.random_point_x_min, new_node.pos.x - self.radius))
        self.random_point_x_max = min(self.world_x_max, max(self.random_point_x_max, new_node.pos.x + self.radius))
        self.random_point_y_min = max(self.world_y_min, min(self.random_point_y_min, new_node.pos.y - self.radius))
        self.random_point_y_max = min(self.world_y_max, max(self.random_point_y_max, new_node.pos.y + self.radius))
        
        # go through all the nearest nodes and check for a better parent
        best_parent = nearest_nodes[0]
        min_cost = best_parent.cost + new_node.distance(best_parent)
        for next_nearest_node in nearest_nodes:
            if next_nearest_node.distance(new_node) > self.radius:
                break
            
            if next_nearest_node.cost + new_node.distance(next_nearest_node) < min_cost:
                best_parent = next_nearest_node
                min_cost = best_parent.cost + new_node.distance(best_parent)
        self.add_connection(best_parent, new_node)
        
        # go through all the nearest nodes and check re-routing through new_node
        for next_nearest_node in nearest_nodes:
            if next_nearest_node.distance(new_node) > self.radius:
                break
            
            if new_node.cost + new_node.distance(next_nearest_node) < next_nearest_node.cost:
                self.remove_connection(next_nearest_node.parent, next_nearest_node)
                self.add_connection(new_node, next_nearest_node)
                
        return True
                    
    # adds node2 as a child of node1 and adds node2 as a parent of node1
    def add_connection(self, node1, node2):
        # node1.children.append(node2)
        node2.parent = node1
        node2.cost = node1.cost + node1.distance(node2)
        
        
    # removes node2 from the children of node1 and removes node1 as a parent of node2
    def remove_connection(self, node1, node2):
        # for i in range(len(node1.children)):
        #     if node1.children[i] == node2:
        #         node1.children.pop(i)
        #         break
            
        node2.parent = None
            
    # adds a node to our rtree and sets its index
    def add_node(self, node):
        node.rtree_index = self.rtree_size
        node.nodes_index = len(self.nodes)
        self.rtree.insert(node.rtree_index, (node.pos.x, node.pos.y, node.pos.x, node.pos.y), node.nodes_index)
        self.rtree_size += 1
        self.nodes.append(node)
    
    def get_nearest_nodes(self, node):
        obstacles = [] # list of polygons representing the squares, only added once discovered as relavant
        nearest_nodes = []
        
        # returns true if there is a clear straight line path (not intersecting any obstacles)
        def clear_path(node1, node2):
            path = LineString([(node1.pos.x, node1.pos.y), (node2.pos.x, node2.pos.y)])
            for obstacle in obstacles:
                if path.intersects(obstacle):
                    return False
            return True
        
        for i in self.get_nearest_rtree_nodes(node.pos.x, node.pos.y):
            i = i.object
            if type(i) == int:
                node2 = self.nodes[i]
                if not clear_path(node, node2):
                    continue
                else: #we are within radius distance and we have a clear path, add it to a list
                    nearest_nodes.append(node2)
            elif type(i) == Polygon:
                obstacles.append(i)
            else:
                raise Exception('Something went wrong with the rtree')
            
        return nearest_nodes
    
    # generates a node with a random position in our world
    def generate_random_node(self):
        x_pos = random.random() * (self.world_x_max - self.world_x_min) + self.world_x_min
        y_pos = random.random() * (self.world_y_max - self.world_y_min) + self.world_y_min
        
        return Node(Point(x_pos, y_pos))
    
    # returns true if a node is in an obstacle
    def node_in_obstacle(self, node):
        # for i in self.rtree.intersection((node.pos.x, node.pos.y, node.pos.x, node.pos.y), objects=True):
        #     if type(i) == Polygon:
        #         return True
            
        # return False
        
        return self.occupancy.query_obstacle(node.pos.x,node.pos.y)
    
    # returns the generator object for the whole rtree based on distance from the point given
    def get_nearest_rtree_nodes(self, x, y):        
        # return sorted(list(self.rtree.intersection((x - self.radius,y - self.radius,x + self.radius,y + self.radius), objects=True)), key=lambda x: min)
        return self.rtree.nearest((x,y,x,y), num_results = math.ceil(self.rtree_size * (self.radius ** 2 * math.pi / self.world_area)) + self.num_obstacle_squares, objects=True)
    
    def clear_tree(self):
        self.rtree = index.Index()
        self.rtree_size = 0
        self.num_obstacle_squares = 0