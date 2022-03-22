from math import sqrt
import math
import random
import matplotlib.pyplot as plt
import numpy as np
from quads import QuadTree, BoundingBox

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
        return sqrt((self.x) ** 2 + (self.y) ** 2)
    
    def get_angle_towards(self, point):
        return math.atan2(point.y - self.y, point.x - self.x)

    def distance(self, point):
        return sqrt((self.x - point.x) ** 2 + (self.y - point.y) ** 2)

class Node():
    def __init__(self, point, cost = None):
        self.pos = point
        self.cost = cost
        self.parent = None
        self.children = []
        self.nodes_index = -1
        
    # for the quadtree to store our structure
    def get_x(self):
        return self.pos.x
    
    def get_y(self):
        return self.pos.y
        
    def __str__(self):
        s = 'Node ' + str(self.nodes_index) + ': ' + str(self.pos) + (' | Parent: ' + self.parent.index if self.parent != None else ' ') + 'Children: '
        for n in self.children:
            s += ' ' + str(n.index)
            
        return s
        
    def distance(self, node2):
        return sqrt((self.pos.x - node2.pos.x) ** 2 + (self.pos.y - node2.pos.y) ** 2)

class Tree():
    def __init__(self, start, radius, world_bounds, occupancy_grid):
        self.debug = False
        
        self.world_x_min = world_bounds[0]
        self.world_x_max = world_bounds[2]
        self.world_y_min = world_bounds[1]
        self.world_y_max = world_bounds[3]
        
        self.nodes = []
        self.quad_tree = QuadTree(((self.world_x_max - self.world_x_min) / 2.0, (self.world_y_max - self.world_y_min) / 2.0), (self.world_x_max - self.world_x_min), (self.world_y_max - self.world_y_min))
        self.quad_tree_size = 0
        self.occupancy_grid = occupancy_grid
        
        self.radius = radius
        
        self.world_area = (self.world_x_max - self.world_x_min) * (self.world_y_max - self.world_y_min)
        
        self.goal_node = None
        
        self.add_node(Node(Point(start[0], start[1]), 0))
        
    def display(self, ax):
        if self.debug:
            x = []
            y = []
            for n in self.nodes:
                if n != self.goal_node:
                    x.append(n.pos.x)
                    y.append(n.pos.y)
            
            ax.scatter(x,y, c='g')
            
            for n in self.nodes:
                for child in n.children:
                    #plt.arrow(x=n.pos.x, y=n.pos.y, dx=(child.pos.x - n.pos.x), dy=(child.pos.y - n.pos.y), width=0.05) 
                    ax.plot([n.pos.x, child.pos.x], [n.pos.y, child.pos.y], linewidth=0.5, color='b')
                    
        if self.goal_node != None:
            ax.scatter(self.goal_node.pos.x,self.goal_node.pos.y, marker="P", c='purple')
        
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
        
        # connect the nearest node to the new node
        best_parent = nearest_nodes[0][0]
        min_cost = best_parent.cost + nearest_nodes[0][1]
        for next_nearest_node in nearest_nodes:
            if next_nearest_node[1] > self.radius:
                break
            
            n = next_nearest_node[0]
            if n.cost + next_nearest_node[1] < min_cost:
                best_parent = n
                min_cost = best_parent.cost + next_nearest_node[1]
        self.add_connection(best_parent, new_node)
        
        # go through all the nearest nodes and check re-routing through new_node
        for next_nearest_node in nearest_nodes:
            if next_nearest_node[1] > self.radius:
                break
            
            n = next_nearest_node[0]
            if new_node.cost + next_nearest_node[1] < n.cost:
                self.remove_connection(n.parent, n)
                self.add_connection(new_node, n)
                
        return True
                    
    # adds node2 as a child of node1 and adds node2 as a parent of node1
    def add_connection(self, node1, node2):
        if self.debug:
            node1.children.append(node2)
        node2.parent = node1
        node2.cost = node1.cost + sqrt((node1.pos.x - node2.pos.x) ** 2 + (node1.pos.y - node2.pos.y) ** 2)
        
    # removes node2 from the children of node1 and removes node1 as a parent of node2
    def remove_connection(self, node1, node2):
        if self.debug:
            for i in range(len(node1.children)):
                if node1.children[i] == node2:
                    node1.children.pop(i)
                    break
            
        node2.parent = None
            
    # adds a node to our node list and sets its index
    def add_node(self, node):
        node.nodes_index = len(self.nodes)
        self.nodes.append(node)
        self.quad_tree.insert((node.pos.x, node.pos.y), data=node)
        self.quad_tree_size += 1
    
    def get_nearest_nodes(self, node):
        nearest_nodes = []
        # look within our box first
        for n in self.get_neighbors_in_box(node.pos.x, node.pos.y):
            n = n.data
            d = sqrt((node.pos.x - n.pos.x) ** 2 + (node.pos.y - n.pos.y) ** 2)
            if d <= self.radius:
                if self.clear_path(n, node): # move this check up
                    nearest_nodes.append([n, d])
            
        if len(nearest_nodes) == 0:
            # if there was nothing in our box look for nearest neighbors
            for n in self.get_nearest_neighbors(node.pos.x, node.pos.y):
                n = n.data
                d = sqrt((node.pos.x - n.pos.x) ** 2 + (node.pos.y - n.pos.y) ** 2)
                if d <= self.radius or len(nearest_nodes) == 0:
                    if self.clear_path(n, node): # move this check up
                        nearest_nodes.append([n, d])
                elif len(nearest_nodes) > 0:
                    break
        
        nearest_nodes = sorted(nearest_nodes, key=lambda x: x[1])
        return nearest_nodes
    
    # generates a node with a random position in our world
    def generate_random_node(self):
        x_pos = random.random() * (self.world_x_max - self.world_x_min) + self.world_x_min
        y_pos = random.random() * (self.world_y_max - self.world_y_min) + self.world_y_min
        
        return Node(Point(x_pos, y_pos))
    
    # returns true if a node is in an obstacle
    def node_in_obstacle(self, node):
        if self.occupancy_grid is None:
            return False
        
        y = int(node.pos.y)
        x = int(node.pos.x)
        
        return self.occupancy_grid.query_obstacle(x,y)
    
    # returns true if there is a clear straight line path (not intersecting any obstacles)
    def clear_path(self, node1, node2):
        if self.occupancy_grid is None:
            return True
        
        return self.occupancy_grid.query_free(node1.pos.x, node1.pos.y, node2.pos.x, node2.pos.y)
    
    # returns the generator object for the whole quad tree based all points in the bounding box
    def get_neighbors_in_box(self, x, y):        
        return self.quad_tree.within_bb(BoundingBox(min_x=x - self.radius, min_y=y - self.radius, max_x=x + self.radius, max_y=y + self.radius))
    
    # returns the generator object for the quad tree based on distance
    def get_nearest_neighbors(self, x, y):        
        return self.quad_tree.nearest_neighbors((x, y), count=2)
    # math.ceil(self.quad_tree_size * (self.radius ** 2 * math.pi / self.world_area))
    
    def clear_tree(self, start):
        self.nodes = []
        self.quad_tree = QuadTree(((self.world_x_max - self.world_x_min) / 2.0, (self.world_y_max - self.world_y_min) / 2.0), (self.world_x_max - self.world_x_min), (self.world_y_max - self.world_y_min))
        self.quad_tree_size = 0
        
        self.add_node(Node(Point(start[0], start[1]), 0))