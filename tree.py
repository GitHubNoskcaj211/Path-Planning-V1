import math
import random
import matplotlib.pyplot as plt

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
        index = -1
        
    def __str__(self):
        s = 'Node ' + str(self.index) + ': ' + str(self.pos) + (' | Parent: ' + self.parent.index if self.parent != None else ' ') + 'Children: '
        for n in self.children:
            s += ' ' + str(n.index)
            
        return s
        
    def distance(self, node2):
        return self.pos.distance(node2.pos)

class Tree():
    def __init__(self, start, radius, world_bounds):
        self.nodes = []
        self.occupancy_grid = None
        
        self.radius = radius
        
        self.world_x_min = world_bounds[0]
        self.world_x_max = world_bounds[2]
        self.world_y_min = world_bounds[1]
        self.world_y_max = world_bounds[3]
        
        self.goal_node = None
        
        self.add_node(Node(Point(start[0], start[1]), 0))
        
    def display(self, ax):
        x = []
        y = []
        for n in self.nodes:
            if n != self.goal_node:
                x.append(n.pos.x)
                y.append(n.pos.y)
        
        # grid = sns.JointGrid(df['x'], df['y'], space=0, height=8, ratio=100)
        # grid.plot_joint(plt.scatter, color="g")
        ax.scatter(x,y, c='g')
        if self.goal_node != None:
            ax.scatter(self.goal_node.pos.x,self.goal_node.pos.y, marker="P", c='purple')
        for n in self.nodes:
            for child in n.children:
                #plt.arrow(x=n.pos.x, y=n.pos.y, dx=(child.pos.x - n.pos.x), dy=(child.pos.y - n.pos.y), width=0.05) 
                ax.plot([n.pos.x, child.pos.x], [n.pos.y, child.pos.y], linewidth=0.25, color='b')
        
    def get_path_to_goal(self):
        if self.goal_node == None:
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
    def build_tree(self, occupancy_grid, iterations):
        self.occupancy_grid = occupancy_grid
        
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
        nearest_node = self.nodes[nearest_nodes[0][0]]
        self.add_connection(nearest_node, new_node)
        
        # go through all the nearest nodes and check for a better parent
        for next_nearest_node in nearest_nodes:
            if next_nearest_node[1] > self.radius:
                break
            
            n = self.nodes[next_nearest_node[0]]
            if n.cost + new_node.distance(n) < new_node.cost:
                self.remove_connection(new_node.parent, new_node)
                self.add_connection(n, new_node)
        
        # go through all the nearest nodes and check re-routing through new_node
        for next_nearest_node in nearest_nodes:
            if next_nearest_node[1] > self.radius:
                break
            
            n = self.nodes[next_nearest_node[0]]
            if new_node.cost + new_node.distance(n) < n.cost:
                self.remove_connection(n.parent, n)
                self.add_connection(new_node, n)
                
        return True
                    
    # adds node2 as a child of node1 and adds node2 as a parent of node1
    def add_connection(self, node1, node2):
        node1.children.append(node2)
        node2.parent = node1
        node2.cost = node1.cost + node1.distance(node2)
        
    # removes node2 from the children of node1 and removes node1 as a parent of node2
    def remove_connection(self, node1, node2):
        for i in range(len(node1.children)):
            if node1.children[i] == node2:
                node1.children.pop(i)
                break
            
        node2.parent = None
            
    # adds a node to our node list and sets its index
    def add_node(self, node):
        node.index = len(self.nodes)
        self.nodes.append(node)
    
    def get_nearest_nodes(self, node):
        node_distance_list = []
        
        for n in self.nodes:
            if self.clear_path(n, node):
                node_distance_list.append([n.index, n.distance(node)])
            
        # sort from shortest to longest distance
        node_distance_list = sorted(node_distance_list, key=lambda x: x[1])
        
        return node_distance_list
    
    # generates a node with a random position in our world
    def generate_random_node(self):
        x_pos = random.random() * (self.world_x_max - self.world_x_min) + self.world_x_min
        y_pos = random.random() * (self.world_y_max - self.world_y_min) + self.world_y_min
        
        return Node(Point(x_pos, y_pos))
    
    # returns true if a node is in an obstacle
    def node_in_obstacle(self, node):
        return False
        # return self.occupancy_grid.query(node.pos.x, node.pos.y)
    
    def clear_path(self, node1, node2):
        # TODO
        return True
    
    def clear_tree(self):
        self.nodes = []