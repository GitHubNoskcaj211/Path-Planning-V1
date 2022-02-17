from cmath import inf
import math
import random
from re import X
from tracemalloc import start
import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd

class Vector():
    def __init__(self,x,y):
        self.x = x
        self.y = y
    
    def angle_between_vectors(self, vector):
        return math.acos(self.dot(vector) / (self.magnitude() * vector.magnitude()))
        
    def dot(self, vector):
        return self.x * vector.x + self.y * vector.y    
    
    def magnitude(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)        

# Holds x, y, and theta position. supports equality; set(x,y); set(point); distance(point)
class Point():        
    def __init__(self, x, y, theta = 0):
        self.x = x
        self.y = y
        self.theta = theta

    def __eq__(self, point):
        return self.x == point.x and self.y == point.y and self.theta == point.theta
    
    def __str__(self):
        return str(self.x) + ' ' + str(self.y) + ' ' + str(self.theta)

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

class Path():
    def __init__(self):
        self.path = []
        
    def append_node(self, node):
        self.path.append(node)
        
    def prepend_node(self, node):
        self.path.insert(0, node)
        
    def insert_node(self, node, index):
        self.path.insert(index, node)
        
    def __str__(self):
        s = ''
        for node in self.path:
            s += str(node) + '\n'
        return s

class Graph():
    def __init__(self):
        self.goal_radius = 1
        self.connection_radius = 2
        self.graph = []

    def add_node(self, node):
        self.graph.append(node)
        node.index = len(self.graph) - 1
        
        for n in self.graph:
            if node.distance(n) < self.connection_radius and node != n:
                # connect with an edge
                node.add_edge(n)
    
    def get_node(self, index):
        return self.graph[index]
    
    def display(self):
        l = [[n.pos.x, n.pos.y] for n in self.graph]
        df = pd.DataFrame(l, columns=['x', 'y'])
        
        grid = sns.JointGrid(df['x'], df['y'], space=0, height=3, ratio=50)
        grid.plot_joint(plt.scatter, color="g")
        for n in self.graph:
            for e in n.edges:
                n2 = self.graph[e.node_2_index]
                plt.plot([n.pos.x, n2.pos.x], [n.pos.y, n2.pos.y], linewidth=0.1, color='b')
                
        plt.show()
        
    def max_edge_angle(self, node):
        if len(node.edges) == 0:
            return math.pi
        
        max_angle_between_vectors = 0
        for e in node.edges:
            n2 = self.graph[e.node_2_index]
            v1 = Vector(n2.pos.x - node.pos.x, n2.pos.y - node.pos.y)
            closest_angle_between_vectors = math.pi
            for e2 in node.edges:
                if not e == e2: # if the edges are distinct
                    n3 = self.graph[e2.node_2_index]
                    v2 = Vector(n3.pos.x - node.pos.x, n3.pos.y - node.pos.y)
                    new_angle = v1.angle_between_vectors(v2)
                    if new_angle < closest_angle_between_vectors:
                        closest_angle_between_vectors = new_angle
            if closest_angle_between_vectors > max_angle_between_vectors:
                max_angle_between_vectors = closest_angle_between_vectors
                
        return max_angle_between_vectors
            
    
    def generate_exploratory_node(self, start_node, goal_node):
        # consider idea of how many bits do you need to know where you are in the environemnt based on 
        # where the points are (aka, if you only have 2 points, you wouldnt know where you were in all the empty areas because
        # you have nothing to go off of)
        node_list, visited = self.dijkstras(start_node, goal_node, False)
        
        # get the indexes of the leafs of the tree
        leaf_list = []
        for i in range(len(node_list)):
            if not visited[i]:
                continue
            
            is_parent = False
            for n in node_list:
                if n[1] == i: # n has i as a parent
                    is_parent = True
                    break
            if not is_parent:
                leaf_list.append(i)
        print(leaf_list)
        # how much do we care about information (prop to distance to goal). How much information do we have
        # how much information we have TODO idea -> inactive edges are high information, active edges are lower
        # max angle between all the edges connecting in OR avereage of angle between edges connecting
        # big angle and close distance means select
        scores = [[leaf, 1 / (self.graph[leaf].distance(goal_node) + 1) * self.max_edge_angle(self.graph[leaf])] for leaf in leaf_list]
        scores = sorted(scores, key=lambda x: x[1])
        node = self.graph[scores[0][0]]
        print(node)
        # order nodes by distance
        #distances = [[node.index, 1 / (len(node.edges) + 1) + 1 / (node.distance(goal_node) + 1)] for node in self.graph]
        #   distances = [[node.index, (len(node.edges) + 1) * (node.distance(goal_node) + 1)] for node in self.graph]
        #   distances[goal_node.index][1] = inf
        #   distances = sorted(distances, key=lambda x: x[1]) # sort in reverse order
        # weighted choice with node
        #weights = [x[1] for x in distances]
        #   node = self.graph[distances[0][0]]#self.graph[random.choices(distances, weights=tuple(weights), k=1)[0][0]]#
        
        x,y = self.new_point_direction(node, goal_node)
        mag = math.sqrt(x ** 2 + y ** 2)
        if mag > self.connection_radius:
            x /= mag
            x *= random.random() * self.connection_radius * 0.99
            y /= mag
            y *= random.random() * self.connection_radius * 0.99
        return Node(Point(node.pos.x + x, node.pos.y + y, node.pos.theta + random.random() / 2 - 0.25))
      
    # gross clean up later  
    def new_point_direction(self, node, goal_node):
        x = 0#random.random() * 100 - 50
        y = 0#random.random() * 100 - 50
        # force away from other points
        for n in self.graph:
            x_dir = node.pos.x - n.pos.x
            y_dir = node.pos.y - n.pos.y
            mag = math.sqrt(x_dir ** 2 + y_dir ** 2)
            if mag == 0:
                continue
            
            x_dir /= mag
            y_dir /= mag
            
            force = 1 / node.distance(n) ** 2
            
            x += force * x_dir
            y += force * y_dir
        
        # force towards goal
        x_dir = goal_node.pos.x - node.pos.x
        y_dir = goal_node.pos.y - node.pos.y
        
        x_dir += random.random() * x_dir / 4 - x_dir / 8
        y_dir += random.random() * y_dir / 4 - y_dir / 8
        
        mag = math.sqrt(x_dir ** 2 + y_dir ** 2)
        if mag != 0:
            x_dir /= mag
            y_dir /= mag
            
            force = node.distance(goal_node) ** 2
            
            x += force * x_dir
            y += force * y_dir
            
        mag = math.sqrt(x ** 2 + y ** 2)
        if mag == 0:
            x += 1
        
        return x,y
        
    # gross clean up later
    # if a goal node is denoted, it will run the modified a_star encoding
    def dijkstras(self, starting_node, goal_node, a_star):
        starting_node_index = starting_node.index
        num_nodes = len(self.graph)
        node_list = [[None, None, self.graph[i].distance(goal_node)] for i in range(num_nodes)] #current weight number, and what parent it has in the tree, distance from goal. First two initialize to None
        node_list[starting_node_index][0] = 0
        visited = [False] * num_nodes
        
        # if we are in a_star mode, continue until all possible nodes are searched. Otherwise, end when
        # goal is reached
        while not visited[goal_node.index] or a_star:           
            # get the smallest node to connect in
            smallest_pos = -1
            for i in range(len(node_list)):
                if not visited[i] and node_list[i][0] != None and (smallest_pos == -1 or node_list[i][0] + (node_list[i][2] if a_star else 0) < node_list[smallest_pos][0] + (node_list[smallest_pos][2] if a_star else 0)):
                    smallest_pos = i
            
            # check if we didn't find anything (visited all nodes possible)
            if smallest_pos == -1:
                break
            
            # at this point we need to connect in smallest_pos
            for edge in self.graph[smallest_pos].edges:
                if edge.active and not visited[edge.node_2_index]:
                    new_weight = node_list[smallest_pos][0] + edge.weight 
                    old_weight = node_list[edge.node_2_index][0]
                    if old_weight == None or new_weight < old_weight:
                        node_list[edge.node_2_index][0] = new_weight
                        node_list[edge.node_2_index][1] = edge.node_1_index
                
            visited[smallest_pos] = True
        
        return node_list, visited
        
    # given a starting node and goal node of the graph, return a Path (if one exists)
    def find_shortest_path(self, starting_node, goal_node):
        node_list, visited = self.dijkstras(starting_node, goal_node, False)
        goal_node_index = goal_node.index
        
        if not visited[goal_node_index]:
            return None
        
        path = Path()
        n_index = goal_node_index
        while n_index != None:
            path.prepend_node(self.graph[n_index])
            n_index = node_list[n_index][1]
            
        return path    
    
    def __str__(self):
        s = ''
        for n in self.graph:
            s += str(n) + '\n'
        
        return s
            
class Node():
    def __init__(self, point):
        self.pos = point
        self.edges = []
        index = -1
        
    def __str__(self):
        s = 'Node ' + str(self.index) + ': ' + str(self.pos) + ' | Edges: '
        for e in self.edges:
            s += ' ' + str(e)
            
        return s
        
    def distance(self, node2):
        return self.pos.distance(node2.pos)

    def add_edge(self, node2): # TODO check for obstacle
        self.edges.append(Edge(self.index, node2.index, self.distance(node2), True))
        node2.edges.append(Edge(node2.index, self.index, self.distance(node2), True))

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
        
    def __str__(self):
        return '(' + str(self.node_2_index) + ',' + str(self.weight) + ',' + str(self.active) + ')'
    
    def __eq__(self, edge):
        return self.node_1_index == edge.node_1_index and self.node_2_index == edge.node_2_index
        
        
# testing path creation
# g = Graph()
# g.add_node(Node(Point(0,0)))
# g.add_node(Node(Point(0,1)))
# g.add_node(Node(Point(0,2)))
# g.add_node(Node(Point(1,0)))
# g.add_node(Node(Point(1,1)))
# g.add_node(Node(Point(1,2)))
# g.add_node(Node(Point(2,0)))
# g.add_node(Node(Point(2,1)))
# g.add_node(Node(Point(2,2)))
# g.get_node(0).add_edge(g.get_node(1))
# g.get_node(0).add_edge(g.get_node(8))
# g.get_node(4).add_edge(g.get_node(5))
# g.get_node(4).add_edge(g.get_node(6))
# g.get_node(2).add_edge(g.get_node(7))
# g.get_node(3).add_edge(g.get_node(7))
# g.get_node(1).add_edge(g.get_node(5))
# g.get_node(8).add_edge(g.get_node(6))

# path = g.find_shortest_path(g.get_node(0), g.get_node(6))
# print(path)