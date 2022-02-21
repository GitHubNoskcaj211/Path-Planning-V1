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
    
    def subtract(self, vector):
        return Vector(self.x - vector.x, self.y - vector.y)
    
    def calc_information_vector(self, vector, angle_between):
        # calculating on the wrong side of information gain
        if angle_between > math.pi:
            return 0
        
        information = 0
        # guarenteed triangle area
        l1 = self.magnitude()
        l2 = vector.magnitude()
        smaller_l1 = l1 / 2
        smaller_l2 = l2 / 2
        bisected_angle = angle_between / 2
        # can throw an exception
        angle_bisector_length = smaller_l1 * smaller_l2 * math.sin(angle_between) / (smaller_l1 * math.sin(bisected_angle) + smaller_l2 * math.sin(bisected_angle))
        
        information = 1/2 * l1 * angle_bisector_length * math.sin(bisected_angle) + 1/2 * l2 * angle_bisector_length * math.sin(bisected_angle)
        
        return information
    
    def distance(self, vector):
        return math.sqrt((self.x - vector.x) ** 2 + (self.y - vector.y) ** 2)
        
    def magnitude(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)
    
    def theta(self):
        return math.atan2(self.x, self.y)

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
                n2 = e.node2
                plt.plot([n.pos.x, n2.pos.x], [n.pos.y, n2.pos.y], linewidth=0.1, color='b')
                
        plt.show()
    
    # reports the amount of information covered (as a percentage) in a specific node with its connections
    # returns the percent coverage (out of 1)
    def information_coverage_calc(self, node):
        if len(node.edges) == 0:
            return 0
        
        # consider making this case different
        if len(node.edges) == 1:
            return 0
        
        max_information = math.pi * self.connection_radius ** 2
        angles = [[e, e.vector.theta()] for e in node.edges] # [edge (obj), theta of the edge]
        angles = sorted(angles, key=lambda x: x[1])
        
        total_information = 0
        total_information += angles[-1][0].vector.calc_information_vector(angles[0][0].vector, 2 * math.pi - (angles[-1][1] - angles[0][1])) # check the boundary (last minus first) first
        for i in range(len(angles)-1):
            total_information += angles[i+1][0].vector.calc_information_vector(angles[i][0].vector, (angles[i+1][1] - angles[i][1])) # check the boundary (last minus first) first
            
        return total_information / max_information
        
    def max_edge_angle(self, node):
        if len(node.edges) == 0:
            return math.pi * 2
        
        if len(node.edges) == 1:
            return math.pi * 2
        
        angles = [[e, e.vector.theta()] for e in node.edges] # [edge (obj), theta of the edge]
        angles = sorted(angles, key=lambda x: x[1])
        max_angle = 2 * math.pi - (angles[-1][1] - angles[0][1]) # check the boundary (last minus first) first
        for i in range(len(angles)-1):
            if angles[i+1][1] - angles[i][1] > max_angle:
                max_angle = angles[i+1][1] - angles[i][1]
        print(max_angle)
        return max_angle
            
    def generate_exploratory_node(self, start_node, goal_node):
        # consider idea of how many bits do you need to know where you are in the environemnt based on 
        # where the points are (aka, if you only have 2 points, you wouldnt know where you were in all the empty areas because
        # you have nothing to go off of)
        node_list, visited = self.dijkstras(start_node, goal_node, False)
        print(node_list)
        
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
        
        scores = [[leaf, 1 / (self.graph[leaf].distance(goal_node) + node_list[leaf][0]) * self.information_coverage_calc(self.graph[leaf])] for leaf in leaf_list]
        scores = sorted(scores, key=lambda x: x[1], reverse=True)
        node = self.graph[scores[0][0]]
        print(node)
        
        return self.new_node(node, goal_node)
    
    # gross clean up later  
    def new_node(self, node, goal_node):
        if len(node.edges) == 0:
            max_r = self.connection_radius
            min_r = self.connection_radius / 2
            mag = random.random() * (max_r - min_r) + min_r
            print(mag)
            theta = math.atan2(goal_node.pos.y - node.pos.y, goal_node.pos.x - node.pos.x)
            print(theta)
            return Node(Point(mag * math.cos(theta) + node.pos.x, mag * math.sin(theta) + node.pos.y))
        
        # discretize into every 1 degree
        angles = [i / 180 * math.pi for i in range(-180,180)]
        edges = [[e, e.vector.theta()] for e in node.edges] # [edge (obj), theta of the edge]
        print(edges)
        # copy the last vector before and the first vector after but looping the angles
        copied_e1 = edges[-1].copy()
        copied_e1[1] -= math.pi * 2
        copied_e2 = edges[0].copy()
        copied_e2[1] = copied_e2[1] + math.pi * 2
        edges.insert(0, copied_e1)
        edges.append(copied_e2)
        
        max_score = 0
        max_score_node = None
        
        v1_pos = 0
        v2_pos = 1
        edges = sorted(edges, key=lambda x: x[1])
        for theta in angles:
            if theta > edges[v2_pos][1]:
                v1_pos += 1
                v2_pos += 1
            
            angle_between_old = edges[v2_pos][1] - edges[v1_pos][1]
            if angle_between_old == 0 or angle_between_old >= math.pi:
                old_information = 0
                min_r = self.connection_radius / 2
            else:
                old_information = edges[v1_pos][0].vector.calc_information_vector(edges[v2_pos][0].vector, angle_between_old)
                # get new average point vector
                l1 = edges[v1_pos][0].vector.magnitude()
                t1 = edges[v1_pos][1]
                l2 = edges[v2_pos][0].vector.magnitude()
                t2 = edges[v2_pos][1]
                x1 = l1 * math.cos(t1)
                y1 = l1 * math.sin(t1)
                x2 = l2 * math.cos(t2)
                y2 = l2 * math.sin(t2)
                if x2 - x1 == 0:
                    print('division by 0!!!!!')
                    continue
                m = (y2 - y1) / (x2 - x1)
                min_r = (y1 - m * x1) / (math.sin(theta) - m * math.cos(theta))
            
            max_r = self.connection_radius
            average_r = (min_r + max_r) / 2
            avg_new_vector = Vector(average_r * math.cos(theta), average_r * math.sin(theta))\
            
            
            new_information = 0
            angle_between_new1 = theta - edges[v1_pos][1]
            if angle_between_new1 == 0 or angle_between_new1 >= math.pi:
                new_information += 0
            else:
                new_information += edges[v1_pos][0].vector.calc_information_vector(avg_new_vector, angle_between_new1)
            
            angle_between_new2 = edges[v2_pos][1] - theta
            if angle_between_new2 == 0 or angle_between_new2 >= math.pi:
                new_information += 0
            else:
                new_information += avg_new_vector.calc_information_vector(edges[v2_pos][0].vector, angle_between_new2)
            
            delta_information = new_information - old_information
            score = delta_information * 1 / (avg_new_vector.distance(goal_node.pos) + 1)
            print('theta',theta,'info gain',delta_information,'score',score)
            if delta_information < 0:
                print("BAD!")
                
            if score > max_score:
                max_score = score
                mag = random.random() * (max_r - min_r) + min_r
                max_score_node = Node(Point(mag * math.cos(theta) + node.pos.x, mag * math.sin(theta) + node.pos.y))
            
        return max_score_node
        
    # gross clean up later
    # if a goal node is denoted, it will run the modified a_star encoding
    def dijkstras(self, starting_node, goal_node, stop_early, a_star = False):
        starting_node_index = starting_node.index
        num_nodes = len(self.graph)
        node_list = [[None, None, self.graph[i].distance(goal_node)] for i in range(num_nodes)] #current weight number, and what parent it has in the tree, distance from goal. First two initialize to None
        node_list[starting_node_index][0] = 0
        visited = [False] * num_nodes
        
        # if we are in a_star mode, continue until all possible nodes are searched. Otherwise, end when
        # goal is reached
        while not visited[goal_node.index] or not stop_early:           
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
                if edge.active and not visited[edge.node2.index]:
                    new_weight = node_list[smallest_pos][0] + edge.weight 
                    old_weight = node_list[edge.node2.index][0]
                    if old_weight == None or new_weight < old_weight:
                        node_list[edge.node2.index][0] = new_weight
                        node_list[edge.node2.index][1] = edge.node1.index
                
            visited[smallest_pos] = True
        
        return node_list, visited
        
    # given a starting node and goal node of the graph, return a Path (if one exists)
    def find_shortest_path(self, starting_node, goal_node):
        node_list, visited = self.dijkstras(starting_node, goal_node, True)
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
        self.edges.append(Edge(self, node2, self.distance(node2), True))
        node2.edges.append(Edge(node2, self, self.distance(node2), True))

class Edge():
    def __init__(self, node1, node2, distance, active):
        self.node1 = node1
        self.node2 = node2
        self.weight = distance
        self.active = active
        self.vector = Vector(self.node2.pos.x - self.node1.pos.x, self.node2.pos.y - self.node1.pos.y)

    def set(self, active):
        self.active = active
        
        
    def __str__(self):
        return '(' + str(self.node1.index) + ' to ' + str(self.node2.index) + ',' + str(self.weight) + ',' + str(self.active) + ')'
    
    def __eq__(self, edge):
        return self.node1.index == edge.node1.index and self.node2.index == edge.node2.index
        
        
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