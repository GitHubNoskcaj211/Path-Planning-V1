import math

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
        self.graph = []

    def add_node(self, node):
        self.graph.append(node)
        node.index = len(self.graph) - 1
    
    def get_node(self, index):
        return self.graph[index]

    def find_shortest_path(self, starting_node, goal_node):
        starting_node_index = starting_node.index
        goal_node_index = goal_node.index
        num_nodes = len(self.graph)
        node_list = [[None, None] for i in range(num_nodes)] #current weight number, and what parent it has in the tree. Both initialize to None
        node_list[starting_node_index][0] = 0
        visited = [False] * num_nodes
        
        while not visited[goal_node_index]:           
            # get the smallest node to connect in
            smallest_pos = -1
            for i in range(len(node_list)):
                if not visited[i] and node_list[i][0] != None and (smallest_pos == -1 or node_list[i][0] < node_list[smallest_pos][0]):
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
        
        if not visited[goal_node_index]:
            return None
        
        
        
        path = Path()
        n_index = goal_node_index
        while n_index != None:
            path.prepend_node(self.graph[n_index])
            n_index = node_list[n_index][1]
            
        return path
            
            
class Node():
    def __init__(self, point):
        self.pos = point
        self.edges = []
        index = -1
        
    def __str__(self):
        return 'Node ' + str(self.index) + ': ' + str(self.pos)
        
    def distance(self, node2):
        return self.pos.distance(node2.pos)

    def add_edge(self, node2):
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
        
        
# testing path creation
g = Graph()
g.add_node(Node(Point(0,0)))
g.add_node(Node(Point(0,1)))
g.add_node(Node(Point(0,2)))
g.add_node(Node(Point(1,0)))
g.add_node(Node(Point(1,1)))
g.add_node(Node(Point(1,2)))
g.add_node(Node(Point(2,0)))
g.add_node(Node(Point(2,1)))
g.add_node(Node(Point(2,2)))
g.get_node(0).add_edge(g.get_node(1))
g.get_node(0).add_edge(g.get_node(8))
g.get_node(4).add_edge(g.get_node(5))
g.get_node(4).add_edge(g.get_node(6))
g.get_node(2).add_edge(g.get_node(7))
g.get_node(3).add_edge(g.get_node(7))
g.get_node(1).add_edge(g.get_node(5))
g.get_node(8).add_edge(g.get_node(6))

path = g.find_shortest_path(g.get_node(0), g.get_node(6))
print(path)