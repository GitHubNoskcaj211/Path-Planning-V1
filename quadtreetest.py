import quads
import math
import random

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
        self.nodes_index = -1
        self.rtree_index = -1
        
    # for the quadtree to store our structure
    def get_x(self):
        return self.pos.x
    
    def get_y(self):
        return self.pos.y
        
    def __str__(self):
        s = 'Node ' + str(self.nodes_index) + ': ' + str(self.pos) + (' | Parent: ' + self.parent.index if self.parent != None else ' ')
        return s
        
    def distance(self, node2):
        return math.sqrt((self.pos.x - node2.pos.x) ** 2 + (self.pos.y - node2.pos.y) ** 2)


tree = quads.QuadTree((15, 15), 30, 30)

for i in range(3000):
    x = random.random() * 30
    y = random.random() * 30
    tree.insert((x, y), data=Node(Point(x,y)))
    bb = quads.BoundingBox(min_x=x-3, min_y=y-3, max_x=x+3, max_y=y+3)


for i in tree.within_bb(bb):
    print(i.data)