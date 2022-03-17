from pickletools import stringnl
from rtree import index
import math
import random
import time

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

n = Node(Point(0,0))
n.index = 0
idx = index.Index()
left, bottom, right, top = (0.0, 0.0, 0.0, 0.0)

for i in range(0,10000):
    idx.insert(i, (random.random()-1,random.random()-1,random.random(),random.random()), i)

# idx.delete(0, (0,0,0,0))

# gen = idx.intersection((-1.0, -1.0, 2.0, 2.0), objects=True)
# list(gen)
# list(idx.intersection((1.0000001, 1.0000001, 2.0, 2.0)))
print('filled', time.time(),flush=True)
# for i in idx.nearest((512,213,512,213), num_results=1000):
#     print(i)
# print(idx.get_size())
x = idx.nearest((512,213,512,213), num_results=10000, objects=True)
# for i in idx.nearest((0,0,0,0), num_results=10000, objects=True):
#     print(i.object, i.bounds)
print('done gen', time.time(), flush=True)
print(idx.get_size())


print('here')
idx = index.Index()
for i in range(0,500):
    idx.insert(i, (i,i,i,i), str(i))
for i in range(0,4500):
    idx.delete(124, (124,124,124,124))
    idx.insert(124, (124,124,124,124))
print('there')