from occupancygrid import OccupancyGrid
from tree import Tree
from tree import Point as TreePoint
from robot import Robot
import matplotlib.pyplot as plt
import math
import random
from shapely.geometry import Point, Polygon
import numpy as np
import time

class Simulation():
    def __init__(self, start, goal, world_bounds):
        self.world_x_min = world_bounds[0]
        self.world_x_max = world_bounds[2]
        self.world_y_min = world_bounds[1]
        self.world_y_max = world_bounds[3]
        
        self.robot = Robot(start)
        self.goal = TreePoint(goal[0], goal[1])
        
        self.obstacles = []
        self.occupancy = OccupancyGrid(world_bounds, self.robot.sensing_radius, self.robot.sensing_angle)
        
        self.rrtree = Tree((self.robot.position.x, self.robot.position.y), 3, world_bounds, self.occupancy)
        self.path = None
        
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        
        self.fig.set_figwidth(8)
        self.fig.set_figheight(8)
        
    def generate_obstacles(self, N, radius=3):
        n = self.world_x_max # Question: assumes square?
        self.obstacles = []
        for i in range(N):
            point = Point(random.random() * n, random.random() * n)
            while True:
                for obstacle in self.obstacles:
                    if obstacle.distance(point) < radius:
                        point = Point(random.random() * n, random.random() * n)
                        break
                break
            points = []
            for i in np.arange(0, math.pi*2, math.pi/4):
                mag = np.random.uniform(low=radius/2, high=radius)
                points.append([math.cos(i) * mag + point.x, math.sin(i) * mag + point.y])
            points.append(points[0])
            obstacle = Polygon(points)
            self.obstacles.append(obstacle)
    
    def plot_obstacles(self, ax):
        for obstacle in self.obstacles:
            x, y = obstacle.exterior.xy
            ax.plot(x, y, color='#6699cc', alpha=1,
            linewidth=3, solid_capstyle='round', zorder=2)
            
    def plot_path(self, ax):
        if self.path == None:
            return
        if len(self.path) > 0:
            ax.plot([self.robot.position.x, self.path[0].x], [self.robot.position.y, self.path[0].y], linewidth=0.75, color='y')
            for i in range(len(self.path) - 1):
                ax.plot([self.path[i].x, self.path[i+1].x], [self.path[i].y, self.path[i+1].y], linewidth=0.75, color='y')
        
    def build_initial_rrt(self):
        # while we cannot add the goal node
        while not self.rrtree.add_goal_node(self.goal.x, self.goal.y):
            self.rrtree.build_tree(500)
        
        self.path = self.rrtree.get_path_to_goal()
            
    # returns true when the path is finished, otherwise returns false
    def step(self):
        if len(self.path) == 0:
            return True
        
        sensor_distance = self.robot.get_sensor_distance(self.obstacles)
        occupancy_changed = self.occupancy.fill_occupancy(self.robot.position, sensor_distance)
        if occupancy_changed: #occupancy_changed:
            self.display()
            # for angle in np.linspace(self.robot.position.theta, self.robot.position.theta + math.pi*2, 32, endpoint=True):
            #     self.robot.position.theta = angle
            #     sensor_distance = self.robot.get_sensor_distance(self.obstacles)
            #     occupancy_changed = self.occupancy.fill_occupancy(self.robot.position, sensor_distance)
            #     self.display()
            
            # self.robot.position.theta -= math.pi * 2
            self.path = None
            # regenerate!
            self.rrtree = Tree((self.robot.position.x, self.robot.position.y), 3, (self.world_x_min,self.world_y_min,self.world_x_max,self.world_y_max), self.occupancy)
            self.build_initial_rrt()

        # move the robot
        if not self.robot.move_robot(self.path[0]):
            # if the robot is already at its point, get the next point in the path
            self.path.pop(0)
            
        return False
        
    # display on the graph
    def display(self):
        self.ax.cla()
        
        self.occupancy.display(self.ax)
        self.plot_obstacles(self.ax)

        self.rrtree.display(self.ax)
        
        self.plot_path(self.ax)
        
        self.robot.display(self.ax)
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()