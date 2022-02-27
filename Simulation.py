from tree import Tree, Point
import matplotlib.pyplot as plt
import math

class Simulation():
    def __init__(self, start, goal, world_bounds):
        self.robot = Point(start[0], start[1], start[2])
        self.goal = Point(goal[0], goal[1])
        self.rrtree = Tree((self.robot.x, self.robot.y), 1, world_bounds)
        self.path = None
        
        self.max_turning_speed = 0.2
        self.max_movement_speed = 0.2
        
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        
        self.fig.set_figwidth(8)
        self.fig.set_figheight(8)
        
        self.arrow = self.ax.arrow(self.robot.x, self.robot.y, math.cos(self.robot.theta), math.sin(self.robot.theta), alpha=0.6, width = 0.05, color='red')
        
    def build_initial_rrt(self):
        # while we cannot add the goal node
        while not self.rrtree.add_goal_node(self.goal.x, self.goal.y):
            self.rrtree.build_tree(None, 500)
        
        self.path = self.rrtree.get_path_to_goal()
            
    # returns true when the path is finished, otherwise returns false
    def step(self):
        if len(self.path) == 0:
            return True
        
        # TODO check for obstacles
        
        # move the robot
        if not self.move_robot():
            # if the robot is already at its point, get the next point in the path
            self.path.pop(0)
            
        return False
        
    # returns true if the robot was moved and false if the robot was not moved (already at the point)
    def move_robot(self):
        next_pos = self.path[0]
        target_theta = self.robot.get_angle_towards(next_pos)
        movement_direction = Point(next_pos.x - self.robot.x, next_pos.y - self.robot.y, target_theta - self.robot.theta)
        
        # we are already at the point, no more movement necessary
        if movement_direction.mag() < 0.0001:
            return False
        
        # turn before moving x and y
        if abs(movement_direction.theta) >= 0.01:
            movement_direction.x = 0
            movement_direction.y = 0
        elif movement_direction.mag() > self.max_movement_speed:
            mag = movement_direction.mag()
            movement_direction.x = movement_direction.x / mag * self.max_movement_speed
            movement_direction.y = movement_direction.y / mag * self.max_movement_speed
        
        # normalize the movement_direction
        if abs(movement_direction.theta) > self.max_turning_speed:
            movement_direction.theta = movement_direction.theta / abs(movement_direction.theta) * self.max_turning_speed
        
        self.robot.add(movement_direction)
        
        return True

    # display on the graph
    def display(self):
        self.ax.cla()
        self.rrtree.display(self.ax)
        
        self.arrow.remove()
        self.arrow = self.ax.arrow(self.robot.x, self.robot.y, .4*math.cos(self.robot.theta), .4*math.sin(self.robot.theta), alpha=0.6, width = 0.05, color='red')
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()