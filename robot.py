from tree import Point as TreePoint
import math
from shapely.geometry import Point, Polygon
import numpy as np


class Robot():
    def __init__(self, start):
        self.position = TreePoint(start[0], start[1], start[2])
        
        self.max_turning_speed = 0.3
        self.max_movement_speed = 0.5
        
        self.sensing_angle = math.pi / 4
        self.sensing_radius = 6
        
    # returns true if the robot was moved and false if the robot was not moved (already at the point)
    def move_robot(self, next_pos):
        target_theta = self.position.get_angle_towards(next_pos)
        movement_direction = TreePoint(next_pos.x - self.position.x, next_pos.y - self.position.y, target_theta - self.position.theta)
        
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
        
        self.position.add(movement_direction)
        
        return True
    
    def get_sensor_distance(self, obstacles):
        p = Point(self.position.x, self.position.y, self.position.theta)
        sector = self.get_sensing_sector()
        distance = np.inf
        for obstacle in obstacles:
            if obstacle.distance(p) > self.sensing_radius:
                continue
            else:
                if sector.intersection(obstacle).area > 0:
                    cur_distance = obstacle.distance(p)
                    if cur_distance < distance:
                        distance = cur_distance
                        
        return distance
    
    def get_sensing_sector(self, steps = 200):
        center = Point(self.position.x, self.position.y, self.position.theta)
        
        #in degrees
        start_angle = (self.position.theta - self.sensing_angle / 2) * 180 / math.pi
        end_angle = (self.position.theta + self.sensing_angle / 2) * 180 / math.pi
        
        def polar_point(origin_point, angle,  distance):
            return [origin_point.x + math.cos(math.radians(angle)) * distance, origin_point.y + math.sin(math.radians(angle)) * distance]

        if start_angle > end_angle:
            start_angle = start_angle - 360
        else:
            pass
        step_angle_width = (end_angle-start_angle) / steps
        sector_width = (end_angle-start_angle) 
        segment_vertices = []

        segment_vertices.append(polar_point(center, 0,0))
        segment_vertices.append(polar_point(center, start_angle, self.sensing_radius))

        for z in range(1, steps):
            segment_vertices.append((polar_point(center, start_angle + z * step_angle_width, self.sensing_radius)))
        segment_vertices.append(polar_point(center, start_angle+sector_width, self.sensing_radius))
        segment_vertices.append(polar_point(center, 0,0))
        return Polygon(segment_vertices)
    
    def display(self, ax):
        ax.arrow(self.position.x, self.position.y, .4*math.cos(self.position.theta), .4*math.sin(self.position.theta), alpha=0.6, width = 0.05, color='red')
        
        x, y = self.get_sensing_sector().exterior.xy
        ax.plot(x, y, color='orange', alpha=1,
        linewidth=3, solid_capstyle='round', zorder=2)