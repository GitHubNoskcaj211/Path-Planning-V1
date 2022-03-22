import numpy as np
from tree import Point as TreePoint
from shapely.geometry import Point, LineString, Polygon
import math
import seaborn as sns

class OccupancyGrid():
    def __init__(self, world_bounds, sensing_radius, sensing_angle):
        self.grid = np.full((world_bounds[2],world_bounds[3],2), False) # first is if there is an obstacle; second is if it is known
        self.sensing_radius = sensing_radius
        self.sensing_angle = sensing_angle
        
    def query_obstacle(self, x, y):
        return self.grid[y,x,0]
    
    def query_free(self, x, y):
        return not self.grid[y,x,0]
    
    # def query_free(self, x1, y1, x2, y2):
    #     visited = np.full(self.grid.shape[:2], False)
    #     def valid_square(y,x):
    #         y_max = visited.shape[0] - 1
    #         x_max = visited.shape[1] - 1
    #         y_min = 0
    #         x_min = 0
    #         return y <= y_max and y >= y_min and x <= x_max and x >= x_min
        
    #     path = LineString([(x1, y1), (x2, y2)])
        
    #     y = math.floor(y1)
    #     x = math.floor(x1)
    #     queue = [[y,x]]
    #     visited[y,x] = True
    #     while len(queue) > 0:
    #         y, x = queue.pop(0)
            
    #         if self.grid[y,x,0]:
    #             return False
            
    #         if valid_square(y+1,x) and not visited[y+1,x] and path.intersects(LineString([(x, y+1), (x+1, y+1)])):
    #             queue.append([y+1,x])
    #             visited[y+1,x] = True
    #         if valid_square(y-1,x) and not visited[y-1,x] and path.intersects(LineString([(x, y), (x+1, y)])):
    #             queue.append([y-1,x])
    #             visited[y-1,x] = True
    #         if valid_square(y,x+1) and not visited[y,x+1] and path.intersects(LineString([(x+1, y), (x+1, y+1)])):
    #             queue.append([y,x+1])
    #             visited[y,x+1] = True
    #         if valid_square(y,x-1) and not visited[y,x-1] and path.intersects(LineString([(x, y), (x, y+1)])):
    #             queue.append([y,x-1])
    #             visited[y,x-1] = True
                
    #     # went through all squares that intersect the line and did not find 
    #     return True

    def query_free(self, x1, y1, x2, y2):
        # make sure x1 is on the left
        if x1 > x2:
            x1, y1, x2, y2 = x2, y2, x1, y1
        # y = ax + b
        a = (y2 - y1) / (x2 - x1)
        b = y1 - x1 * a
        start = int(x1)
        end = int(x2+1)
        # floor = math.floor if y1 < y2 else math.ceil
        # ceil = math.floor if y1 < y2 else math.floor
        if y1 < y2:
            for index, x_left in enumerate(range(start, end)):
                x_right = x_left + 1
                y_start = int(a*x_left+b)
                y_end = int(a*x_right+b)
                if index == 0:
                    y_start = int(y1)
                if index == end - start - 1:
                    y_end = int(y2)
                y_list = list(range(y_start, y_end+1, 1))
                for y in y_list:
                    if x_left >= self.grid.shape[0] or y >= self.grid.shape[1]:
                        continue
                    if self.grid[y, x_left, 0]:
                        return False
        #     if y1 >= y2:
        #         y_list = [y for y in y_list]
        if y1 >= y2:
            for index, x_left in enumerate(range(start, end)):
                x_right = x_left + 1
                y_start = int(a*x_left+b)
                y_end = int(a*x_right+b)
                if index == 0:
                    y_start = int(y1)
                if index == end - start - 1:
                    y_end = int(y2)
                y_list = list(range(y_start, y_end-1, -1))
                for y in y_list:
                    if x_left >= self.grid.shape[0] or y >= self.grid.shape[1]:
                        continue
                    if self.grid[y, x_left, 0]:
                        return False
        return True

    def fill_occupancy(self, robot_position, distance):
        data_changed = False
        
        if distance == np.inf:
            # no obstacle detected
            obstacle_polygon = self.get_sensing_sector(robot_position, self.sensing_radius)
            
            for y in range(self.grid.shape[0]):
                for x in range(self.grid.shape[1]):
                    square = self.get_occupancy_square_polygon(x,y,x+1,y+1)
                    
                    if obstacle_polygon.intersection(square).area == 1:
                        if self.grid[y,x,0]:
                            # if there was an obstacle there
                            data_changed = True
                        
                        # known that there isnt an obstacle
                        self.grid[y,x,0] = False
                        self.grid[y,x,1] = True
        else:
            # an obstacle was detected
            obstacle_arc = self.get_obstacle_arc(robot_position, distance)
            obstacle_polygon = self.get_sensing_sector(robot_position, distance)
            
            for y in range(self.grid.shape[0]):
                for x in range(self.grid.shape[1]):
                    square = self.get_occupancy_square_polygon(x,y,x+1,y+1)
                    
                    if obstacle_arc.intersects(square) and not self.grid[y,x,1]:
                        if not self.grid[y,x,0]:
                            # if there wasn't an obstacle there
                            data_changed = True
                        
                        # fill as obstacle
                        self.grid[y,x,0] = True
                    elif obstacle_polygon.intersection(square).area == 1:
                        if self.grid[y,x,0]:
                            # if there was an obstacle there
                            data_changed = True
                        
                        # known that there isnt an obstacle
                        self.grid[y,x,0] = False
                        self.grid[y,x,1] = True
                        
        return data_changed
        
    def print(self):
        print('occupancy:')
        for i in range(self.grid.shape[0] - 1, -1, -1):
            for j in range(self.grid.shape[1]):
                if self.grid[i,j,0]:
                    print(9, end=' ')
                else:
                    if self.grid[i,j,1]:
                        print(0, end=' ')
                    else:
                        print(5, end=' ')
            print()
            
    # gets a polygon representing the square on the occupancy grid         
    def get_occupancy_square_polygon(self, start_x, start_y, end_x, end_y):
        segment_vertices = []
        segment_vertices.append([start_x, start_y])
        segment_vertices.append([start_x, end_y])
        segment_vertices.append([end_x, end_y])
        segment_vertices.append([end_x, start_y])
        segment_vertices.append([start_x, start_y])
        
        return Polygon(segment_vertices)
        
    # gets a LineString representing the arc where the obstacle is
    def get_obstacle_arc(self, robot_position, distance, steps = 200):
        center = Point(robot_position.x, robot_position.y, robot_position.theta)
        
        #in degrees
        start_angle = (robot_position.theta - self.sensing_angle / 2) * 180 / math.pi
        end_angle = (robot_position.theta + self.sensing_angle / 2) * 180 / math.pi
        
        def polar_point(origin_point, angle,  distance):
            return [origin_point.x + math.cos(math.radians(angle)) * distance, origin_point.y + math.sin(math.radians(angle)) * distance]
        
        segment_vertices = []
        step_angle_width = (end_angle-start_angle) / steps
        sector_width = (end_angle-start_angle) 
        
        for z in range(0, steps):
            segment_vertices.append((polar_point(center, start_angle + z * step_angle_width, distance)))
        segment_vertices.append(polar_point(center, start_angle+sector_width, distance))
            
        return LineString(segment_vertices)
    
    # gets the Polygon representing the robot sensing sector
    def get_sensing_sector(self, robot_position, distance, steps = 200):
        center = Point(robot_position.x, robot_position.y, robot_position.theta)
        
        #in degrees
        start_angle = (robot_position.theta - self.sensing_angle / 2) * 180 / math.pi
        end_angle = (robot_position.theta + self.sensing_angle / 2) * 180 / math.pi
        
        def polar_point(origin_point, angle,  distance):
            return [origin_point.x + math.cos(math.radians(angle)) * distance, origin_point.y + math.sin(math.radians(angle)) * distance]
        
        segment_vertices = []
        step_angle_width = (end_angle-start_angle) / steps
        sector_width = (end_angle-start_angle) 
        
        segment_vertices.append(polar_point(center, 0,0))
        segment_vertices.append(polar_point(center, start_angle, self.sensing_radius))
        for z in range(1, steps):
            segment_vertices.append((polar_point(center, start_angle + z * step_angle_width, distance)))
        segment_vertices.append(polar_point(center, start_angle+sector_width, distance))
        segment_vertices.append(polar_point(center, 0,0))
            
        return Polygon(segment_vertices)

    def display(self, ax, pos, d):
        heatmap_fill = np.zeros(shape=self.grid.shape[:2])
        for i in range(self.grid.shape[0]):
            for j in range(self.grid.shape[1]):
                if self.grid[i,j,0]:
                    heatmap_fill[i,j] = 1 # there is an obstacle
                else:
                    if self.grid[i,j,1]:
                        heatmap_fill[i,j] = 0 # known no obstacle
                    else:
                        heatmap_fill[i,j] = 0.5 # unknown no obstacle
        
        sns.heatmap(heatmap_fill, linewidth=0.1, ax=ax, cbar=False, vmin=0, vmax=1, cmap='Greys')
        ax.invert_yaxis()
        
        if d != np.inf:
            x, y = self.get_obstacle_arc(pos, d).coords.xy
            ax.plot(x, y, color='green', alpha=1,
            linewidth=3, solid_capstyle='round', zorder=2)