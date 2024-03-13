#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import math
import time
import numpy as np
import copy
 
#cpg network
import hexapod_robot.cpg.oscilator_network as osc
 
#import messages
from messages import *
 
import matplotlib.pyplot as plt
 
import scipy.ndimage as ndimg
from sklearn.cluster import KMeans
 
import skimage.measure as skm
 
import collections
import heapq
 
from queue import PriorityQueue
import math

def path_length(path: Path):
    total = 0
    for i in range(len(path.poses) - 1):
        total += path.poses[i].dist(path.poses[i + 1])
    return total
 
def bresenham_line(start, goal):
    """Bresenham's line algorithm
    Args:
        start: (float64, float64) - start coordinate
        goal: (float64, float64) - goal coordinate
    Returns:
        interlying points between the start and goal coordinate
    """
    (x0, y0) = start
    (x1, y1) = goal
    line = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            line.append((x,y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            line.append((x,y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    x = goal[0]
    y = goal[1]
    return line
 
def heuristic(row, col, goal_row, goal_col):
    return math.sqrt((row - goal_row)**2 + (col - goal_col)**2)
 
def astar(maze, start, end):
        open_set = PriorityQueue()
        open_set.put((0, start[0], start[1]))
 
        came_from = {}
        g_score = {(row_i, col_i): float('inf') for row_i in range(len(maze)) for col_i in range(len(maze[row_i]))}
        g_score[(start[0], start[1])] = 0
 
        while not open_set.empty():
            _, current_row, current_col = open_set.get()
 
            if current_row == end[0] and current_col == end[1]:
                # Reconstruct the path and return it
                path = []
                pos = (current_row, current_col)
                while pos in came_from:
                    path.append(pos)
                    pos = came_from[pos]
                path.reverse()
                return path
 
            neighbors = [(current_row - 1, current_col), (current_row + 1, current_col),
                         (current_row, current_col - 1), (current_row, current_col + 1),
                         (current_row - 1, current_col + 1), (current_row + 1, current_col - 1),
                         (current_row - 1, current_col - 1), (current_row + 1, current_col + 1)]
 
            for neighbor_row, neighbor_col in neighbors:
                if (
                    neighbor_row < 0
                    or neighbor_row >= len(maze)
                    or neighbor_col < 0
                    or neighbor_col >= len(maze[0])
                    or maze[neighbor_row][neighbor_col] == 1.0
                ):
                    continue
 
                # handle guide the search from the delays with g score
 
                # TODO improve the score
                tentative_g_score = g_score[(current_row, current_col)] + heuristic(neighbor_row, neighbor_col, current_row, current_col)
 
                if tentative_g_score < g_score[(neighbor_row, neighbor_col)]:
                    came_from[(neighbor_row, neighbor_col)] = (current_row, current_col)
                    g_score[(neighbor_row, neighbor_col)] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor_row, neighbor_col, end[0], end[1])
                    open_set.put((f_score, neighbor_row, neighbor_col))


def update_free(P_mi):
    """method to calculate the Bayesian update of the free cell with the current occupancy probability value P_mi 
    Args:
        P_mi: float64 - current probability of the cell being occupied
    Returns:
        p_mi: float64 - updated probability of the cell being occupied
    """
    p_occ = 1.95/2
    p_free = 0.05/2
    p_mi = (p_free * P_mi / (p_occ * P_mi + (p_free * (1 - P_mi))))
    return p_mi
 
def update_occupied(P_mi):
    """method to calculate the Bayesian update of the occupied cell with the current occupancy probability value P_mi
    Args:
        P_mi: float64 - current probability of the cell being occupied
    Returns:
        p_mi: float64 - updated probability of the cell being occupied
    """
    p_occ = 1.95/2
    p_free = 0.05/2
    p_mi = (p_occ * P_mi / (p_occ * P_mi + (p_free * (1 - P_mi))))
    return p_mi

def compute_mutal_information(p_occ):
    h_mi = -p_occ * math.log(p_occ) - (1 - p_occ) * math.log(1 - p_occ)
    return h_mi
 
class HexapodExplorer:
 
    def __init__(self):
        self.map_set = False
        self.global_map = None
        pass
    
    def project_laser_scan(self, grid_map: OccupancyGrid, laser_scan: LaserScan, odometry: Odometry):
        robot_heading = odometry.pose.orientation.to_Euler()[0]
        laser_points_x = []
        laser_points_y = []
         
        # positioning the lidar scans into the world scan
        for i, d in enumerate(laser_scan.distances):
            theta = robot_heading + laser_scan.angle_min + i * laser_scan.angle_increment
            laser_points_x.append(d * math.cos(theta) + odometry.pose.position.x)
            laser_points_y.append(d * math.sin(theta) + odometry.pose.position.y)
 
        # handling the grid
        map_orig_x = grid_map.origin.position.x
        map_orig_y = grid_map.origin.position.y
        resolution = grid_map.resolution

        for i, (x, y) in enumerate(zip(laser_points_x, laser_points_y)):
            laser_points_x[i] = round((x - map_orig_x) / resolution)
            laser_points_y[i] = round((y - map_orig_y) / resolution)

        return laser_points_x, laser_points_y
    
    def init_gridmap(self, grid_map: OccupancyGrid, odometry: Odometry):
        cur_x = odometry.pose.position.x
        cur_y = odometry.pose.position.y
        map_orig_x = math.floor(cur_x)
        map_orig_y = math.floor(cur_y)

        grid_map = OccupancyGrid()
        grid_map.resolution = 0.1
        grid_map.origin = Pose(Vector3(map_orig_x, map_orig_y, 0.0), Quaternion(1, 0, 0, 0))
        grid_map.width = 10
        grid_map.height = 10
        grid_map.data = 0.5 * np.ones((grid_map.height * grid_map.width))
        return grid_map
    
    def points_in_map(self, grid_map: OccupancyGrid, points):
        points_x = points[0]
        points_y = points[1]
        max_x = max(points_x)
        min_x = min(points_x)
        max_y = max(points_y)
        min_y = min(points_y)
        return max_x < grid_map.width and min_x >= 0 and max_y < grid_map.height and min_y >= 0
    
    def resize_gridmap(self, grid_map: OccupancyGrid, points):
        points_x = points[0]
        points_y = points[1]
        x_shift = min(0, min(points_x))
        y_shift = min(0, min(points_y))
        new_width = max(grid_map.width, max(points_x)) - x_shift + 1
        new_height = max(grid_map.height, max(points_y)) - y_shift + 1

        grid_map.origin.position.x = x_shift * grid_map.resolution + grid_map.origin.position.x
        grid_map.origin.position.y = y_shift * grid_map.resolution + grid_map.origin.position.y

        new_data = 0.5 * np.ones((new_height, new_width))
        new_data[-y_shift:grid_map.height - y_shift, -x_shift:grid_map.width - x_shift] = grid_map.data.reshape(grid_map.height, grid_map.width)

        grid_map.width = new_width
        grid_map.height = new_height
        grid_map.data = new_data.flatten()
        return grid_map

    def fuse_laser_scan_resizing(self, grid_map: OccupancyGrid, laser_scan: LaserScan, odometry: Odometry):
        if odometry == None:
            return grid_map

        if grid_map is None:
            grid_map = self.init_gridmap(grid_map, odometry)
        
        laser_points_x, laser_points_y = self.project_laser_scan(grid_map, laser_scan, odometry)

        if not self.points_in_map(grid_map, (laser_points_x, laser_points_y)):
            grid_map = self.resize_gridmap(grid_map, (laser_points_x, laser_points_y))
            laser_points_x, laser_points_y = self.project_laser_scan(grid_map, laser_scan, odometry)

        resolution = grid_map.resolution
        map_orig_x = grid_map.origin.position.x
        map_orig_y = grid_map.origin.position.y

        robot_x = round((odometry.pose.position.x - map_orig_x) / resolution)
        robot_y = round((odometry.pose.position.y - map_orig_y) / resolution)

        free_points = []
        for (x, y) in zip(laser_points_x, laser_points_y):
            pts = bresenham_line((robot_x, robot_y), (x, y))
            free_points.extend(pts)

        #construct the 2D ggrid
        data = grid_map.data.reshape(grid_map.height, grid_map.width)
        for (x, y) in zip(laser_points_x, laser_points_y):
            data[y, x] = update_occupied(data[y,x])
        for (x, y) in free_points:
            data[y, x] = update_free(data[y,x])
 
        grid_map.data = data.flatten()
        return grid_map
 
    def fuse_laser_scan(self, grid_map: OccupancyGrid, laser_scan: LaserScan, odometry: Odometry):
        """ Method to fuse the laser scan data sampled by the robot with a given 
            odometry into the probabilistic occupancy grid map
        Args:
            grid_map: OccupancyGrid - gridmap to fuse te laser scan to
            laser_scan: LaserScan - laser scan perceived by the robot
            odometry: Odometry - perceived odometry of the robot
        Returns:
            grid_map_update: OccupancyGrid - gridmap updated with the laser scan data
        """
        grid_map_update = copy.deepcopy(grid_map)

        if odometry == None:
            return grid_map_update
 
        robot_heading = odometry.pose.orientation.to_Euler()[0]
        laser_points_x = []
        laser_points_y = []
         
        # positioning the lidar scans into the world scan
        for i, d in enumerate(laser_scan.distances):
            theta = robot_heading + laser_scan.angle_min + i * laser_scan.angle_increment
            laser_points_x.append(d * math.cos(theta) + odometry.pose.position.x)
            laser_points_y.append(d * math.sin(theta) + odometry.pose.position.y)
 
        # handling the grid
        map_orig_x = grid_map.origin.position.x
        map_orig_y = grid_map.origin.position.y
        resolution = grid_map.resolution
        for i, (x, y) in enumerate(zip(laser_points_x, laser_points_y)):
            laser_points_x[i] = round((x - map_orig_x) / resolution)
            laser_points_y[i] = round((y - map_orig_y) / resolution)
         
        # filter the points
        for i, (x, y) in enumerate(zip(laser_points_x, laser_points_y)):
            if x < 0 or x >= grid_map.width or y < 0 or y >= grid_map.height:
                laser_points_x.pop(i)
                laser_points_y.pop(i)

        # print(laser_points_x)
        # plt.scatter(laser_points_x, laser_points_y)
        # plt.grid()
        # plt.show()

        robot_x = round((odometry.pose.position.x - map_orig_x) / resolution)
        robot_y = round((odometry.pose.position.y - map_orig_y) / resolution)

        occupied_points = []
        free_points = []

        for (x, y) in zip(laser_points_x, laser_points_y):
            pts = bresenham_line((robot_x, robot_y), (x, y))

            #save the coordinate of free space cells
            free_points.extend(pts)
            #save the coordinate of occupied cell 
            occupied_points.append((x, y))
        
        # print(laser_points_x)

        #construct the 2D ggrid
        if not self.map_set:
            data = grid_map.data.reshape(grid_map_update.height, grid_map_update.width)
        else:
            data = self.global_map
 
        for (x, y) in zip(laser_points_x, laser_points_y):
            data[y, x] = update_occupied(data[y,x])

        for (x, y) in free_points:
            data[y, x] = update_free(data[y,x])
 
        #serialize the data back (!watch for the correct width and height settings if you are doing the harder assignment)
        grid_map_update.data = data.flatten()
        

        # plt.scatter(laser_points_x, laser_points_y)
        # plt.scatter(list(map(lambda p: p[0], free_points)), list(map(lambda p: p[1], free_points)))
        self.global_map = data
 
 
        return grid_map_update
 
 
    def find_free_edge_frontiers(self, grid_map: OccupancyGrid, use_kmeans: bool = True):
        """Method to find the free-edge frontiers (edge clusters between the free and unknown areas)
        Args:
            grid_map: OccupancyGrid - gridmap of the environment
        Returns:
            pose_list: Pose[] - list of selected frontiers
        """
        data = grid_map.data.reshape(grid_map.height, grid_map.width)
        
        free_c = data < 0.5
        unknown_c = data == 0.5
        # convolution for cells with free neighbour
        mask = np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]])
        free_neighbours = ndimg.convolve(free_c, mask, mode='constant', cval=0.0)
        unknown_neighbours = ndimg.convolve(unknown_c, mask, mode='constant', cval=0.0)

        frontier_c = free_neighbours & unknown_neighbours & free_c

        # convolution for cells with unknown neighbour
        labeled_image, num_labels = skm.label(frontier_c, connectivity=2, return_num=True)
        # plt.imshow(labeled_image, cmap="viridis")

        if num_labels == 0:
            return None
        else:
            pose_list = []

            for label in range(1, num_labels + 1):
                indicies = np.where(labeled_image == label)
                means = []

                if use_kmeans:
                    f = len(indicies[0])
                    D = 100
                    count_clusters = 1 + math.floor(f / D + 0.5)
                    kmeans = KMeans(n_clusters=count_clusters, random_state=0, n_init='auto').fit(np.array(indicies).T)
                    for i in range(count_clusters):
                        means.append((kmeans.cluster_centers_[i][1], kmeans.cluster_centers_[i][0]))
                else:
                    means.append((np.mean(indicies[1]), np.mean(indicies[0])))


                for (x, y) in means:
                    pose = Pose()
                    pose.position.x = x * grid_map.resolution
                    pose.position.y = y * grid_map.resolution

                    pose_list.append(pose)

            for p in pose_list:
                p.position.x += grid_map.origin.position.x
                p.position.y += grid_map.origin.position.y

            return pose_list
        
    def find_inf_frontiers(self, grid_map: OccupancyGrid, mutal_information_radius: float = 1.0, use_kmeans: bool = True):
        """Method to find the frontiers based on information theory approach
        Args:
            grid_map: OccupancyGrid - gridmap of the environment
        Returns:
            pose_list: Pose[] - list of selected frontiers
        """
        # create the mutal information map for each cell
        data = grid_map.data.reshape(grid_map.height, grid_map.width)
        mutal_informations = np.zeros((grid_map.height, grid_map.width))
        for y in range(grid_map.height):
            for x in range(grid_map.width):
                p_occ = update_occupied(data[y, x])
                if p_occ == 0 or p_occ == 1:
                    mutal_informations[y, x] = 0
                    continue
                h_mi = compute_mutal_information(p_occ)
                mutal_informations[y, x] = h_mi

        neighborhood_cell_count = round(mutal_information_radius / grid_map.resolution)
 
        pose_list = self.find_free_edge_frontiers(grid_map, use_kmeans)
        map_orig_x = grid_map.origin.position.x
        map_orig_y = grid_map.origin.position.y

        # compute the mutal information for each pose
        pose : Pose
        for pose in pose_list:
            sx = round((pose.position.x - map_orig_x) / grid_map.resolution)
            sy = round((pose.position.y - map_orig_y) / grid_map.resolution)

            x_boundries = (sx - neighborhood_cell_count, sx + neighborhood_cell_count -1)
            y_boundries = (sy - neighborhood_cell_count, sy + neighborhood_cell_count -1)
            x_boundries = (max(0, x_boundries[0]), min(grid_map.width, x_boundries[1]))
            y_boundries = (max(0, y_boundries[0]), min(grid_map.height, y_boundries[1]))

            mutal_info = np.sum(mutal_informations[y_boundries[0]:y_boundries[1], x_boundries[0]:x_boundries[1]])
            pose.orientation.z = mutal_info

        return pose_list
 
    def grow_obstacles(self, grid_map: OccupancyGrid, robot_size: float, robot: Pose = None, inflate_unknown: bool = False):
        """ Method to grow the obstacles to take into account the robot embodiment
        Args:
            grid_map: OccupancyGrid - gridmap for obstacle growing
            robot_size: float - size of the robot
        Returns:
            grid_map_grow: OccupancyGrid - gridmap with considered robot body embodiment
        """
        grid_map.data = grid_map.data.reshape(grid_map.height, grid_map.width)
 
        grid_map_grow = copy.deepcopy(grid_map)
 
        obstacles = np.zeros((grid_map.height, grid_map.width))

        for x in range(grid_map.height):
            for y in range(grid_map.width):
                if grid_map.data[x][y] < 0.5:
                    obstacles[x, y] = 1
                
                if grid_map.data[x][y] == 0.5 and not inflate_unknown:
                    obstacles[x, y] = 1
         
        distances = ndimg.distance_transform_edt(obstacles)
 
        for x in range(grid_map.height):
            for y in range(grid_map.width):
                if distances[x, y] * grid_map.resolution < robot_size:
                    grid_map_grow.data[x, y] = 1.0
                else:
                    grid_map_grow.data[x, y] = 0.0
        
        # added not to inflate the empty cells
        for x in range(grid_map.height):
            for y in range(grid_map.width):
                if grid_map.data[x][y] == 0.5:
                    grid_map_grow.data[x][y] = 1.0
        
        if robot == None:
            return grid_map_grow

        # need to grow around robot
        rx = round((robot.position.x) / grid_map.resolution)
        ry = round((robot.position.y) / grid_map.resolution)
        robot_size_cells = round(0.3 / grid_map.resolution)

        for x in range(robot_size_cells):
            for y in range(robot_size_cells):
                grid_map_grow.data[ry + y, rx + x] = 0
                grid_map_grow.data[ry - y, rx - x] = 0
                grid_map_grow.data[ry - y, rx + x] = 0
                grid_map_grow.data[ry + y, rx - x] = 0
 
        return grid_map_grow
 
 
 
    def plan_path(self, grid_map: OccupancyGrid, start: Pose, goal: Pose) -> Path:
        """ Method to plan the path from start to the goal pose on the grid
        Args:
            grid_map: OccupancyGrid - gridmap for obstacle growing
            start: Pose - robot start pose
            goal: Pose - robot goal pose
        Returns:
            path: Path - path between the start and goal Pose on the map
        """
        # shift start and goal to the gridmap origin
        # start.position.x -= grid_map.origin.position.x
        # start.position.y -= grid_map.origin.position.y
        goal.position.x -= grid_map.origin.position.x
        goal.position.y -= grid_map.origin.position.y
 
        path = Path()
        #add the start pose
        path.poses.append(start)
 
        grid_res = grid_map.resolution
        start_preprocessed = (round(start.position.y / grid_res), round(start.position.x / grid_res))
        goal_preprocessed = (round(goal.position.y / grid_res), round(goal.position.x / grid_res))
        # print(start.position.y, start_preprocessed[0])
        # print(start.position.x, start_preprocessed[1])

        planned_path = astar(grid_map.data, start_preprocessed, goal_preprocessed)
        if planned_path is None:
            return None

        # plt.title("Planned path")
        # plt.imshow(grid_map.data.reshape(grid_map.height, grid_map.width), cmap='gray')
        # plt.scatter(list(map(lambda p: p[1], planned_path)), list(map(lambda p: p[0], planned_path)), c='b')
        # plt.scatter(start_preprocessed[1], start_preprocessed[0], c='r')
        # plt.scatter(goal_preprocessed[1], goal_preprocessed[0], c='g')
        # plt.show()
 
        for point in planned_path:
            new_pose = Pose()
            new_pose.position.x = point[1] * grid_res
            new_pose.position.y = point[0] * grid_res
            path.poses.append(new_pose)

        path.poses.append(goal)
 
        return path
 
    def simplify_path(self, grid_map: OccupancyGrid, path: Path) -> Path:
        """ Method to simplify the found path on the grid
        Args:
            grid_map: OccupancyGrid - gridmap for obstacle growing
            path: Path - path to be simplified
        Returns:
            path_simple: Path - simplified path
        """
 
        if grid_map == None or path == None:
            return None
  
        path_simplified = Path()
        #add the start pose
        path_simplified.poses.append(path.poses[0])
         
        shorten_start = 1
        #iterate through the path and simplify the path
        while path_simplified.poses[-1] != path.poses[-1]: #until the goal is not reached
        #find the connected segment
            previous_pose = path_simplified.poses[-1]

            for i, pose in enumerate(path.poses[shorten_start:]):
                x1 = round(path_simplified.poses[-1].position.x / grid_map.resolution)
                y1 = round(path_simplified.poses[-1].position.y / grid_map.resolution)
                x2 = round(pose.position.x / grid_map.resolution)
                y2 = round(pose.position.y / grid_map.resolution)
 
                # rr, cc = skimage.draw.line(x1, y1, x2, y2)
                # bres_points = list(zip(cc, rr))
                bres_points = bresenham_line((x1, y1), (x2, y2))
                if all(grid_map.data[point[1]][point[0]] <= 0.5 for point in bres_points): #there is no collision
                    previous_pose = pose
 
                    #the goal is reached
                    if pose == path.poses[-1]:  
                        path_simplified.poses.append(pose)
                        break
             
                else: #there is collision
                    path_simplified.poses.append(previous_pose)
                    shorten_start += i
                    break
         
        # path_simplified.poses.append(path.poses[-1])
    
        # x = [p.position.x + grid_map.origin.position.x for p in path_simplified.poses]
        # y = [p.position.y + grid_map.origin.position.y for p in path_simplified.poses]

        # _, ax = plt.subplots()
        # plt.title("Simplified path")
        # grid_map.plot(ax)
        # plt.scatter(x, y, c='red')
        # plt.plot(x, y, c='red')
        # plt.show()
 
        return path_simplified
  
    ###########################################################################
    #INCREMENTAL Planner
    ###########################################################################
 
    def plan_path_incremental(self, grid_map, start, goal):
        """ Method to plan the path from start to the goal pose on the grid
        Args:
            grid_map: OccupancyGrid - gridmap for obstacle growing
            start: Pose - robot start pose
            goal: Pose - robot goal pose
        Returns:
            path: Path - path between the start and goal Pose on the map
        """
 
        if not hasattr(self, 'rhs'): #first run of the function
            self.rhs = np.full((grid_map.height, grid_map.width), np.inf)
            self.g = np.full((grid_map.height, grid_map.width), np.inf)
 
        #TODO:[t1x-dstar] plan the incremental path between the start and the goal Pose
  
        return self.plan_path(grid_map, start, goal), self.rhs.flatten(), self.g.flatten()