#!/usr/bin/env python3

import math
import numpy as np
import heapq

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from typing import Tuple, List


def distance(point1, point2):
    x1, y1 = point1[0:2]
    x2, y2 = point2[0:2]
    dist2 = (x1 - x2) ** 2 + (y1 - y2) ** 2
    return math.sqrt(dist2)


def is_occupied(map: OccupancyGrid, i: int, j: int) -> bool:
    index = map.info.width * i + j
    if not 0 <= index < len(map.data):
        return False
    return map.data[index] == 100


def is_gridcell_occupied(map: OccupancyGrid, cellsize: int, i: int, j: int) -> bool:
    i_range = range(i * cellsize, (i + 1) * cellsize)
    j_range = range(j * cellsize, (j + 1) * cellsize)
    for i in i_range:
        for j in j_range:
            if is_occupied(map, i, j):
                return True
    return False


class GridMap:
    def __init__(self, map: OccupancyGrid, cellsize: int, movement="8N"):
        """param movement can be either 4-connectivity ('4N') or 8-connectivity ('8N', default)"""
        self.resolution = map.info.resolution * cellsize
        self.origin = map.info.origin.position

        # generate low resolution grid map for
        height = int(map.info.height / cellsize) + 1
        width = int(map.info.width / cellsize) + 1
        self.grid = np.ndarray((height, width), dtype="bool")
        for i in range(height):
            for j in range(width):
                self.grid[i][j] = is_gridcell_occupied(map, cellsize, i, j)

        # initialize the possible movements for the path-finding algorithm
        if movement == "4N":
            self.movements = [(1, 0, 1.0), (0, 1, 1.0), (-1, 0, 1.0), (0, -1, 1.0)]
        elif movement == "8N":
            self.movements = [
                (1, 0, 1.0),
                (0, 1, 1.0),
                (-1, 0, 1.0),
                (0, -1, 1.0),
                (1, 1, math.sqrt(2)),
                (-1, 1, math.sqrt(2)),
                (-1, -1, math.sqrt(2)),
                (1, -1, math.sqrt(2)),
            ]
        else:
            raise ValueError("Unknown movement")

    def is_occupied_xy(self, xy: Tuple[int, int]) -> bool:
        (i, j) = self.world2grid(xy)
        return self.grid[i][j]

    def is_occupied_idx(self, ij: Tuple[int, int]) -> bool:
        return self.grid[ij]

    def is_inside_idx(self, ij: Tuple[int, int]) -> bool:
        return 0 <= ij[0] < self.grid.shape[0] and 0 <= ij[1] < self.grid.shape[1]

    def world2grid(self, xy: Tuple[int, int]) -> Tuple[int, int]:
        """Returns the grid indices for given x and y coordinates."""
        i = int((xy[1] - self.origin.y) / self.resolution)
        j = int((xy[0] - self.origin.x) / self.resolution)
        return (i, j)

    def grid2world(self, ij: Tuple[int, int]) -> Tuple[int, int]:
        """Returns x and y coordinates for given grid indices."""
        x = ij[1] * self.resolution + self.origin.x + self.resolution / 2
        y = ij[0] * self.resolution + self.origin.y + self.resolution / 2
        return (x, y)

    def a_star(self, start_xy: Point, goal_xy: Point) -> List[Point]:
        """A* algorithm. Returns the resulting path as a list of waypoints."""
        # get array indices of start and goal
        start = self.world2grid((start_xy.x, start_xy.y))
        goal = self.world2grid((goal_xy.x, goal_xy.y))

        visited = np.zeros(self.grid.shape, dtype="bool")

        # check if start and goal nodes correspond to free spaces
        if self.is_occupied_idx(start):
            raise Exception("Start node is not traversable")

        if self.is_occupied_idx(goal):
            raise Exception("Goal node is not traversable")

        # add start node to front
        # front is a list of (total estimated cost to goal, total cost from start to node, node, previous node)
        start_node_cost = 0
        start_node_estimated_cost_to_goal = distance(start, goal) + start_node_cost
        front = [(start_node_estimated_cost_to_goal, start_node_cost, start, None)]

        # use a dictionary to remember where we came from in order to reconstruct the path later on
        came_from = {}

        # while there are elements to investigate in our front.
        while front:
            # get smallest item and remove from front.
            element = heapq.heappop(front)

            # if this has been visited already, skip it
            _total_cost, cost, pos_idx, previous = element
            if visited[pos_idx]:
                continue

            # now it has been visited, mark with cost
            visited[pos_idx] = True

            # set its previous node
            came_from[pos_idx] = previous

            # if the goal has been reached, we are done!
            if pos_idx == goal:
                break

            # check all neighbors
            for dx, dy, deltacost in self.movements:
                # determine new position
                new_x = pos_idx[0] + dx
                new_y = pos_idx[1] + dy
                new_pos = (new_x, new_y)

                # check whether new position is inside the map
                # if not, skip node
                if not self.is_inside_idx(new_pos):
                    continue

                # add node to front if it was not visited before and is not an obstacle
                if (not visited[new_pos]) and (not self.is_occupied_idx(new_pos)):
                    cost = 10.0
                    new_cost = cost + deltacost
                    new_total_cost = new_cost + distance(new_pos, goal)

                    heapq.heappush(front, (new_total_cost, new_cost, new_pos, pos_idx))

        # reconstruct path backwards (only if we reached the goal)
        path = []
        if pos_idx == goal:
            while pos_idx:
                # convert to world coordinates
                x, y = self.grid2world(pos_idx)
                path.append(Point(x=x, y=y))
                pos_idx = came_from[pos_idx]

            # reverse so that path is from start to goal.
            path.reverse()
            path.append(goal_xy)

        return path
