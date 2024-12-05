#!/usr/bin/env python3

import math
import numpy as np

from RuleNode import RuleNode
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import OccupancyGrid
from typing import Tuple

from navigation.AStar import a_star


def is_occupied(map: OccupancyGrid, i, j) -> bool:
    index = map.info.width * i + j
    if not 0 <= index < len(map.data):
        return False
    return map.data[index] == 100


def is_gridcell_occupied(map: OccupancyGrid, cellsize: int, i, j) -> bool:
    i_range = range(i * cellsize, (i + 1) * cellsize)
    j_range = range(j * cellsize, (j + 1) * cellsize)
    for i in i_range:
        for j in j_range:
            if is_occupied(map, i, j):
                return True
    return False


class GridMap:
    def __init__(self, map: OccupancyGrid, cellsize: int):
        self.resolution = map.info.resolution * cellsize
        self.origin = map.info.origin.position

        height = int(map.info.height / cellsize) + 1
        width = int(map.info.width / cellsize) + 1
        self.grid = np.ndarray((height, width), dtype="bool")
        for i in range(height):
            for j in range(width):
                self.grid[i][j] = is_gridcell_occupied(map, cellsize, i, j)

        path, path_idx = a_star((1, 0), (4, 3), self, occupancy_cost_factor=10)
        for i in range(len(path_idx)):
            print(f"{path[i][0] : >+2.1f}, {path[i][1]: >+2.1f}  ({path_idx[i]})  ({self.is_occupied_idx(path_idx[i])})")

    def shape(self) -> Tuple[int, int]:
        return self.grid.shape

    def is_occupied_xy(self, x, y) -> bool:
        (i, j) = self.get_index_from_coordinates(x, y)
        return self.grid[i][j]

    def is_occupied_idx(self, ij: Tuple[int, int]) -> bool:
        return self.grid[ij]

    def is_inside_idx(self, ij: Tuple[int, int]) -> bool:
        return 0 <= ij[0] < self.grid.shape[0] and 0 <= ij[1] < self.grid.shape[1]

    def get_index_from_coordinates(self, x, y) -> Tuple[int, int]:
        i = int((y - self.origin.y) / self.resolution)
        j = int((x - self.origin.x) / self.resolution)
        return (i, j)

    def get_coordinates_from_index(self, ij: Tuple[int, int]):
        i, j = ij
        x = j * self.resolution + self.origin.x
        y = i * self.resolution + self.origin.y
        return (x, y)
