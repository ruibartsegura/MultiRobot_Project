#!/usr/bin/env python3

import rospy
import numpy as np

from RuleNode import RuleNode
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Vector3
from reynolds_rules.msg import ArrayVectors
from typing import List, Tuple


Position = Tuple[float, float]
Vector = Tuple[float, float]


def to_vector3(v: Vector) -> Vector3:
    result = Vector3()
    result.x = v[0]
    result.y = v[1]
    return result


class ObstacleAvoidanceNode(RuleNode):
    def __init__(self):
        super().__init__("obstacle_avoidance", 10)

        self.view_range = rospy.get_param("~view_range", 0.5)
        self.view_angle = rospy.get_param("~view_angle", 90)

        self.map: OccupancyGrid | None = None
        rospy.Subscriber("map", OccupancyGrid, self.callback_map)

    def callback_map(self, msg: OccupancyGrid):
        self.map = msg

    # implement abstract method
    def control_cycle(self, _):
        obstacle_avoidance_vectors = self.obstacle_avoidance_rule()

        data = ArrayVectors()
        data.vectors = [to_vector3(v) for v in obstacle_avoidance_vectors]
        self.pub.publish(data)

    def map_lookup(self, pos: Position) -> bool:
        """
        Return True if the position is within the map and there is an obstacle, else False
        """
        if not self.map:
            return False
        i = int((pos[1] - self.map.info.origin.position.y) / self.map.info.resolution)
        j = int((pos[0] - self.map.info.origin.position.x) / self.map.info.resolution)
        index = self.map.info.width * i + j
        if not 0 <= index < len(self.map.data):
            return False
        return self.map.data[index] == 100

    def find_obstacles(self, robot_position: Position) -> List[Position]:
        """
        Return the position of obstacles around the robot_position
        Current implementation: square with size (2*self.view_range)^2
        """
        if not self.map:
            return []

        step_size = self.map.info.resolution
        n = int(self.view_range / step_size)

        obstacles: List[Position] = []
        for i in range(-n, n + 1):
            for j in range(-n, n + 1):
                pos = (
                    robot_position[0] + j * step_size,
                    robot_position[1] + i * step_size,
                )
                if self.map_lookup(pos):
                    obstacles.append(pos)
        return obstacles

    def sum_weighted_repellent_vectors(
        self, robot_position: Position, obstacles: List[Position]
    ) -> Vector:
        """
        Calculate a repellent force from each obstacle and return the sum of all of those vectors
        """
        vec = (0, 0)
        for obstacle in obstacles:
            opposite_vector = np.array(robot_position) - np.array(obstacle)
            distance = np.linalg.norm(opposite_vector)
            if distance != 0:
                vec += opposite_vector / distance
        return vec

    def obstacle_avoidance_rule(self) -> List[Vector]:
        """
        Compute the sum of repellent vectors for each robot
        """
        vectors: list[Vector] = [(0, 0)] * self.n_robots
        for i in range(self.n_robots):
            robot = self.robots[i]
            if robot:
                pos = (robot.pose.pose.position.x, robot.pose.pose.position.y)
                obstacles = self.find_obstacles(pos)
                vectors[i] = self.sum_weighted_repellent_vectors(pos, obstacles)
        return vectors


if __name__ == "__main__":
    rospy.init_node("obstacle_avoidance_rule")
    node = ObstacleAvoidanceNode()
    rospy.spin()
