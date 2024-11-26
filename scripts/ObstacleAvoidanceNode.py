#!/usr/bin/env python3

import rospy
import itertools
import numpy as np

from RuleNode import RuleNode
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Vector3, Pose
from reynolds_rules.msg import VectorArray
from typing import List, Tuple


Position2D = Tuple[float, float]

ANGULAR_STEPSIZE = np.deg2rad(2)


class ObstacleAvoidanceNode(RuleNode):
    def __init__(self):
        super().__init__("obstacle_avoidance", 10)

        # maximum range of perception
        self.view_range = rospy.get_param("~view_range", 0.5)
        # view_angle [degrees]: angular range of perception (half-width of the perception cone)
        self.view_angle = rospy.get_param("~view_angle", 60)
        # view_split [degrees]: angle that splits the perception cone on each side
        self.view_split = rospy.get_param("~view_split", 0)
        # example: view_angle:=60 view_split:=15
        #   -> angular ranges [-75:-15] and [15:75] are observed

        print("Starting the ObstacleAvoidance node.")
        print(f"view_range: {self.view_range}")
        print(f"view_angle: {self.view_angle : >3} deg")
        print(f"view_split: {self.view_split : >3} deg")

        self.map: OccupancyGrid | None = None
        rospy.Subscriber("map", OccupancyGrid, self.callback_map)

    def callback_map(self, msg: OccupancyGrid):
        self.map = msg

    # implement abstract method
    def control_cycle(self, _):
        data = VectorArray()
        data.vectors = [
            self.sum_weighted_repellent_vectors(i) for i in range(self.n_robots)
        ]
        self.pub.publish(data)

    def sum_weighted_repellent_vectors(self, robot_index: int) -> Vector3:
        """
        Find obstacles in the view range and compute the weighted sum of the distance vectors.
        The weight for each distance vector is (-1 / distance) to simulate a repellent force.
        """
        if not self.robots[robot_index]:
            return Vector3()

        robot_pose = self.robots[robot_index].pose.pose
        obstacles = self.find_obstacles(robot_pose)
        if len(obstacles) == 0:
            return Vector3()

        robot_position = np.array((robot_pose.position.x, robot_pose.position.y))

        v = np.array((0.0, 0.0))
        for obstacle in obstacles:
            opposite_vector = robot_position - np.array(obstacle)
            distance = np.linalg.norm(opposite_vector)
            if distance != 0:
                v += opposite_vector / distance

        v = v / len(obstacles)

        v3 = Vector3()
        v3.x = v[0]
        v3.y = v[1]
        return v3

    def find_obstacles(self, robot_pose: Pose) -> List[Position2D]:
        """
        Return the position of obstacles around the robot_position
        Current implementation: square with size (2*self.view_range)^2
        """
        if not self.map:
            return []

        robot_x = robot_pose.position.x
        robot_y = robot_pose.position.y
        robot_q = robot_pose.orientation

        robot_yaw = 2 * np.arctan2(robot_q.z, robot_q.w)

        # look left and right
        min_angle = np.deg2rad(self.view_split)
        max_angle = np.deg2rad(self.view_split + self.view_angle)
        s = ANGULAR_STEPSIZE
        angular_range = itertools.chain(
            range(int(-max_angle / s), int(-min_angle / s)),  # left
            range(int(min_angle / s), int(max_angle / s)),  # right
        )

        radial_step_size = self.map.info.resolution
        n_radial = int(self.view_range / radial_step_size)

        obstacles = []
        for a in angular_range:
            alpha = robot_yaw + a * ANGULAR_STEPSIZE
            cos_alpha = np.cos(alpha)
            sin_alpha = np.sin(alpha)
            for r in range(1, n_radial):
                distance = r * radial_step_size
                p = (robot_x + distance * cos_alpha, robot_y + distance * sin_alpha)
                if self.map_lookup(p):
                    obstacles.append(p)
        return obstacles

    def map_lookup(self, pos: Position2D) -> bool:
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


if __name__ == "__main__":
    rospy.init_node("obstacle_avoidance_rule")
    node = ObstacleAvoidanceNode()
    rospy.spin()
