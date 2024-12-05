#!/usr/bin/env python3

import rospy
import math
import numpy as np

from RuleNode import RuleNode
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import OccupancyGrid
from reynolds_rules.msg import VectorArray

from navigation.GridMap import GridMap


def distance(p1: Point, p2: Point) -> float:
    x = p2.x - p1.x
    y = p2.y - p1.y
    return math.sqrt(x * x + y * y)


class Nav2PointRuleNode(RuleNode):
    def __init__(self):
        super().__init__("nav2point")

        self.point = Point()
        self.point.x = rospy.get_param("~point_x", 0)
        self.point.y = rospy.get_param("~point_y", 0)
        self.threshold_vel = rospy.get_param("~threshold_vel", 2)
        self.grid_resolution = rospy.get_param("~grid_resolution", 10)
        self.distance_to_waypoint = rospy.get_param("~distance_to_waypoint", 1.0)

        print(f"  point_x: {self.point.x}")
        print(f"  point_y: {self.point.y}")

        self.paths = [[]] * self.n_robots
        self.map: OccupancyGrid | None = None
        rospy.Subscriber("map", OccupancyGrid, self.callback_map)

        self.startTimer()

    def callback_map(self, msg: OccupancyGrid):
        self.map = msg
        self.grid = GridMap(msg, self.grid_resolution)
        self.compute_paths()

    def compute_paths(self):
        if not self.map:
            return

        positions = [robot.pose.pose.position for robot in self.robots]
        goal = self.point
        self.paths = [self.grid.a_star(pos, goal) for pos in positions]

    # Return vector from point 1 to 2
    # Limits vector to a threshold
    def calc_vector(self, point1, point2):
        vector = Vector3()

        vector.x = point2.x - point1.x
        vector.y = point2.y - point1.y

        vector_length = math.sqrt(vector.x * vector.x + vector.y * vector.y)
        if vector_length > self.threshold_vel:
            factor = self.threshold_vel / vector_length
            vector.x *= factor
            vector.y *= factor

        return vector

    def vector_to_next_waypoint(self, robot_index) -> Vector3:
        if len(self.paths[robot_index]) == 0:
            return Vector3()

        robot_position = self.robots[robot_index].pose.pose.position

        # if the robot reached the waypoint, remove the waypoint unless it is the final goal
        if len(self.paths[robot_index]) != 1 and (
            distance(robot_position, self.paths[robot_index][0])
            < self.distance_to_waypoint
        ):
            self.paths[robot_index] = self.paths[robot_index][1:]

        return self.calc_vector(robot_position, self.paths[robot_index][0])

    # Make and publish array of velocity vector to given point
    def control_cycle(self, _):
        # Check if there is a new target point
        x = rospy.get_param("~point_x", self.point.x)
        y = rospy.get_param("~point_y", self.point.y)
        if x != self.point.x or y != self.point.y:
            self.point.x = x
            self.point.y = y
            self.compute_paths()

        vectors = [self.vector_to_next_waypoint(i) for i in range(self.n_robots)]
        self.pub.publish(VectorArray(vectors=vectors))


if __name__ == "__main__":
    rospy.init_node("nav2point_rule")
    node = Nav2PointRuleNode()
    rospy.spin()
