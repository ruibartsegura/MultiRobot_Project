#!/usr/bin/env python3

import rospy
import math
import numpy as np

from RuleNode import RuleNode
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import OccupancyGrid
from reynolds_rules.msg import VectorArray

from navigation.GridMap import GridMap


class Nav2PointRuleNode(RuleNode):
    def __init__(self):
        super().__init__("nav2point")

        self.point = Point()
        self.point.x = rospy.get_param("~point_x", 0)
        self.point.y = rospy.get_param("~point_y", 0)
        self.threshold_vel = rospy.get_param("~threshold_vel", 2)
        self.grid_resolution = rospy.get_param("~grid_resolution", 10)

        print(f"  point_x: {self.point.x}")
        print(f"  point_y: {self.point.y}")

        self.map: OccupancyGrid | None = None
        rospy.Subscriber("map", OccupancyGrid, self.callback_map)

        self.startTimer()

    def callback_map(self, msg: OccupancyGrid):
        self.map = msg
        self.grid = GridMap(msg, self.grid_resolution)

    # Return vector from point 1 to 2
    # Limits vector to a threshold
    def calc_vector(self, point1, point2):
        vector = Vector3()

        vector.x = point2.x - point1.x
        vector.y = point2.y - point1.y

        vector_lenght = math.sqrt(vector.x * vector.x + vector.y * vector.y)
        if vector_lenght > self.threshold_vel:
            factor = self.threshold_vel / vector_lenght
            vector.x *= factor
            vector.y *= factor

        return vector

    # Make and publish array of velocity vector to given point
    def control_cycle(self, _):
        # Check if there is a new target point
        self.point.x = rospy.get_param("~point_x", self.point.x)
        self.point.y = rospy.get_param("~point_y", self.point.y)

        nav2point_vectors = VectorArray()

        for robot in self.robots:
            nav2point_vector = self.calc_vector(robot.pose.pose.position, self.point)
            nav2point_vectors.vectors.append(nav2point_vector)

        self.pub.publish(nav2point_vectors)


if __name__ == "__main__":
    rospy.init_node("nav2point_rule")
    node = Nav2PointRuleNode()
    rospy.spin()
