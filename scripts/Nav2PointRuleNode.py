#!/usr/bin/env python3

import rospy
import math
from RuleNode import RuleNode

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from typing import List, Tuple
from reynolds_rules.msg import VectorArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3

Position2D = Tuple[float, float]


# Return distance between two points
def calc_distance(pos1, pos2):
    x = pos1.x - pos2.x
    y = pos1.y - pos2.y

    return math.sqrt(x * x + y * y)


class Nav2PointRuleNode(RuleNode):
    def __init__(self):
        super().__init__("nav2point")

        self.point = Point()
        self.point.x = rospy.get_param("~point_x", 0)
        self.point.y = rospy.get_param("~point_y", 0)

        print(f"  point_x: {self.point.x}")
        print(f"  point_y: {self.point.y}")

        self.waypoints = []
        for x in range(-4, 5, 2):
            for y in range(-4, 5, 2):
                wp = Point()
                wp.x = x
                wp.y = y
                self.waypoints.append(wp)

        self.map: OccupancyGrid | None = None
        rospy.Subscriber("map", OccupancyGrid, self.callback_map)

    def callback_map(self, msg: OccupancyGrid):
        if self.map is None:  # Solo procesar el primer mapa recibido
            self.map = msg
            self.check_paths_between_waypoints()

    def check_paths_between_waypoints(self):
        for wp in self.waypoints:
            neighbors = self.find_neighbors(self.waypoints, wp)
            for neighbor in neighbors:
                start = (wp.x, wp.y)
                end = (neighbor.x, neighbor.y)
                if self.is_path_clear(start, end):
                    rospy.loginfo(f"Camino libre entre {start} y {end}.")
                else:
                    rospy.logwarn(f"Obstáculo entre {start} y {end}.")

    def find_neighbors(self, waypoints: List[Point], current_wp: Point, step=2) -> List[Point]:
        neighbors = []
        for wp in waypoints:
            if (abs(wp.x - current_wp.x) == step and wp.y == current_wp.y) or \
               (abs(wp.y - current_wp.y) == step and wp.x == current_wp.x):
                neighbors.append(wp)
        return neighbors

    def is_path_clear(self, start: Position2D, end: Position2D) -> bool:
        if not self.map:
            return False

        start_i = int((start[1] - self.map.info.origin.position.y) / self.map.info.resolution)
        start_j = int((start[0] - self.map.info.origin.position.x) / self.map.info.resolution)
        end_i = int((end[1] - self.map.info.origin.position.y) / self.map.info.resolution)
        end_j = int((end[0] - self.map.info.origin.position.x) / self.map.info.resolution)

        delta_i = abs(end_i - start_i)
        delta_j = abs(end_j - start_j)
        sign_i = 1 if end_i > start_i else -1
        sign_j = 1 if end_j > start_j else -1

        error = delta_i - delta_j
        current_i, current_j = start_i, start_j

        while True:
            index = self.map.info.width * current_i + current_j
            if not (0 <= index < len(self.map.data)):
                return False
            if self.map.data[index] == 100:
                return False

            if current_i == end_i and current_j == end_j:
                break

            error2 = 2 * error
            if error2 > -delta_j:
                error -= delta_j
                current_i += sign_i
            if error2 < delta_i:
                error += delta_i
                current_j += sign_j

        return True

    # Return vector from point 1 to 2
    def calc_vector(self, point1, point2):
        vector = Vector3()

        vector.x = point2.x - point1.x
        vector.y = point2.y - point1.y

        return vector

    # Make and publish array of velocity vector to given point
    def control_cycle(self, _):
        # Check if there is a new target point
        self.point.x = rospy.get_param("~point_x", self.point.x)
        self.point.y = rospy.get_param("~point_y", self.point.y)

        # Get the closer waypoint to the position of the robots
        start = Point()
        for robot in self.robots:
            start.x += robot.pose.pose.position.x
            start.y += robot.pose.pose.position.y
        
        start.x = start.x / self.n_robots
        start.y = start.y / self.n_robots

        dist = None
        closer_2_start = Point()
        for wp in self.waypoints:
            if (dist == None or calc_distance(start, wp) < dist):
                dist = calc_distance(start, wp)
                closer_2_start = wp
        nav2point_vectors = VectorArray()
        print("closer_2_start ", closer_2_start)
        
        dist = None
        closer_2_target = Point()
        for wp in self.waypoints:
            if (dist == None or calc_distance(self.point, wp) < dist):
                dist = calc_distance(start, wp)
                closer_2_target = wp
        nav2point_vectors = VectorArray()
        print("closer_2_target ", closer_2_target)

        for robot in self.robots:
            nav2point_vector = self.calc_vector(robot.pose.pose.position, self.point)
            nav2point_vectors.vectors.append(nav2point_vector)

        self.pub.publish(nav2point_vectors)

if __name__ == "__main__":
    rospy.init_node("nav2point_rule")
    node = Nav2PointRuleNode()
    rospy.spin()
