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
        self.threshold_vel = rospy.get_param("~threshold_vel", 2)

        print(f"  point_x: {self.point.x}")
        print(f"  point_y: {self.point.y}")

        # Previos target point to avoid doing extra calculations
        self.prev_point = Point()
        self.prev_point.x = None
        self.prev_point.y = None

        # List of waypoints
        self.waypoints = []
        for x in range(-4, 5, 2):
            for y in range(-4, 5, 2):
                wp = Point()
                wp.x = x
                wp.y = y
                self.waypoints.append(wp)

        # Path of waypoints that the swarm will follow
        self.path = None

        # Threshold to detect if the swarm has arrive to a waypoint
        self.distance_threshold = 0.3

        self.map: OccupancyGrid | None = None
        rospy.Subscriber("map", OccupancyGrid, self.callback_map)

        self.startTimer()

    def callback_map(self, msg: OccupancyGrid):
        if self.map is None:  # Just process the first map receive
            self.map = msg
            self.check_paths_between_waypoints()

    def check_paths_between_waypoints(self):
        for wp in self.waypoints:
            neighbors = self.find_neighbors(self.waypoints, wp)
            for neighbor in neighbors:
                start = (wp.x, wp.y)
                end = (neighbor.x, neighbor.y)
                if self.is_path_clear(start, end):
                    rospy.loginfo(f"Free way between {start} and {end}.")
                else:
                    rospy.logwarn(f"Obstacle between {start} and {end}.")

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

    def find_path_through_waypoints(self, start: Point, target: Point) -> List[Point]:
        """
        Find a route from `start` to `target` using BFS only between the waypoints,
        ensuring that paths between neighbors are clear.
        """
        start_pos = (start.x, start.y)
        target_pos = (target.x, target.y)
        
        queue = [(start_pos, [start_pos])]
        visited = set()
        visited.add(start_pos)

        waypoint_positions = {(wp.x, wp.y): wp for wp in self.waypoints}

        while queue:
            current_pos, path = queue.pop(0)

            # Check if we are already in the target
            if current_pos == target_pos:
                return [waypoint_positions[pos] for pos in path]

            current_wp = waypoint_positions[current_pos]
            neighbors = self.find_neighbors(self.waypoints, current_wp)

            # Check posible paths with the neighbors
            for neighbor in neighbors:
                neighbor_pos = (neighbor.x, neighbor.y)
                if neighbor_pos not in visited and self.is_path_clear(current_pos, neighbor_pos):
                    visited.add(neighbor_pos)
                    queue.append((neighbor_pos, path + [neighbor_pos]))

        rospy.logwarn("No available route found..")
        return []

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

    def find_path_through_waypoints(self, start: Point, target: Point) -> List[Point]:
        """
        Find a route from `start` to `target` using BFS only between the waypoints,
        ensuring that paths between neighbors are clear.
        """
        start_pos = (start.x, start.y)
        target_pos = (target.x, target.y)
        
        queue = [(start_pos, [start_pos])]
        visited = set()
        visited.add(start_pos)

        waypoint_positions = {(wp.x, wp.y): wp for wp in self.waypoints}

        while queue:
            current_pos, path = queue.pop(0)

            # Check if we are currentrly in the target
            if current_pos == target_pos:
                return [waypoint_positions[pos] for pos in path]

            current_wp = waypoint_positions[current_pos]
            neighbors = self.find_neighbors(self.waypoints, current_wp)

            # Check posible paths with the neighbors
            for neighbor in neighbors:
                neighbor_pos = (neighbor.x, neighbor.y)
                if neighbor_pos not in visited and self.is_path_clear(current_pos, neighbor_pos):
                    visited.add(neighbor_pos)
                    queue.append((neighbor_pos, path + [neighbor_pos]))

        rospy.logwarn("No available route found..")
        return []

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

        if (self.point.x != self.prev_point.x or self.point.y != self.prev_point.y):

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
            
            dist = None
            closer_2_target = Point()
            for wp in self.waypoints:
            
                if (dist == None or calc_distance(self.point, wp) < dist):
                    dist = calc_distance(self.point, wp)
                    closer_2_target = wp

            self.path = self.find_path_through_waypoints(closer_2_start, closer_2_target)

            self.path.append(self.point)
            if self.path:
                rospy.loginfo(f"Route found: {[f'({p.x}, {p.y})' for p in self.path]}")
            else:
                rospy.logwarn("No available route found.")

        # Check if the swarm arrive to the waypoint
        average_position = Point()
        for robot in self.robots:
            average_position.x += robot.pose.pose.position.x
            average_position.y += robot.pose.pose.position.y
        
        average_position.x = average_position.x / self.n_robots
        average_position.y = average_position.y / self.n_robots

        # Delete the waypoint from the path if the swarm has arrived
        if (calc_distance(average_position, self.path[0]) < self.distance_threshold):
            if (self.path[0] != self.point):
                self.path.pop(0)
        nav2point_vectors = VectorArray()
        
        dist = None
        closer_2_target = Point()
        for wp in self.waypoints:
            if (dist == None or calc_distance(self.point, wp) < dist):
                dist = calc_distance(closer_2_target, wp)
                closer_2_target = wp
        nav2point_vectors = VectorArray()

        for robot in self.robots:
            nav2point_vector = self.calc_vector(robot.pose.pose.position, self.path[0])
            nav2point_vectors.vectors.append(nav2point_vector)

        self.prev_point.x = self.point.x
        self.prev_point.y = self.point.y

        self.pub.publish(nav2point_vectors)

if __name__ == "__main__":
    rospy.init_node("nav2point_rule")
    node = Nav2PointRuleNode()
    rospy.spin()
