#!/usr/bin/env python3

import rospy
import math

from RuleNode import RuleNode

from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from reynolds_rules.msg import VectorArray


# Return distance between two points
def calc_distance(pos1, pos2):
    x = pos1.x - pos2.x
    y = pos1.y - pos2.y

    return math.sqrt(x * x + y * y)


# Implements cohesion algorithm to the swarm
class CohesionRuleNode(RuleNode):
    def __init__(self):
        super().__init__("cohesion")
        self.cohesion_range = rospy.get_param("~cohesion_range", 0.3)

    # Returns average position between all robots positions in the neighbor
    def calc_average_pos(self, positions):
        average_pos = Point()

        for position in positions:
            average_pos.x += position.pose.pose.position.x
            average_pos.y += position.pose.pose.position.y

        average_pos.x = average_pos.x / len(positions)
        average_pos.y = average_pos.y / len(positions)

        return average_pos

    # Returns cohesion vector of each robots using only neighbors robots
    def calc_cohesion_vector(self, robot_pos):
        vector = Vector3()
        neighbors = []

        # Make a list with all robots poses in the range of cohesion
        for robot in self.robots:
            dist = calc_distance(robot_pos, robot.pose.pose.position)

            if dist < self.cohesion_range:
                neighbors.append(robot)

        # Calculate vector from robot_pos to average_pos of neighbors
        average_pos = self.calc_average_pos(neighbors)
        vector.x = average_pos.x - robot_pos.x
        vector.y = average_pos.y - robot_pos.y

        return vector

    # Publish list with all vectors to average position to achieve cohesion
    def control_cycle(self, _):
        cohesion_vectors = VectorArray()

        for robot in self.robots:
            cohesion_vector = self.calc_cohesion_vector(robot.pose.pose.position)
            cohesion_vectors.vectors.append(cohesion_vector)

        self.pub.publish(cohesion_vectors)


if __name__ == "__main__":
    rospy.init_node("cohesion_rule")
    node = CohesionRuleNode()
    rospy.spin()
