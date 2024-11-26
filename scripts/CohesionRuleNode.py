#!/usr/bin/env python3

import rospy
from RuleNode import RuleNode

from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from reynolds_rules.msg import VectorArray


# Implements cohesion algorithm to the swarm
class CohesionRuleNode(RuleNode):
    def __init__(self):
        super().__init__("cohesion", 20)

    # Returns average position between all robots positions
    def calc_average_pos(self):
        average_pos = Point()

        for robot in self.robots:
            average_pos.x += robot.pose.pose.position.x
            average_pos.y += robot.pose.pose.position.y

        average_pos.x = average_pos.x / self.n_robots
        average_pos.y = average_pos.y / self.n_robots

        return average_pos

    # Return vector from point 1 to 2 with adecuate multiplier
    def calc_vector(self, point1, point2):
        vector = Vector3()

        vector.x = self.multiplier * (point2.x - point1.x)
        vector.y = self.multiplier * (point2.y - point1.y)

        return vector

    # Publish list with all vectors to average position to achieve cohesion
    def control_cycle(self, _):
        cohesion_pos = self.calc_average_pos()
        cohesion_vectors = VectorArray()

        for robot in self.robots:
            cohesion_vector = self.calc_vector(robot.pose.pose.position, cohesion_pos)
            cohesion_vectors.vectors.append(cohesion_vector)

        self.pub.publish(cohesion_vectors)


if __name__ == "__main__":
    rospy.init_node("cohesion_rule")
    node = CohesionRuleNode()
    rospy.spin()
