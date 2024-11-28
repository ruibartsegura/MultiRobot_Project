#!/usr/bin/env python3

import rospy
import math

from RuleNode import RuleNode

from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from reynolds_rules.msg import VectorArray


# Implements cohesion algorithm to the swarm
class CohesionRuleNode(RuleNode):
    def __init__(self):
        super().__init__("cohesion", 20)

    def get_distance(self, pos1, pos2):
        x = pos1.x - pos2.x
        y = pos1.y - pos2.y
        return math.sqrt(x * x + y * y)

    # Returns average position between all robots positions in the neighbor
    def calc_average_pos(self, robots):
        average_pos = Point()

        for robot in robots:
            average_pos.x += robot.pose.pose.position.x
            average_pos.y += robot.pose.pose.position.y

        average_pos.x = average_pos.x / len(robots)
        average_pos.y = average_pos.y / len(robots)

        return average_pos

    # Return vector from point 1 to 2 with adecuate multiplier
    def calc_vector(self, position, name):
        vector = Vector3()
        robots = [] 
        for robot in self.robots:
            if robot.header.frame_id != name:
                dist = self.get_distance(position, robot.pose.pose.position)
                if dist > 0.0 and dist < 0.3:
                    robots.append(robot)

        if len(robots) == 0:
            vector.x = 0.0
            vector.y = 0.0
        else:
            posi = self.calc_average_pos(robots)
            vector.x = self.multiplier * (posi.x - position.x)
            vector.y = self.multiplier * (posi.y - position.y)

        return vector

    # Publish list with all vectors to average position to achieve cohesion
    def control_cycle(self, _):
        cohesion_vectors = VectorArray()

        for robot in self.robots:
            cohesion_vector = self.calc_vector(robot.pose.pose.position, robot.header.frame_id)
            cohesion_vectors.vectors.append(cohesion_vector)

        self.pub.publish(cohesion_vectors)


if __name__ == "__main__":
    rospy.init_node("cohesion_rule")
    node = CohesionRuleNode()
    rospy.spin()
