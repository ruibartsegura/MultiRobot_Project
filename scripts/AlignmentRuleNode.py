#!/usr/bin/env python3

import rospy
from RuleNode import RuleNode

from geometry_msgs.msg import Vector3
from reynolds_rules.msg import VectorArray


# Implements alignment rule to the swarm
class AlignmentRuleNode(RuleNode):
    def __init__(self):
        super().__init__("alignment")

    # Compute average velocity of all robots
    def calc_average_velocity(self):
        avg_velocity = Vector3()

        for robot in self.robots:
            avg_velocity.x += robot.twist.twist.linear.x
            avg_velocity.y += robot.twist.twist.linear.y

        avg_velocity.x /= self.n_robots
        avg_velocity.y /= self.n_robots

        return avg_velocity

    # Publish list with all vectors for robots to align with average velocity
    def control_cycle(self, _):
        avg_velocity = self.calc_average_velocity()

        alignment_vectors = VectorArray(
            vectors=[
                Vector3(
                    x=avg_velocity.x - robot.twist.twist.linear.x,
                    y=avg_velocity.y - robot.twist.twist.linear.y,
                )
                for robot in self.robots
            ]
        )

        self.pub.publish(alignment_vectors)


if __name__ == "__main__":
    rospy.init_node("alignment_rule")
    node = AlignmentRuleNode()
    rospy.spin()
