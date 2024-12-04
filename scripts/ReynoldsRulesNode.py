#!/usr/bin/env python3

import rospy
import math

from geometry_msgs.msg import Twist, Vector3
from reynolds_rules.msg import VectorArray  # Import the custom message


def calc_length(weight, vector):
    x = weight * vector.x
    y = weight * vector.y
    return math.sqrt(x * x + y * y)


def add_rule(total_vector, total_length, max_length, rule_weight, rule_vector):
    """Adds the rule to total_vector so that it does not exceed max_length"""
    l = calc_length(rule_weight, rule_vector)
    multiplier = 1
    if total_length + l > max_length:
        # scale down the rule_vector so that the total_length will be the same as max_length
        t = min(max_length, total_length)  # use min to prevent a negative multiplier
        multiplier = (max_length - t) / (total_length + l)

    total_vector.x += multiplier * rule_weight * rule_vector.x
    total_vector.y += multiplier * rule_weight * rule_vector.y
    total_length += multiplier * l
    return (total_vector, total_length)


class ReynoldsRulesNode:
    def __init__(self):
        refresh_rate = rospy.get_param("/refresh_rate", 20)
        self.n_robots = rospy.get_param("/number_robots", 10)

        # Get threshold for priorities
        self.threshold_priorities = rospy.get_param("~threshold_priorities", 1.0)

        # Get and print weigths for each rule
        self.separation_weight = rospy.get_param("~separation_weight", 1.0)
        self.alignment_weight = rospy.get_param("~alignment_weight", 1.0)
        self.cohesion_weight = rospy.get_param("~cohesion_weight", 1.0)
        self.nav2point_weight = rospy.get_param("~nav2point_weight", 1.0)
        self.obstacle_avoidance_weight = rospy.get_param(
            "~obstacle_avoidance_weight", 1.0
        )

        print("Starting the reynolds_rules node.")
        print(f"  separation_weight: {self.separation_weight: >.1f}")
        print(f"  alignment_weight: {self.alignment_weight: >.1f}")
        print(f"  cohesion_weight: {self.cohesion_weight: >.1f}")
        print(f"  nav2point_weight: {self.nav2point_weight: >.1f}")
        print(f"  obstacle_avoidance_weight: {self.obstacle_avoidance_weight: >.1f}")
        print(f"  threshold_priorities: {self.threshold_priorities: >.1f}")

        # Variables to store the value of the rule vectors
        self.separation_vectors = [Vector3() for _ in range(self.n_robots)]
        self.cohesion_vectors = [Vector3() for _ in range(self.n_robots)]
        self.nav2point_vectors = [Vector3() for _ in range(self.n_robots)]
        self.obstacle_avoidance_vectors = [Vector3() for _ in range(self.n_robots)]
        self.alignment_vectors = [Vector3() for _ in range(self.n_robots)]

        # Subscribers to the rules topics
        rospy.Subscriber("/separation_vectors", VectorArray, self.separation_callback)
        rospy.Subscriber("/cohesion_vectors", VectorArray, self.cohesion_callback)
        rospy.Subscriber("/nav2point_vectors", VectorArray, self.nav2point_callback)
        rospy.Subscriber("/alignment_vectors", VectorArray, self.alignment_callback)
        rospy.Subscriber(
            "/obstacle_avoidance_vectors", VectorArray, self.obstacle_avoidance_callback
        )

        # Make a tuple with the correct namespace of the robots
        # I use a tuple to avoid having problems later if by mistake the list is changed
        self.robot_names = []
        name = "robot_"
        for num in range(self.n_robots):
            new_name = name + str(num)
            self.robot_names.append(new_name)
        self.robot_names = tuple(self.robot_names)

        # Make a dictionary, it will match each namespace with the corresponding publisher
        # To use the publisher self.publishers[self.robot_names[nº]].publish(TWIST)
        self.publishers = {}
        for name in self.robot_names:
            topic = "/" + name + "/cmd_vel"
            self.publishers[name] = rospy.Publisher(topic, Twist, queue_size=1)

        rospy.Timer(rospy.Duration(1 / refresh_rate), self.control_cycle)

    # Callback for each rule. Save vector list in class atribute
    def separation_callback(self, data):
        self.separation_vectors = data.vectors

    def cohesion_callback(self, data):
        self.cohesion_vectors = data.vectors

    def nav2point_callback(self, data):
        self.nav2point_vectors = data.vectors

    def obstacle_avoidance_callback(self, data):
        self.obstacle_avoidance_vectors = data.vectors

    def alignment_callback(self, data):
        self.alignment_vectors = data.vectors

    # Summ vectors of each element of the swarm and publish them to its vel topic
    def control_cycle(self, _):
        for i in range(self.n_robots):
            vector = Vector3()
            length = 0
            # 1st priority: separation
            (vector, length) = add_rule(
                total_vector=vector,
                total_length=length,
                max_length=self.threshold_priorities,
                rule_weight=self.separation_weight,
                rule_vector=self.separation_vectors[i],
            )
            # 2nd priority: obstacle avoidance
            (vector, length) = add_rule(
                total_vector=vector,
                total_length=length,
                max_length=self.threshold_priorities,
                rule_weight=self.obstacle_avoidance_weight,
                rule_vector=self.obstacle_avoidance_vectors[i],
            )
            # 3rd priority: cohesion
            (vector, length) = add_rule(
                total_vector=vector,
                total_length=length,
                max_length=self.threshold_priorities,
                rule_weight=self.cohesion_weight,
                rule_vector=self.cohesion_vectors[i],
            )
            # 4th priority: alignment
            (vector, length) = add_rule(
                total_vector=vector,
                total_length=length,
                max_length=self.threshold_priorities,
                rule_weight=self.alignment_weight,
                rule_vector=self.alignment_vectors[i],
            )
            # 5th priority: navigation
            (vector, length) = add_rule(
                total_vector=vector,
                total_length=length,
                max_length=self.threshold_priorities,
                rule_weight=self.nav2point_weight,
                rule_vector=self.nav2point_vectors[i],
            )

            self.publishers[self.robot_names[i]].publish(Twist(linear=vector))


if __name__ == "__main__":
    rospy.init_node("reynolds_rules_node")
    node = ReynoldsRulesNode()
    rospy.spin()
