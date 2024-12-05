#!/usr/bin/env python3

import rospy
import math

from geometry_msgs.msg import Twist, Vector3, Quaternion
from reynolds_rules.msg import VectorArray  # Import the custom message
from RuleNode import RuleNode


def calc_length(vector, weight=1):
    # Return scalated vector module
    x = weight * vector.x
    y = weight * vector.y
    return math.sqrt(x * x + y * y)


def wrap_to_pi(angle):
    # Normalizar al rango [-π, π]
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi

    return angle


class ReynoldsRulesNode(RuleNode):
    def __init__(self):
        super().__init__("null")

        # Get threshold for priorities
        self.linear_mult = rospy.get_param("~linear_mult", 1.0)
        self.angular_mult = rospy.get_param("~angular_mult", 2.0)
        self.max_linear_vel = rospy.get_param("~max_linear_vel", 1.0)
        self.min_linear_vel = rospy.get_param("~min_linear_vel", 0)
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
        print(f"  max_linear_vel: {self.max_linear_vel: >.1f}")

        # Make a tuple with the correct namespace of the robots
        # I use a tuple to avoid having problems later if by mistake the list is changed
        self.robot_names = []
        for num in range(self.n_robots):
            name = "robot_" + str(num)
            self.robot_names.append(name)

        self.robot_names = tuple(self.robot_names)

        # Make a dictionary, it will match each namespace with the corresponding publisher
        # To use the publisher self.publishers[self.robot_names[nº]].publish(TWIST)
        self.publishers = {}
        for name in self.robot_names:
            topic = "/" + name + "/cmd_vel"
            self.publishers[name] = rospy.Publisher(topic, Twist, queue_size=1)

        self.init_rule_vectors()
        self.init_rule_subscribers()

    def init_rule_vectors(self):
        # Variables to store the value of the rule vectors
        self.separation_vectors = [Vector3() for _ in range(self.n_robots)]
        self.cohesion_vectors = [Vector3() for _ in range(self.n_robots)]
        self.nav2point_vectors = [Vector3() for _ in range(self.n_robots)]
        self.obstacle_avoidance_vectors = [Vector3() for _ in range(self.n_robots)]
        self.alignment_vectors = [Vector3() for _ in range(self.n_robots)]

    def init_rule_subscribers(self):
        # Subscribers to the rules topics
        rospy.Subscriber("/separation_vectors", VectorArray, self.separation_callback)
        rospy.Subscriber("/cohesion_vectors", VectorArray, self.cohesion_callback)
        rospy.Subscriber("/nav2point_vectors", VectorArray, self.nav2point_callback)
        rospy.Subscriber("/alignment_vectors", VectorArray, self.alignment_callback)
        rospy.Subscriber(
            "/obstacle_avoidance_vectors", VectorArray, self.obstacle_avoidance_callback
        )

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

    # Makes module of the vector the linear x velocity component
    # and the angle to where the vector point as angular velocity
    def vector2twist(self, vector, current_orientation: Quaternion):
        twist = Twist()

        current_angle = 2 * math.atan2(current_orientation.z, current_orientation.w)
        error_angle = wrap_to_pi(math.atan2(vector.y, vector.x) - current_angle)

        twist.angular.z = self.angular_mult * error_angle

        # reduce linear speed exponentially when turning to shrink the turning radius
        linear_vel = (
            self.linear_mult
            * calc_length(vector)
            * math.exp(-abs(twist.angular.z)) + self.min_linear_vel
        )

        twist.linear.x = min(linear_vel, self.max_linear_vel)
        return twist

    # Summ vectors of each element of the swarm and publish them to its vel topic
    def control_cycle(self, _):
        # Vectors and weights list in prioritiy order
        rules = [
            self.separation_vectors,
            self.nav2point_vectors,
            self.obstacle_avoidance_vectors,
            self.alignment_vectors,
            self.cohesion_vectors,
        ]

        weights = [
            self.separation_weight,
            self.nav2point_weight,
            self.obstacle_avoidance_weight,
            self.cohesion_weight,
            self.alignment_weight,
        ]

        for i in range(self.n_robots):
            total_vector = Vector3()

            # Add each rule to total vector, until total vector exced threshold
            for rule, weight in zip(rules, weights):
                new_vector = Vector3()
                new_vector.x = weight * rule[i].x
                new_vector.y = weight * rule[i].y

                total_vector.x += new_vector.x
                total_vector.y += new_vector.y
                if calc_length(new_vector) > self.max_linear_vel:
                    break

            vel = self.vector2twist(total_vector, self.robots[i].pose.pose.orientation)
            self.publishers[self.robot_names[i]].publish(vel)


if __name__ == "__main__":
    rospy.init_node("reynolds_rules_node")
    node = ReynoldsRulesNode()
    rospy.spin()
