#!/usr/bin/env python3

import rospy

from RuleNode import RuleNode
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from reynolds_rules.msg import VectorArray  # Import the custom message


class ReynoldsRulesNode(RuleNode):
    def __init__(self):
        self.n_robots = rospy.get_param("~number_robots", 10)

        # Create subscribers to the rules topics
        rules = ["cohesion", "separation", "nav2point", "obstacle_avoidance"]

        for rule in rules:
            rule_topic = "/" + rule + "_vectors"
            rospy.Subscriber(rule_topic, VectorArray, self.vectors_callback)

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

        self.map: OccupancyGrid | None = None
        rospy.Subscriber("map", OccupancyGrid, self.callback_map)

    # Save vectors in vectors callback in it correspondat atribute
    def vectors_callback(self, data):
        match data.id:
            case "cohesion":
                self.cohesion_vectors = data.vectors
            case "separation":
                self.separation_vectors = data.vectors
            case "obstacle_avoidance":
                self.obstacle_avoindace_vectors = data.vectors
            case "nav2point":
                self.nav2point_vectors = data.vectors
            # case "allingment":
            #     self.allingment_vectors = data.vectors

    def callback_map(self, msg: OccupancyGrid):
        self.map = msg

    def map_lookup(self, xy) -> bool:
        i = int((xy[1] - self.map.info.origin.position.y) / self.map.info.resolution)
        j = int((xy[0] - self.map.info.origin.position.x) / self.map.info.resolution)
        index = self.map.info.width * i + j
        if not 0 <= index < len(self.map.data):
            return False
        return self.map.data[index] == 100

    def control_cycle(self, _):
        for i in range(self.n_robots):
            vel = Twist()
            vel.linear.x = (
                self.separation_vectors[i].x
                + self.cohesion_vectors[i].x
                + self.obstacle_avoindace_vectors[i].x
                + self.nav2point_vectors[i].x
            )

            vel.linear.y = (
                self.separation_vectors[i].y
                + self.cohesion_vectors[i].y
                + self.obstacle_avoindace_vectors[i].y
                + self.nav2point_vectors[i].y
            )

            print(self.robot_names[i])
            print(f"x {vel.linear.x}, y {vel.linear.y}")
            self.publishers[self.robot_names[i]].publish(vel)


if __name__ == "__main__":
    rospy.init_node("reynolds_rules_node")
    node = ReynoldsRulesNode()
    rospy.spin()
