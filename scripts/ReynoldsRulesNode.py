#!/usr/bin/env python3

import rospy
import tf2_ros
import math

from geometry_msgs.msg import Twist, Vector3
from reynolds_rules.msg import VectorArray  # Import the custom message


class ReynoldsRulesNode():
    def __init__(self):
        self.n_robots = rospy.get_param("~number_robots", 10)

        # Get and print weigths for each rule
        self.separation_weight = rospy.get_param("~separation_weight", 1.0)
        self.alignment_weight = rospy.get_param("~alignment_weight", 1.0)
        self.cohesion_weight = rospy.get_param("~cohesion_weight", 1.0)
        self.nav2point_weight = rospy.get_param("~nav2point_weight", 1.0)
        self.obstacle_avoidance_weight = rospy.get_param(
            "~obstacle_avoidance_weight", 3.0
        )

        print(f"separation_weight: {self.separation_weight: >.1f}")
        print(f"alignment_weight: {self.alignment_weight: >.1f}")
        print(f"cohesion_weight: {self.cohesion_weight: >.1f}")
        print(f"nav2point_weight: {self.nav2point_weight: >.1f}")
        # print(f"obstacle_avoidance_weight: {self.obstacle_avoidance_weight: >.1f}")

        # Variables to store the value of the rule vectors
        self.separation_vectors = [Vector3() for _ in range(10)]
        self.cohesion_vectors = [Vector3() for _ in range(10)]
        self.nav2point_vectors = [Vector3() for _ in range(10)]
        # self.obstacle_avoidance_vectors = [Vector3() for _ in range(10)]
        self.alignment_vectors = [Vector3() for _ in range(10)]

        # Subscribers to the rules topics
        rospy.Subscriber("/separation_vectors", VectorArray, self.separation_callback)
        rospy.Subscriber("/cohesion_vectors", VectorArray, self.cohesion_callback)
        rospy.Subscriber("/nav2point_vectors", VectorArray, self.nav2point_callback)
        rospy.Subscriber("/alignment_vectors", VectorArray, self.alignment_callback)
        # rospy.Subscriber(
        #     "/obstacle_avoidance_vectors", VectorArray, self.obstacle_avoidance_callback
        # )

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

        rospy.Timer(rospy.Duration(1 / 20), self.control_cycle)

        rospy.Timer(rospy.Duration(1 / 3), self.control_cycle)

    # Callback for each rule. Save vector list in class atribute
    def separation_callback(self, data):
        self.separation_vectors = data.vectors

    def cohesion_callback(self, data):
        self.cohesion_vectors = data.vectors

    def nav2point_callback(self, data):
        self.nav2point_vectors = data.vectors

    # def obstacle_avoidance_callback(self, data):
    #     self.obstacle_avoidance_vectors = data.vectors

    def alignment_callback(self, data):
        self.alignment_vectors = data.vectors

    def calc_length(self, weigth, vector):
        x = weigth * vector.x
        y = weigth * vector.y
        return math.sqrt(x * x + y * y)

    # Summ vectors of each element of the swarm and publish them to its vel topic
    def control_cycle(self, _):
        for i in range(self.n_robots):
            vel = Twist()
            total_amount = 0

            # Check separation first to avoid collision between robots
            total_amount += self.calc_length(self.separation_weight, self.separation_vectors[i])
            print(f"1 {total_amount}")
            if (total_amount <= 5):
                vel.linear.x += self.separation_weight * self.separation_vectors[i].x
                vel.linear.y += self.separation_weight * self.separation_vectors[i].y

            # Check avoid obstacle second to avoid collision with the enviroment
            # total_amount += self.calc_length(self.obstacle_avoidance_weight, self.obstacle_vectors[i])
            # if (total_amount <= 5):
            #     vel.linear.x += self.obstacle_avoidance_weight * self.obstacle_vectors[i].x
            #     vel.linear.y += self.obstacle_avoidance_weight * self.obstacle_vectors[i].y
            
            # Check avoid obstacle second to avoid collision with the enviroment
            total_amount += self.calc_length(self.cohesion_weight, self.cohesion_vectors[i])
            print(f"3 {total_amount}")
            if (total_amount <= 5):
                vel.linear.x += self.cohesion_weight * self.cohesion_vectors[i].x
                vel.linear.y += self.cohesion_weight * self.cohesion_vectors[i].y
            
            # Check avoid obstacle second to avoid collision with the enviroment
            total_amount += self.calc_length(self.alignment_weight, self.alignment_vectors[i])
            print(f"4 {total_amount}")
            if (total_amount <= 5):
                vel.linear.x += self.alignment_weight * self.alignment_vectors[i].x
                vel.linear.y += self.alignment_weight * self.alignment_vectors[i].y
            
            # Check avoid obstacle second to avoid collision with the enviroment
            total_amount += self.calc_length(self.nav2point_weight, self.nav2point_vectors[i])
            print(f"5 {total_amount}")
            if (total_amount <= 5):
                vel.linear.x += self.nav2point_weight * self.nav2point_vectors[i].x
                vel.linear.y += self.nav2point_weight * self.nav2point_vectors[i].y
            # else:
            #     scale = total_amount - length
            #     vel.linear.x += self.nav2point_weight * self.nav2point_vectors[i].x
            #     vel.linear.y += self.nav2point_weight * self.nav2point_vectors[i].y
            # total_amount += length

            self.publishers[self.robot_names[i]].publish(vel)


if __name__ == "__main__":
    rospy.init_node("reynolds_rules_node")
    node = ReynoldsRulesNode()
    rospy.spin()
