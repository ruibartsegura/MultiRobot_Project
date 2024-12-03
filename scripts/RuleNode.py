#!/usr/bin/env python3

import rospy

from nav_msgs.msg import Odometry
from reynolds_rules.msg import VectorArray


# Parent class of rule node classes
class RuleNode:
    def __init__(self, name):
        self.n_robots = rospy.get_param("number_robots", 10)
        refresh_rate = rospy.get_param("refresh_rate", 20)

        print(f"Starting the {name} node.")
        print(f"  number_robots: {self.n_robots}")
        print(f"  refresh_rate: {refresh_rate}")

        self.robots = [Odometry() for _ in range(self.n_robots)]

        for i in range(self.n_robots):
            rospy.Subscriber(f"/robot_{i}/odom", Odometry, self.robot_callback)

        self.pub = rospy.Publisher("/" + name + "_vectors", VectorArray, queue_size=1)
        rospy.Timer(rospy.Duration(1 / refresh_rate), self.control_cycle)

    def robot_callback(self, data: Odometry):
        # Saves and updates list with odom of all robots.
        begin = data.header.frame_id.find("/robot_") + len("/robot_")
        end = data.header.frame_id.find("/odom")

        i = int(data.header.frame_id[begin:end])
        self.robots[i] = data

    def control_cycle(self, _):
        # Abstract, principal method.
        pass
