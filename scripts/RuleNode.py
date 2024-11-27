#!/usr/bin/env python3

import rospy

from nav_msgs.msg import Odometry
from reynolds_rules.msg import ArrayVectors


# Parent class os rules nodes classes
class RuleNode:
    def __init__(self, name: str, rate_per_second: int = 10):
        self.n_robots = rospy.get_param("~number_robots", 10)
        self.multiplier = rospy.get_param("~" + name + "_multiplier", 1)

        # Set common atributes of rules
        self.robots = [None] * self.n_robots

        self.pub = rospy.Publisher("/" + name + "_vectors", ArrayVectors, queue_size=1)

        # Creates a suscriber for each robot, but with the same callback
        for i in range(self.n_robots):
            topic = "/robot_" + str(i) + "/odom"
            rospy.Subscriber(topic, Odometry, self.robot_callback)

        rospy.sleep(1)

        rospy.Timer(rospy.Duration(1 / rate_per_second), self.control_cycle)
        #rospy.Timer(rospy.Duration(1 / rate), self.control_cycle)

    # Saves and updates list with odom of all robots
    def robot_callback(self, data: Odometry):
        begin = data.header.frame_id.find("/robot_") + 7
        end = data.header.frame_id.find("/odom")
        i = int(data.header.frame_id[begin:end])
        self.robots[i] = data

    # Abstract method
    def control_cycle(self, _):
        pass
