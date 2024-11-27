#!/usr/bin/env python3

import rospy

from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Vector3


# Parent class os rules nodes classes
class RuleNode:
    def __init__(self, name, rate):
        self.n_robots = rospy.get_param("~number_robots", 10)
        self.multiplier = rospy.get_param("~" + name + "_multiplier", 1)

        # Set common atributes of rules
        self.robots = []

        rospy.Timer(rospy.Duration(1 / rate), self.control_cycle)
        # self.pub = rospy.Publisher("/" + name + "_vectors", Vector3, queue_size=1)

        # Creates a suscriber for each robot, but with the same callback
        for i in range(self.n_robots):
            topic = "/robot_" + str(i + 1) + "/odom"
            rospy.Subscriber(topic, Odometry, self.robot_callback)

    # Saves and updates list with odom of all robots
    def robot_callback(self, data):
        id = int(data.child_frame_id.removesuffix("/odom").removeprefix("/robot_"))
        self.robots[id - 1] = data

    # Abstract method
    def control_cycle(self, _):
        pass
