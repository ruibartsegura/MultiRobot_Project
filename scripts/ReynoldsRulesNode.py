#!/usr/bin/env python3

import rospy
import tf2_ros

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry


class ReynoldsRulesNode:
    def __init__(self):
        # Private param: number of robots
        self.n_robots = rospy.get_param("~number_robots", 10)

        # using tuple to make the list immutable
        self.robot_names = tuple([f"robot_{num}" for num in range(self.n_robots)])

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Make a dictionary, it will match each namespace with the corresponding publisher
        # To use the publisher self.publishers[self.robot_names[nº]].publish(TWIST)
        self.publishers = dict(
            (name, rospy.Publisher(f"{name}/cmd_vel", Twist, queue_size=1))
            for name in self.robot_names
        )

        # Make a dictionary, it will match each namespace with the corresponding position data
        # To get position of a robot self.poses[self.robot_names[0]]
        self.poses = {}
        for name in self.robot_names:
            rospy.Subscriber(f"{name}/odom", Odometry, self.callback_position)

        self.map: OccupancyGrid | None = None
        rospy.Subscriber("map", OccupancyGrid, self.callback_map)

    def callback_position(self, data):
        robot_name = data.header.frame_id.strip("/").removesuffix("odom").strip("/")
        self.poses[robot_name] = data.pose

    def callback_map(self, msg: OccupancyGrid):
        self.map = msg

    def map_lookup(self, xy) -> bool:
        i = int((xy[1] - self.map.info.origin.position.y) / self.map.info.resolution)
        j = int((xy[0] - self.map.info.origin.position.x) / self.map.info.resolution)
        index = self.map.info.width * i + j
        if not 0 <= index < len(self.map.data):
            return False
        return self.map.data[index] == 100

    def separation_rule(self):
        pass

    def alignment_rule(self):
        pass

    def cohesion_rule(self):
        pass

    def navigation_rule(self):
        pass

    def navigation_rule(self):
        pass

    def obstacle_avoidance_rule(self):
        pass

    def weighted_sum(self):
        pass


if __name__ == "__main__":
    rospy.init_node("ReynoldsRulesNode")
    try:
        node = ReynoldsRulesNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
