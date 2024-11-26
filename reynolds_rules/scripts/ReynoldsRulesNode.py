#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid


class ReynoldsRulesNode:
    def __init__(self):
        # Private param: number of robots 
        self.n_robots = rospy.get_param('~number_robots', 10)

        # Make a tuple with the correct namespace of the robots
        # I use a tuple to avoid having problems later if by mistake the list is changed
        self.robot_names = []
        name = "robot_" 
        for num in range (self.n_robots):
            new_name = name + str(num)
            self.robot_names.append(new_name)
        self.robot_names = tuple(self.robot_names)

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Make a dictionary, it will match each namespace with the corresponding publisher
        # To use the publisher self.publishers[self.robot_names[nº]].publish(TWIST)
        self.publishers = {}
        for name in self.robot_names:
            topic = name + "/cmd_vel"
            self.publishers[name] = rospy.Publisher(topic, Twist, queue_size=1)

        # Make a dictionary, it will match each namespace with the corresponding data of position
        # To get position of a robot self.poses[self.robot_names[0]]
        self.poses = {}
        for name in self.robot_names:
            topic = name + "/odom"
            rospy.Subscriber(topic, Odometry, self.callback_position)

    def callback_position(self, data):
        # Get which robot is getting the info
        name = data.header.frame_id.strip("/").removesuffix("odom").strip("/")
        print(name)
        self.poses[name] = data.pose 

        self.map_sub = rospy.Subscriber("map", OccupancyGrid, self.callback_map)
        self.map: OccupancyGrid | None = None

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
