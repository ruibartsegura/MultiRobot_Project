#!/usr/bin/env python3

import rospy
import tf2_ros

from RuleNode import RuleNode
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import OccupancyGrid, Odometry
from reynolds_rules.msg import ArrayVectors # Import the custom message


class ReynoldsRulesNode(RuleNode):
    def __init__(self):
        super().__init__("reynold_rule", 100)
        
        # Subscribers to the rules topics
        rospy.Subscriber("/separation_vectors", ArrayVectors, self.separation_callback)
        #rospy.Subscriber("/name_vectors", ArrayVectors, self.name_callback)
        #rospy.Subscriber("/name_vectors", ArrayVectors, self.name_callback)
        #rospy.Subscriber("/name_vectors", ArrayVectors, self.name_callback)
        #rospy.Subscriber("/name_vectors", ArrayVectors, self.name_callback)

        # Variables to store the value of the publishers
        self.separation_vectors = ArrayVectors(vectors=[Vector3() for _ in range(10)])
        # self.name_vectors = ArrayVectors()
        # self.name_vectors = ArrayVectors()
        # self.name_vectors = ArrayVectors()
        # self.name_vectors = ArrayVectors()

        # Make a tuple with the correct namespace of the robots
        # I use a tuple to avoid having problems later if by mistake the list is changed
        self.robot_names = []
        name = "robot_" 
        for num in range (self.n_robots):
            new_name = name + str(num)
            self.robot_names.append(new_name)
        self.robot_names = tuple(self.robot_names)

        # Make a dictionary, it will match each namespace with the corresponding publisher
        # To use the publisher self.publishers[self.robot_names[nº]].publish(TWIST)
        self.publishers = {}
        for name in self.robot_names:
            topic = name + "/cmd_vel"
            self.publishers[name] = rospy.Publisher(topic, Twist, queue_size=1)


        self.map: OccupancyGrid | None = None
        rospy.Subscriber("map", OccupancyGrid, self.callback_map)

    def separation_callback(self, data):
        self.separation_vectors = data

    # def name_callback(self, data):
    #     self.name_vectors = data

    # def name_callback(self, data):
    #     self.name_vectors = data

    # def name_callback(self, data):
    #     self.name_vectors = data

    # def name_callback(self, data):
    #     self.name_vectors = data

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
            x_sep = self.separation_vectors.vectors[i].x
            y_sep = self.separation_vectors.vectors[i].y
            x_ali = 0.0  #self.separation_vectors.vectors[i].x
            y_ali = 0.0  #self.separation_vectors.vectors[i].y
            x_cohe = 0.0 #self.separation_vectors.vectors[i].x
            y_cohe = 0.0 #self.separation_vectors.vectors[i].y
            x_nav = 0.0  #self.separation_vectors.vectors[i].x
            y_nav = 0.0  #self.separation_vectors.vectors[i].y
            x_obst = 0.0 #self.separation_vectors.vectors[i].x
            y_obst = 0.0 #self.separation_vectors.vectors[i].y

            vel = Twist()
            vel.linear.x = x_sep + x_ali + x_cohe + x_nav + x_obst
            vel.linear.y = y_sep + y_ali + y_cohe + y_nav + y_obst
            print(self.robot_names[i])
            print(f"x {vel.linear.x}, y {vel.linear.y}")
            self.publishers[self.robot_names[i]].publish(vel)


if __name__ == "__main__":
    rospy.init_node("ReynoldsRulesNode")
    node = ReynoldsRulesNode()
    rospy.spin()

