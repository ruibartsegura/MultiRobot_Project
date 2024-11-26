#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid


class ReynoldsRulesNode:
    def __init__(self):
        pass

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
