#!/usr/bin/env python3

import rospy


class ReynoldsRulesNode:
    def __init__(self):
        pass

    def map_subscription(self):
        pass

    def map_lookup(self):
        pass

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
