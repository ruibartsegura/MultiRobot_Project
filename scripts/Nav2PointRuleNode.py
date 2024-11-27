#!/usr/bin/env python3

import rospy

class Nav2PointRuleNode:
    def __init__(self):
        pass

if __name__ == "__main__":
    rospy.init_node("nav_2_point")
    node = Nav2PointRuleNode()
    rospy.spin()