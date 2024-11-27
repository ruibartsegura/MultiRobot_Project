#!/usr/bin/env python3
import rospy
import tf2_ros
import math

from RuleNode import RuleNode
from geometry_msgs.msg import Twist, TransformStamped, Quaternion, PoseStamped, Pose
from tf2_geometry_msgs import do_transform_pose
from nav_msgs.msg import Odometry

class SeparationRuleNode(RuleNode):
    def __init__(self):
        super().__init__("separation", 10)

    
    def get_distance(pos1, pos2):
        x = pos1.x - pos2.x
        y = pos1.y - pos2.y
        return sqr(x * x + y * y)

    def calc_vector(position, num):
        for i in range(self.n_robots):
            #print(f"Distancia entre {num} y {i} = {self.get_distance(position, self.robot[i].position)}")
            if i != num or self.get_distance(position, self.robots[i].position) < 10:
                # Calc vector sep
                print("Distancia valida")
        return [0, 0]

    
    def control_cycle(self, _):
        vectors = [self.n_robots]
        
        for i in range(self.n_robots):
            print(f"i = {i}")
            vectors.append(self.calc_vector(self.robots[i].position, i))


if __name__ == '__main__':
    rospy.init_node("separation_rule")
    node = SeparationRuleNode()
    rospy.spin()
