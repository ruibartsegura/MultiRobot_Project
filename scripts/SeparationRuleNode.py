#!/usr/bin/env python3
import rospy
import tf2_ros
import math

from RuleNode import RuleNode
from reynolds_rules.msg import VectorArray # Import the custom message
from geometry_msgs.msg import Point


class SeparationRuleNode(RuleNode):
    def __init__(self):
        super().__init__("separation", 10)
        
        self.separation_range = rospy.get_param("~separation_range", 0.3)

    
    def get_distance(self, pos1, pos2):
        x = pos1.x - pos2.x
        y = pos1.y - pos2.y
        return math.sqrt(x * x + y * y)

    def calc_vector(self, position, num):
        repulsive_vector = Point()
        # k is the cte of force
        k = 0.01

        for i in range(self.n_robots):            
            # Avoid calculating the robotr's own vector
            if i != num:
                # Get the distance betwen the robots
                dist = self.get_distance(position, self.robots[i].pose.pose.position)

                # Check if the distance is in the radious
                if dist > 0.0 and dist < self.separation_range:
                    #Get the x, y coords of the vector
                    x = position.x - self.robots[i].pose.pose.position.x
                    y = position.y - self.robots[i].pose.pose.position.y

                    # Normalize the vector
                    norm = math.sqrt(x * x + y * y)
                    if norm > 0:
                        direction = [x / norm, y / norm]
                    else:
                        direction = [0.0, 0.0]

                    # Magnitude od the repulsice vector
                    magnitude = k / (dist*dist)

                    # Sum to the total the repulsive vector of robot_i
                    repulsive_vector.x += magnitude * direction[0]
                    repulsive_vector.y += magnitude * direction[1]

        return repulsive_vector

    
    def control_cycle(self, _):
        msg = VectorArray()
        separation_vectors = []
        
        for i in range(self.n_robots):
            separation_vectors.append(self.calc_vector(self.robots[i].pose.pose.position, i))

        msg.vectors = separation_vectors
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node("separation_rule")
    node = SeparationRuleNode()
    rospy.spin()

# TODO Create a param to the distance of separation