#!/usr/bin/env python3
import rospy
import tf2_ros
import math

from geometry_msgs.msg import Twist, TransformStamped, Quaternion, PoseStamped
from tf2_geometry_msgs import do_transform_pose

class SeparationNode():

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
        self.publishers = {}
        for name in self.robot_names:
            topic = name + "/cmd_vel"
            self.publishers[name] = rospy.Publisher(topic, Twist, queue_size=1)
        
        # Timer Callback
        rospy.Timer(rospy.Duration(1.0 / 30.0), self.run)


    def run(self, _):
        # Main while loop.
        while not rospy.is_shutdown():
            self.cmd_vel = Twist()
            # Initialize message variables.
            self.cmd_vel.linear.x = 1
            self.cmd_vel.angular.z = 1

            # How to publish
            self.publishers[self.robot_names[9]].publish(self.cmd_vel)


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('pyclass')
    # Go to class functions that do all the heavy lifting.
    try:
        ne = SeparationNode()
        rospy.spin()
    except rospy.ROSInterruptException: pass
