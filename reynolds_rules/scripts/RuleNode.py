import rospy

from nav_msgs.msg import Odometry


class RuleNode:
    def __init__(self):
        n_robots = rospy.get_param("~number_robots", 10)
        self.robots = list[n_robots]

        for i in range(n_robots):
            rospy.Subscriber("/robot_" + i + "/odom", Odometry, self.robot_callback)

    def robot_callback(self, data):
        id = int(data.child_frame_id.removesuffix("/odom").removeprefix("/robot_"))
        self.robots[id] = data
