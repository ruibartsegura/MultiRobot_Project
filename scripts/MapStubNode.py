#!/usr/bin/env python3

import json
import rospy
import rospkg
import os

from nav_msgs.msg import OccupancyGrid


class MapStubNode:
    def __init__(self):
        self.map_file = rospy.get_param("~map_file", "map.json")

        self.map: OccupancyGrid | None = None
        self.pub = rospy.Publisher("map", OccupancyGrid, queue_size=1)

        package = rospkg.RosPack().get_path("reynolds_rules")
        self.map_file = f"{package}/{self.map_file}"
        print(f"map_file: {self.map_file}")
        self.read_map_file()

        rospy.Subscriber("src", OccupancyGrid, self.callback_map)

    def callback_map(self, msg: OccupancyGrid):
        self.map = msg

        if self.map_file:
            self.write_map_file()

    def read_map_file(self):
        if not os.path.isfile(self.map_file):
            print("WARNING: map_file does not exist")
            return

        with open(self.map_file, "r") as f:
            json_object = json.load(f)
            self.map = OccupancyGrid()
            self.map.info.resolution = json_object["resolution"]
            self.map.info.width = json_object["width"]
            self.map.info.height = json_object["height"]
            self.map.info.origin.position.x = json_object["origin_x"]
            self.map.info.origin.position.y = json_object["origin_y"]
            self.map.data = json_object["data"]
            self.pub.publish(self.map)
        print(f"Read and published map to {self.pub.resolved_name}.")

    def write_map_file(self):
        data = {
            "resolution": self.map.info.resolution,
            "width": self.map.info.width,
            "height": self.map.info.height,
            "origin_x": self.map.info.origin.position.x,
            "origin_y": self.map.info.origin.position.y,
            "data": self.map.data,
        }

        json_object = json.dumps(data)
        with open(self.map_file, "w") as f:
            f.write(json_object)

        print("Wrote to map file.")


if __name__ == "__main__":
    rospy.init_node("map_stub")
    node = MapStubNode()
    rospy.spin()
