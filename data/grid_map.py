#!/usr/bin/env python

import rospy
import numpy as np
import tf.transformations
from nav_msgs.msg import OccupancyGrid

class GridMap():
    def __init__(self):
        self.grid_map = OccupancyGrid()
        self.map_pub = rospy.Publisher('/grid_map', OccupancyGrid, queue_size = 1, latch = True)

        rospy.Timer(rospy.Duration(1), self.publishNow)

    def publishNow(self, tevent):
        self.grid_map.header.stamp = rospy.Time.now()
        self.grid_map.header.frame_id = "map"
        self.grid_map.header.seq = 53
        self.grid_map.info.map_load_time.secs = 0
        self.grid_map.info.map_load_time.nsecs = 0 
        self.grid_map.info.map_load_time.nsecs = 0
        self.grid_map.info.resolution = 0.5
        self.grid_map.info.width = 10
        self.grid_map.info.height = 20
        self.grid_map.info.origin.position.x =-5
        self.grid_map.info.origin.position.y = .5
        self.grid_map.info.origin.position.z = 0
        self.grid_map.info.origin.orientation.x = 0
        self.grid_map.info.origin.orientation.y = 0
        self.grid_map.info.origin.orientation.z = 0
        self.grid_map.info.origin.orientation.w = 1

        self.grid_map.data =[-1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                            -1, 100, -1, -1, -1, -1, -1, -1, -1, -1,
                            -1, 100, -1, -1, -1, -1, -1, -1, -1, -1,
                            -1, 100, -1, -1, -1, -1, -1, -1, -1, -1,
                            -1, 100, -1, -1, -1, -1, -1, -1, -1, -1,
                            -1, 100, -1, -1, -1, -1, -1, -1, -1, -1,
                            -1, 100, -1, -1, -1, -1, -1, -1, -1, -1,
                            -1, 100, 100, 100, 100, -1, -1, -1, -1, -1,
                            -1, 0, 0, 0, 0, 100, 0, 0, 0, -1,
                            -1, 0, 0, 0, 0, 100, 0, 0, 0, -1,
                            -1, 0, 0, 0, 0, 100, 0, 0, 0, -1,
                            -1, 0, 0, 0, 0, 100, 0, 0, 0, -1,
                            -1, 0, 0, 0, 0, 100, 0, 0, 0, -1,
                            -1, 100, 100, 100, 100, 100, -1, -1, -1, -1,
                            -1, 100, -1, -1, -1, -1, -1, -1, -1, -1,
                            -1, 100, -1, -1, -1, -1, -1, -1, -1, -1,
                            100, 0, 0, 0, 0, 0, -1, -1, -1, 100,
                            100, 0, 0, 0, 0, 0, -1, -1, -1, 100,
                            100, 0, 0, 0, 0, 0, -1, -1, -1, 100,
                            100, 100, 100, 100, 100, 100, 100, 100, 100, 100]
        self.map_pub.publish(self.grid_map)
def main():
    rospy.init_node("grid_map")
    gridmap = GridMap()

    while not rospy.is_shutdown():
        rospy.spin
if __name__ == '__main__':
    main()

# header:
  # seq: 53
  # stamp:
    # secs: 2106
    # nsecs: 214000000
  # frame_id: "map"
# info:
  # map_load_time:
    # secs: 0
    # nsecs:         0
  # resolution: 0.0500000007451
  # width: 10
  # height: 20
  # origin:
    # position:
      # x: -5.62600040436
      # y: -5.33032560349
      # z: 0.0
    # orientation:
      # x: 0.0
      # y: 0.0
      # z: 0.0
      # w: 1.0
# data: [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        # -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        # -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        # -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        # -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        # -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        # -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        # -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        # -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        # -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        # -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        # -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        # -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        # -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        # -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        # -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        # -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        # -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        # -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
        # -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]
