#!/usr/bin/env python

import rospy
from nav_msgs.msg import MapMetaData, OccupancyGrid

# Callbacks should...:
# Store the ramp location
# Store where we've been
# Store where samples are

class MapPublisher():
    def __init__(self):
        rospy.init_node("map_publisher")

        self.map = OccupancyGrid()

        # header stuff
        self.map.header.seq = 0 # increment this every time we publish the map
        self.map.header.stamp = rospy.Time.now()
        self.map.header.frame_id = "map"

        # info stuff
        self.map.info.origin.position.x = 10
        self.map.info.origin.position.y = 10
        self.map.info.width = 3
        self.map.info.height = 3
        self.map.info.resolution = .01 #m/cell
        self.map.data = [0] * self.map.info.height * self.map.info.width # that row-major order

        # publisher/subscriber stuff
        self.map_pub = rospy.Publisher("map",OccupancyGrid)
        # rospy.Subscriber("current_pos", subscribertype, somecallback, queue_size=1)
        # rospy.Subscriber("sample_finder", subscribertype, somecallback, queue_size=1)

        # mark_ramp()

    def publish_map(self):
        # handy helper function to call whenever you want to update the map
        self.map.header.seq += 1
        self.map_pub.publish(self.map)

    def pos_cb(self):
        # fill in squares where you are
        self.publish_map()

    def sample_cb(self):
        # fill in squares where sample is 
        self.publish_map()

    def mark_ramp(self):
        # fills in where the ramp is
        self.publish_map()

    def run(self):
        r = rospy.Rate(2)
        while not(rospy.is_shutdown()):
            self.publish_map()
            r.sleep()

if __name__ == '__main__':
    # test that jazz!
    try:
        node = MapPublisher()
        node.run()
    except rospy.ROSInterruptException: pass

