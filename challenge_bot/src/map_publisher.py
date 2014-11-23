#!/usr/bin/env python

# Emily Wang, Eric Schneider
# Computational Robotics, Fall 2014, Olin College, taught by Paul Ruvolo
# This code is responsible for the mapping for our challenge bot. The map 
# keeps track of where the robot has been, the ramp location, and sample locations
# using a ROS OccupancyGrid.

import rospy
from nav_msgs.msg import MapMetaData, OccupancyGrid
from challenge_msgs.srv import PointRequest, PointRequestResponse

class MapPublisher():
    def __init__(self):
        rospy.init_node("map_publisher")

        # ROS page on OccupancyGrid
        # http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html

        # Paul's create_map.py in the occupanygrid_mapping package in comprobo2014 
        # has also been a helpful example/reference for the map things.
        self.map = OccupancyGrid()

        # set up the service for updating the map with positions the robot has been 
        pos_s = rospy.Service('add_pos_to_map', PointRequest, self.handle_pos_service)

        # set up the service for updating the map with sample locations
        sample_s = rospy.Service('add_sample_pos_to_map', PointRequest, self.handle_sample_pos_service)

        # header stuff
        self.map.header.seq = 0 # increment this every time we publish the map
        self.map.header.stamp = rospy.Time.now()
        self.map.header.frame_id = "map"

        # info stuff
        # TODO: what should we do about the origin...
        self.map.info.origin.position.x = 0
        self.map.info.origin.position.y = 0

        # TODO: fix the resolution issues... cells and pixels and things
        self.map.info.width = 40
        self.map.info.height = 20
        self.map.info.resolution = .1 #m/cell
        self.map.data = [0] * self.map.info.height * self.map.info.width # that row-major order

        # occupancy vals for ramps, where we've been, and where the samples are
        self.RAMP_OCCUPANCY_VAL = 30
        self.HAVEBEEN_OCCUPANCY_VAL = 10
        self.SAMPLE_OCCUPANCY_VAL = 20

        # publisher/subscriber stuff
        self.map_pub = rospy.Publisher("map",OccupancyGrid)
        # TODO: edit and uncomment these lines when these subscribers have been made
        # rospy.Subscriber("current_pos", subscribertype, somecallback, queue_size=1)
        # rospy.Subscriber("sample_finder", subscribertype, somecallback, queue_size=1)

        # TODO: use the correct value for the ramp position
        self.mark_ramp(1, 0)
        print 'marked that ramp'

    def publish_map(self):
        # handy helper function to call whenever you want to update the map
        self.map.header.seq += 1
        self.map_pub.publish(self.map)

    def row_major_idx(self, x, y):
        # handy helper function for determining the index of the map list given

        # resolution things,
        x = x/self.map.info.resolution
        y = y/self.map.info.resolution      

        # x and y values. utilizes row major order
        return self.map.info.width*int(x) + int(y)

    def handle_pos_service(self, req):
        print req
        return self.pos_cb(req.point.x, req.point.y)
 
    def pos_cb(self, pos_x, pos_y):
        # fill in squares where you are with HAVEBEEN_OCCUPANCY_VAL
        idx = self.row_major_idx(pos_x, pos_y)
        self.map.data[idx] = self.HAVEBEEN_OCCUPANCY_VAL 
        self.publish_map()
        return PointRequestResponse()

    def handle_sample_pos_service(self, req):
        print req
        return self.sample_cb(req.point.x, req.point.y)

    def sample_cb(self, sample_x, sample_y):
        # fill in squares where sample is with SAMPLE_OCCUPANCY_VAL
        idx = self.row_major_idx(sample_x, sample_y)
        self.map.data[idx] = self.SAMPLE_OCCUPANCY_VAL 
        self.publish_map()
        return PointRequestResponse()

    def mark_ramp(self, ramp_x, ramp_y):
        # encode where the ramp is with RAMP_OCCUPANCY_VAL
        idx = self.row_major_idx(ramp_x, ramp_y)
        self.map.data[idx] = self.RAMP_OCCUPANCY_VAL 
        self.publish_map()

    def run(self):
        # rosrun like you mean it
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