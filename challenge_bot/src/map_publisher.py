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
        self.map.info.origin.position.x = -4
        self.map.info.origin.position.y = -4

        self.map.info.width = 80 #pixels 
        self.map.info.height = 80 #pixels 
        self.map.info.resolution = .1 #m/cell
        self.map.data = [0] * self.map.info.height * self.map.info.width # that row-major order

        # occupancy vals for ramps, where we've been, and where the samples are
        self.RAMP_OCCUPANCY_VAL = 30
        self.HAVEBEEN_OCCUPANCY_VAL = 10
        self.SAMPLE_OCCUPANCY_VAL = 20

        # publisher/subscriber stuff
        self.map_pub = rospy.Publisher("/map", OccupancyGrid)
        # TODO: edit and uncomment these lines when these subscribers have been made
        # rospy.Subscriber("current_pos", subscribertype, somecallback, queue_size=1)
        # rospy.Subscriber("sample_finder", subscribertype, somecallback, queue_size=1)

        # TODO: use the correct value for the ramp position
        self.mark_ramp(1, 0)

    def publish_map(self):
        # handy helper function to call whenever you want to update the map
        self.map.header.seq += 1
        self.map_pub.publish(self.map)

    def row_major_idx(self, x, y):
        """
        handy helper function for determining the index of the map list given
        x and y values. utilizes row major order
        """
        return self.map.info.width*int(y) + int(x)

    def set_value(self, x, y, occupancy_val):
        # convert from meters to pixels yo
        x = (x - self.map.info.origin.position.x) / self.map.info.resolution 
        y = (y - self.map.info.origin.position.y) / self.map.info.resolution      

        # are you in bounds??
        if (x < self.map.info.width and x >= 0) and (y < self.map.info.height and y >= 0):
            self.map.data[self.row_major_idx(x, y)] = occupancy_val
        else: # you're out of bounds 
            rospy.logwarn("Your point, (%d px, %d px), is out of bounds!", x, y)

    def handle_pos_service(self, req):
        return self.pos_cb(req.point.x, req.point.y)
 
    def pos_cb(self, pos_x, pos_y):
        # fill in squares where you are with HAVEBEEN_OCCUPANCY_VAL
        self.set_value(pos_x, pos_y, self.HAVEBEEN_OCCUPANCY_VAL)
        self.publish_map()
        return PointRequestResponse()

    def handle_sample_pos_service(self, req):
        return self.sample_cb(req.point.x, req.point.y)

    def sample_cb(self, sample_x, sample_y):
        # fill in squares where sample is with SAMPLE_OCCUPANCY_VAL
        self.set_value(sample_x, sample_y, self.SAMPLE_OCCUPANCY_VAL) 
        rospy.loginfo("Sample ahoy at (%f, %f)!", sample_x, sample_y)
        self.publish_map()
        return PointRequestResponse()

    def mark_ramp(self, ramp_x, ramp_y):
        # encode where the ramp is with RAMP_OCCUPANCY_VAL
        self.set_value(ramp_x, ramp_y, self.RAMP_OCCUPANCY_VAL)
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