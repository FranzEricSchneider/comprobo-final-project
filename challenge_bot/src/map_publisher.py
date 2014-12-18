#!/usr/bin/env python

# Emily Wang, Eric Schneider
# Computational Robotics, Fall 2014, Olin College, taught by Paul Ruvolo
# This code is responsible for the mapping for our challenge bot. The map 
# keeps track of where the robot has been, the ramp location, and sample locations
# using a ROS OccupancyGrid.

import rospy
import numpy as np
from nav_msgs.msg import MapMetaData, OccupancyGrid
from std_srvs.srv import Empty
from challenge_msgs.srv import PointRequest, PointRequestResponse
from challenge_msgs.srv import SamplePoint, SamplePointResponse
from challenge_msgs.srv import OccupancyValue, OccupancyValueResponse

class MapPublisher():
    def __init__(self):
        rospy.init_node("map_publisher")

        # ROS page on OccupancyGrid
        # http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html

        # Paul's create_map.py in the occupanygrid_mapping package in comprobo2014 
        # has also been a helpful example/reference for the map things.
        self.map = OccupancyGrid()

        ### services! 
        # set up the service for updating the map with positions the robot has been 
        pos_s = rospy.Service('add_pos_to_map', PointRequest, self.handle_pos_service)

        # set up the service for updating the map with sample locations
        sample_s = rospy.Service('add_sample_pos_to_map', SamplePoint, self.handle_sample_pos_service)

        # set up the service for clearing the map
        clear_map_s = rospy.Service('clear_map', Empty, self.handle_clear_map)

        # set up the service to remove points of a specific occupancy value on the map
        remove_specific_occupancy_val_s = rospy.Service('remove_specific_occupancy_val', OccupancyValue, self.handle_remove_specific_occupancy_val)

        # set up the service for updating the map with sample locations
        print_s = rospy.Service('print_all_map_values', Empty, self.print_all_values)

        # set up the service for notifying which region has been traveled in the least
        least_traveled_region_s = rospy.Service('least_traveled_region', Empty, self.handle_least_traveled_region)

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
        self.SAMPLE_OCCUPANCY_VALS = {'a':50, 'b':60, 'c':70, 'd':80, 'g':90, 'f':100}

        # publisher/subscriber stuff
        self.map_pub = rospy.Publisher("/map",OccupancyGrid, queue_size=1)

        # helpful things to track for seek!
        # counting how many steps the bot has taken in the four quadrants on the map
        # do we care about re-stepping?
        # can expand to utilize more regions if desired!
        # ravel
        # self.reshaped_map = self.map.data[:].reshape(self.map.info.width, self.map.info.height)
        self.region_counters = {'top_left':0, 'top_right':0, 'bottom_left':0, 'bottom_right':0} 

        # TODO: use the correct value for the ramp position
        self.RAMP_X = 1
        self.RAMP_Y = 0

        self.mark_ramp(self.RAMP_X, self.RAMP_Y)

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

    # def set_value(self, x, y, occupancy_val):
    #     # convert from meters to pixels yo
    #     x = (x - self.map.info.origin.position.x) / self.map.info.resolution 
    #     y = (y - self.map.info.origin.position.y) / self.map.info.resolution      

    #     # are you in bounds??
    #     if (x < self.map.info.width and x >= 0) and (y < self.map.info.height and y >= 0):
    #         reshaped_map = np.array(self.map.data).reshape(self.map.info.width, self.map.info.height)
    #         reshaped_map[x, y] = occupancy_val

    #         # which region are you in? 
    #         # increment the appropriate region counter to help with SEEK behavior decisions
    #         if (0 < x < self.map.info.width/2) and (0 < y < self.map.info.height/2):
    #             self.region_counters['top_left'] += 1
    #         elif (self.map.info.width/2 < x < self.map.info.width) and (self.map.info.height/2 < x < self.map.info.height):
    #             self.region_counters['top_right'] += 1
    #         elif (0 < x < self.map.info.width/2) and (self.map.info.height/2 < y < self.map.info.height):
    #             self.region_counters['bottom_left'] += 1
    #         elif (self.map.info.width/2 < x < self.map.info.width) and (self.map.info.height/2 < y < self.map.info.height):
    #             self.region_counters['bottom_right'] += 1

    #         self.map.data = reshaped_map.ravel()

    #     else: # you're out of bounds 
    #         rospy.logwarn("Your point, (%d px, %d px), is out of bounds! Offending occupancy_val: %d", x, y, occupancy_val)

    def set_value(self, x, y, occupancy_val):
        # convert from meters to pixels yo
        x = (x - self.map.info.origin.position.x) / self.map.info.resolution 
        y = (y - self.map.info.origin.position.y) / self.map.info.resolution      

        # are you in bounds??
        if (x < self.map.info.width and x >= 0) and (y < self.map.info.height and y >= 0):
            self.map.data[self.row_major_idx(x, y)] = occupancy_val
        else: # you're out of bounds 
            rospy.logwarn("Your point, (%d px, %d px), is out of bounds! Offending occupancy_val: %d", x, y, occupancy_val)


    def handle_pos_service(self, req):
        return self.pos_cb(req.point.x, req.point.y)
 
    def pos_cb(self, pos_x, pos_y):
        """
        fill in squares where you are with HAVEBEEN_OCCUPANCY_VAL
        """
        self.set_value(pos_x, pos_y, self.HAVEBEEN_OCCUPANCY_VAL)
        self.publish_map()
        return PointRequestResponse()

    def handle_sample_pos_service(self, req):
        if req.fiducial in self.SAMPLE_OCCUPANCY_VALS.keys():
            return self.sample_cb(req.point.x, req.point.y, req.fiducial)
        else:
            return rospy.logwarn('You failed to pass in a valid string')

    def sample_cb(self, sample_x, sample_y, sample_f_str):
        """
        fill in squares where sample is with the appropriate SAMPLE_OCCUPANCY_VAL for sample_f_str
        """
        self.set_value(sample_x, sample_y, self.SAMPLE_OCCUPANCY_VALS[sample_f_str]) 
        rospy.logdebug("Sample %s ahoy at (%f, %f)!", sample_f_str, sample_x, sample_y)
        self.publish_map()
        return SamplePointResponse()

    def handle_remove_specific_occupancy_val(self, req):
        return self.remove_specific_occupancy_val_cb(req.occupancy_value)

    def remove_specific_occupancy_val_cb(self, occupancy_value):
        # find squares with occupancy_value and set them to 0 on the map 
        # ex. clearing anything with sample a's value on the map
        for i in range(len(self.map.data)):
            if self.map.data[i] == occupancy_value:
                self.map.data[i] = 0
        return OccupancyValueResponse()

    def handle_clear_map(self, req):
        return self.clear_map_cb()

    def clear_map_cb(self):
        self.map.data = [0] * self.map.info.height * self.map.info.width # that row-major order
        self.region_counters = {'top_left':0, 'top_right':0, 'bottom_left':0, 'bottom_right':0} 
        self.mark_ramp(self.RAMP_X, self.RAMP_Y) # mark the ramp again        
        return []

    def handle_least_traveled_region(self, req):
        return self.least_traveled_region_cb()

    def least_traveled_region_cb(self):
        least_traveled_region = 'top_left'
        for region in self.region_counters:
            if self.region_counters[region] < self.region_counters[least_traveled_region]:
                least_traveled_region = region
        return least_traveled_region

    def mark_ramp(self, ramp_x, ramp_y):
        """
        encode where the ramp is with RAMP_OCCUPANCY_VAL
        """
        self.set_value(ramp_x, ramp_y, self.RAMP_OCCUPANCY_VAL)
        self.publish_map()

    def print_all_values(self, req):
        print "GOT HERE"
        print_string = "Current map values, in meters\n"
        for x in range(self.map.info.width):
            for y in range(self.map.info.height):
                idx = self.row_major_idx(x, y)
                if self.map.data[idx] > 0 and \
                   self.map.data[idx] != self.HAVEBEEN_OCCUPANCY_VAL:
                    x_meters = x * self.map.info.resolution + \
                               self.map.info.origin.position.x
                    y_meters = y * self.map.info.resolution + \
                               self.map.info.origin.position.y
                    print_string += "(%f, %f) has value %d\n" % (x_meters, y_meters,
                                                                 self.map.data[idx])
        rospy.loginfo(print_string)
        return []

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
