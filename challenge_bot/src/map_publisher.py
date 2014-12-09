#!/usr/bin/env python

# Emily Wang, Eric Schneider
# Computational Robotics, Fall 2014, Olin College, taught by Paul Ruvolo
# This code is responsible for the mapping for our challenge bot. The map 
# keeps track of where the robot has been, the ramp location, and sample locations
# using a ROS OccupancyGrid.

import rospy
from nav_msgs.msg import MapMetaData, OccupancyGrid
from std_srvs.srv import Empty
from challenge_msgs.srv import PointRequest, PointRequestResponse
from challenge_msgs.srv import SamplePoint, SamplePointResponse
from challenge_msgs.srv import OccupancyValue, OccupancyValueResponse
from challenge_msgs.srv import ClearMap, ClearMapResponse

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
        clear_map_s = rospy.Service('clear_map', ClearMap, self.handle_clear_map)

        # set up the service to remove points of a specific occupancy value on the map
        remove_specific_occupancy_val_s = rospy.Service('remove_specific_occupancy_val', OccupancyValue, self.handle_remove_specific_occupancy_val)

        # set up the service for updating the map with sample locations
        print_s = rospy.Service('print_all_map_values', Empty, self.print_all_values)

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
        self.SAMPLE_OCCUPANCY_VALS = {'a':20, 'b':21, 'c':22, 'd':23, 'g':24, 'f':25}

        # publisher/subscriber stuff
        self.map_pub = rospy.Publisher("/map",OccupancyGrid, queue_size=1)
        # TODO: edit and uncomment these lines when these subscribers have been made
        # rospy.Subscriber("current_pos", subscribertype, somecallback, queue_size=1)
        # rospy.Subscriber("sample_finder", subscribertype, somecallback, queue_size=1)

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

    def set_value(self, x, y, occupancy_val):
        # convert from meters to pixels yo
        x = (x - self.map.info.origin.position.x) / self.map.info.resolution 
        y = (y - self.map.info.origin.position.y) / self.map.info.resolution      

        # are you in bounds??
        if (x < self.map.info.width and x >= 0) and (y < self.map.info.height and y >= 0):
            self.map.data[self.row_major_idx(x, y)] = occupancy_val
        else: # you're out of bounds 
            rospy.logwarn("Your point, (%d px, %d px), is out of bounds! Offending occupancy_val: ", x, y, occupancy_val)

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
        rospy.loginfo("Sample %s ahoy at (%f, %f)!", sample_f_str, sample_x, sample_y)
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
        self.mark_ramp(self.RAMP_X, self.RAMP_Y) # mark the ramp again        
        return ClearMapResponse()

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
                if self.map.data[idx] > 0:
                    x_meters = x * self.map.info.resolution + self.map.info.origin.position.x
                    y_meters = y * self.map.info.resolution + self.map.info.origin.position.y
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
