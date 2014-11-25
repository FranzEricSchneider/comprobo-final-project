#!/usr/bin/env python

# Emily Wang, Eric Schneider
# Computational Robotics, Fall 2014, Olin College, taught by Paul Ruvolo
# This code has basic tools for searching the maps we made for information

import rospy
from nav_msgs.msg import MapMetaData, OccupancyGrid
from geometry_msgs.msg import Point

def avg_point_of_value(field_map, value):
    avg_x = 0.0
    avg_y = 0.0
    num_pts = 0
    for x_px in range(field_map.info.width):
        for y_px in range(field_map.info.height):
            if field_map.data[field_map.row_major_idx(x_px, y_px)] == value:
                print (x_px, y_px)
                num_pts += 1
                avg_x += 
