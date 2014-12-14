#!/usr/bin/env python

# Emily Wang, Eric Schneider
# Computational Robotics, Fall 2014, Olin College, taught by Paul Ruvolo
# This code has basic tools for searching the maps we made for information

import rospy
from copy import deepcopy
from nav_msgs.msg import MapMetaData, OccupancyGrid
from geometry_msgs.msg import Point

def get_known_samples(field_map):
    """
    Returns a dictionary of found samples paired with the average position of
    the found sample number
    This assumes that the samples are added in increasing order from 20
    """
    found_samples = {}
    sample_val = 50
    for i in range(6):
        point = avg_point_of_value(field_map, sample_val + 10 * i)
        if point == -1:
            pass
        else:
            found_samples[sample_val + 10 * i] = deepcopy(point)
    return found_samples

def avg_point_of_value(field_map, value):
    """
    Takes a value to search for in the map and returns the averaged Point of
    all points that had that value. If that point doesn't exist, returns -1
    """
    avg_x = 0.0
    avg_y = 0.0
    num_pts = 0
    for x_px in range(field_map.info.width):
        for y_px in range(field_map.info.height):
            if field_map.data[row_major_idx(field_map, x_px, y_px)] == value:
                num_pts += 1
                x_m, y_m = pixels_to_meters(field_map, x_px, y_px)
                avg_x += x_m
                avg_y += y_m
    if num_pts == 0:
        return -1
    else:
        avg_x /= num_pts
        avg_y /= num_pts
        return Point(avg_x, avg_y, 0)

def row_major_idx(field_map, x, y):
    """
    handy helper function for determining the index of the map list given
    x and y values. utilizes row major order
    """
    return field_map.info.width*int(y) + int(x)

def pixels_to_meters(field_map, x_px, y_px):
    """
    Takes an x, y pair in pixels and converts them to meters
    """
    x_m = x_px * field_map.info.resolution + field_map.info.origin.position.x
    y_m = y_px * field_map.info.resolution + field_map.info.origin.position.y
    return (x_m, y_m)

def meters_to_pixels(field_map, x_m, y_m):
    """
    Takes an x, y pair in meters and converts them to pixels
    """
    x_px = (x_m - field_map.info.origin.position.x) / field_map.info.resolution
    y_px = (y_m - field_map.info.origin.position.y) / field_map.info.resolution
    return (x_px, y_px)