#!/usr/bin/env python
from rospy import logerr
from geometry_msgs.msg import Point, Vector3
from vector_tools import add_angles, angle_difference

def vector_to_point(vector3):
    return Point(vector3.x, vector3.y, vector3.z)

def point_to_vector(point):
    return Vector3(point.x, point.y, point.z)

def array_to_point(np_array):
	"""
	Converts a numpy array to a ROS Point
	"""
	if len(np_array) != 3:
		logerr("The array given to point_tool.py function array_to_point\n\
				requires 3 elts and was given %d elts", len(np_array))
	point = Point()
	point.x = np_array[0]
	point.y = np_array[1]
	point.z = np_array[2]
	return point

def add_points(point1, point2):
    """
    Add points (rad) and output a new points w/ angles bounded from -/+ pi
    """
    point = Point()
    point.x = point1.x + point2.x
    point.y = point1.y + point2.y
    point.z = add_angles(point1.z, point2.z)
    return point

def point_difference(point1, point2):
    """
    Returns the vector FROM point1 TO point2, with the angle change in radians
    """
    delta_point = Point()
    delta_point.x = point2.x - point1.x
    delta_point.y = point2.y - point1.y
    delta_point.z = angle_difference(point1.z, point2.z)
    return delta_point

def pt_to_pt_distance(point1, point2):
    """
    Returns, as a float, the 2D distance between two points
    """
    diff = point_difference(point1, point2)
    return pow(pow(diff.x, 2) + pow(diff.y, 2), 0.5)