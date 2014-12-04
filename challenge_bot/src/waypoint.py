#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from vector_tools import vector_ang, add_angles, create_angled_vector

class Waypoint():
    def __init__(self, wp_position, radius=0.25):
        self.point = wp_position
        self.radius = radius

    def is_complete(self, robot_point):
        """
        Returns true if the robot is within the waypoint's radius
        """
        dx = self.point.x - robot_point.x
        dy = self.point.y - robot_point.y
        if pow(pow(dx, 2) + pow(dy, 2), 0.5) < self.radius:
            return True
        else:
            return False

    def vector_to_wp(self, robot_point):
        """
        Returns a Vector3 vector pointing from the robot to the waypoint
        """
        dx = self.point.x - robot_point.x
        dy = self.point.y - robot_point.y
        abs_angle = vector_ang(Vector3(dx, dy, 0))
        rel_angle = add_angles(abs_angle, -robot_point.z)
        return create_angled_vector(rel_angle)
