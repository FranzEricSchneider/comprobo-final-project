#!/usr/bin/env python

# Emily Wang, Eric Schneider
# Computational Robotics, Fall 2014, Olin College, taught by Paul Ruvolo
# Tools to take a vector and publish it in RVIZ with markers


import rospy
from copy import deepcopy
from geometry_msgs.msg import Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
from vector_tools import *


class RVIZVector():
    def __init__(self, name):
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker,
                                     queue_size=1)
        self.name = name
        self.shape = Marker.ARROW

        if name == "ObstacleAvoid":
            self.color = ColorRGBA(1., 0., 0., 1.)
        elif name == "CommandedValue":
            self.color = ColorRGBA(0., 0., 1., 1.)
        else:
            self.color = ColorRGBA(0.5, 0., 0.5, 1.)

    def publish_marker(self, current_pos, vector):
        angle = vector_ang(vector) + current_pos.z
        magnitude = vector_mag(vector)
        marker = self.populate_marker()

        # Position
        marker.pose.position = deepcopy(current_pos)
        marker.pose.position.z = 0.0
        
        # Magnitude
        magnitude = max(magnitude, 0.01)
        marker.scale.x = magnitude
        marker.scale.y = min(0.04, magnitude);
        marker.scale.z = min(0.04, magnitude);

        # Angle
        q_angle = quaternion_from_euler(0, 0, angle) # roll, pitch, yaw
        marker.pose.orientation = Quaternion(q_angle[0],
                                             q_angle[1],
                                             q_angle[2],
                                             q_angle[3])
        self.marker_pub.publish(marker);
        rospy.sleep(0.01)  # Without the sleep call the markers eat each other

    def populate_marker(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.id = 0
        marker.ns = self.name
        marker.type = self.shape
        marker.action = Marker.ADD
        marker.color = self.color
        marker.lifetime = rospy.Duration();
        return marker