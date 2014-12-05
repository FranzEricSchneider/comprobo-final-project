#!/usr/bin/env python

import rospy
from copy import deepcopy
from math import sin, cos, pi
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from vector_tools import vector_add, vector_multiply
from tf.transformations import quaternion_from_euler

class ObstacleAvoid():
    def __init__(self):
        rospy.init_node("ObstacleAvoid")

        # Bounds in meters for acceptable laser measurements
        self.LOWER_SCAN_BOUND = 0.1
        self.UPPER_SCAN_BOUND = 5.0
        # If an obstacle is more than OBS_SENSITIVITY meters away, this
        # obstacle avoid code will ignore it
        self.OBS_SENSITIVITY = .5
        # Scalar strength that tunes obstacle avoidance
        self.AVOID_STRENGTH = 1

        self.valid_points = {}
        self.last_scan = LaserScan()

        self.avoidance_pub = rospy.Publisher("/obstacle_avoid", Vector3,
                                             queue_size=1)
        rospy.loginfo("Initialized the /obstacle_avoid publisher")
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_cb)

        rospy.loginfo("Beginning the obstacle_avoid publishing")
        rospy.spin()

    def laser_cb(self, msg):
        """
        Grabs incoming laserscan data and saves it to a state variable
        """
        self.last_scan = deepcopy(msg)
        self.compute_valid()
        self.avoidance_publisher()

    def compute_valid(self):
        """
        Takes the laser scan and creates a dictionary of the valid points, with
        (degree = key) and (value = value)
        """
        self.valid_points = {}
        for i in range(len(self.last_scan.ranges)):
            if self.last_scan.ranges[i] > self.LOWER_SCAN_BOUND and\
               self.last_scan.ranges[i] < self.UPPER_SCAN_BOUND:
                self.valid_points[i] = self.last_scan.ranges[i]

    def avoidance_publisher(self):
        v = Vector3()
        points = deepcopy(self.valid_points)
        max_reaction = 0
        for point in points.keys():
            reaction = self.OBS_SENSITIVITY - points[point]
            if reaction > max_reaction:
                max_reaction = reaction
            unit_vector = [cos(point * (pi / 180.0)),
                           sin(point * (pi / 180.0))]
            x_val = max(reaction, 0.0) * -unit_vector[0]
            y_val = max(reaction, 0.0) * -unit_vector[1]
            v = vector_add(v, Vector3(x_val, y_val, 0.0))

        if max(abs(v.x), abs(v.y)) > 0:
            strength = self.AVOID_STRENGTH * max_reaction / max(abs(v.x), abs(v.y))
            v = vector_multiply(v, strength)
        else:
            v = Vector3()
        self.avoidance_pub.publish(v)

if __name__ == '__main__':
    oa = ObstacleAvoid()
