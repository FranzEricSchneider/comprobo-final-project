#!/usr/bin/env python

# Emily Wang, Eric Schneider
# Computational Robotics, Fall 2014, Olin College, taught by Paul Ruvolo
# This code is the main class code for a robot that is prototyping the NASA
# sample challenge on a Neato robotics platform

import rospy
from geometry_msgs.msg import Twist, Vector3, Point
from nav_msgs.msg import OccupancyGrid
from copy import deepcopy
from math import copysign
from random import random

from waypoint import Waypoint
from vector_tools import *
from map_tools import *
from publish_rviz_vector import *


class ChallengeBot():
    def __init__(self):
        rospy.init_node('ChallengeBot')

        # Sets the max/min velocity (m/s) and linear velocity (rad/s)
        self.MAX_LINEAR = 0.1
        self.MIN_LINEAR = 0.0
        self.MAX_ANGULAR = .3
        self.MIN_ANGULAR = 0.0

        # Cutoff magnitudes below which no drive command will be published
        self.CMD_CUTOFF = 0.01
        self.AVOID_CMD_CUTOFF = 0.1

        # Stores the start time as a float, not as a Time datatype
        self.start_time = rospy.get_time()
        # Time limit of challenge in seconds
        self.TIME_LIMIT = 10 * 60.0

        self.vector_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.obs_avoid_vector = Vector3()
        self.obs_sub = rospy.Subscriber('/obstacle_avoid', Vector3,
                                        self.obs_cb)
        self.current_pos = Point()
        self.pos_sub = rospy.Subscriber('/current_pos', Point, self.pos_cb)
        self.map = OccupancyGrid()
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_cb)

        self.display_obstacle_avoid = RVIZVector("ObstacleAvoid")
        self.display_command_vector = RVIZVector("CommandedValue")
        self.display_combined_vector = RVIZVector("CombinedValue")

        self.unclaimed_samples = {}
        self.claimed_samples = {}

        # Logic for the SEEK behavior
        self.last_seek_cmd = Vector3()

    def stop(self):
        """
        Publishes empty vector to stop robot
        """
        cmd = Twist()
        self.vector_pub.publish(cmd)

    def drive_distance(self, distance):
        """
        Drives a given distance straight forward or back, in meters
        """
        distance_cmd = Twist(linear=Vector3(copysign(0.25, distance), 0, 0))
        self.vector_pub.publish(distance_cmd)
        rospy.sleep(4 * abs(distance))
        self.stop()

    def time_left(self):
        """
        Returns the time left for challenge, in float seconds, until time's out
        """
        return (self.start_time + self.TIME_LIMIT) - rospy.get_time()

    def drive_angle(self, angle):
        """
        Drives a given angle, (CCW, CW) is (+/-). In radians
        """
        angle_cmd = Twist(angular=Vector3(0, 0, copysign(1, angle)))
        self.vector_pub.publish(angle_cmd)
        rospy.sleep(abs(angle))
        self.stop()

    def drive_robot(self, cmd_vector, avoid_obs=True):
        """
        Takes in a Vector3 direction in which to drive the robot, then layers
        the obstacle avoid vector on top of that, if asked to
        """
        combined_vector = vector_add(cmd_vector, self.obs_avoid_vector)
        self.display_vectors(cmd_vector, combined_vector)        

        if vector_mag(self.obs_avoid_vector) < self.AVOID_CMD_CUTOFF\
           or not avoid_obs:

            if vector_mag(cmd_vector) > self.AVOID_CMD_CUTOFF:
                self.drive(cmd_vector)
            else:
                self.stop()
        else:
            if vector_mag(cmd_vector) < self.AVOID_CMD_CUTOFF:
                self.drive(self.obs_avoid_vector)
            else:
                self.drive(combined_vector)

    def drive(self, vector):
        """
        Interprets a vector direction into a Twist command for the Neato
        """
        cmd = Twist()
        ang = vector_ang(vector)

        # Above this angle, turn towards the point (rad)
        forced_turn_angle = 1.5707963
        # Above this angle decrease speed, make a tighter turn (rad)
        max_speed_angle = 0.7853982

        if abs(ang) > forced_turn_angle:
            cmd.linear.x = self.MIN_LINEAR
        else:
            # Linear fit, 2.0x at 0, 1.0x at 45 and 0.0x at 90 degrees
            cmd.linear.x = (1 - ((abs(ang) - max_speed_angle)
                            / max_speed_angle)) * self.MAX_LINEAR
            cmd.linear.x = min(cmd.linear.x, self.MAX_LINEAR)

        if abs(ang) > forced_turn_angle:
            cmd.angular.z = copysign(1, ang) * self.MAX_ANGULAR
        else:
            cmd.angular.z = ang / forced_turn_angle * self.MAX_ANGULAR

        self.vector_pub.publish(cmd)

    # TODO: The drive_wapyoints code needs tp be tested
    def drive_waypoints(self, waypoints):
        r = rospy.Rate(10)
        for wp in waypoints:
            while not wp.is_complete(self.current_pos)\
                  and not rospy.is_shutdown():
                v = wp.vector_to_wp(self.current_pos)
                self.drive_robot(wp.vector_to_wp(self.current_pos))
                r.sleep()
        self.stop()
        return

    def obs_cb(self, msg):
        self.obs_avoid_vector = msg

    def pos_cb(self, msg):
        self.current_pos = msg

    def map_cb(self, msg):
        self.map = msg
        mapped_samples = get_known_samples(self.map)
        for key in mapped_samples.keys():
            if not key in self.claimed_samples.keys():
                self.unclaimed_samples[key] = mapped_samples[key]

    def display_vectors(self, cmd, combine):
        """
        Uses the RVIZVector class to print a drive vector to RVIZ
        """
        self.display_obstacle_avoid.publish_marker(self.current_pos,
                                                   self.obs_avoid_vector)
        self.display_command_vector.publish_marker(self.current_pos,
                                                   cmd)
        self.display_combined_vector.publish_marker(self.current_pos,
                                                    combine)

    def seek(self):
        # TODO: Make smarter code that checks the map, finds the direction to
        # go, and adds a vector in that direction
        v = Vector3(random(), random()*2 - 1, 0)
        self.last_seek_cmd = create_unit_vector(vector_add(v,
                                                           self.last_seek_cmd))
        return self.last_seek_cm

    def grab(self):
        # TODO: Flesh this case out
        goal = closest_sample()
        wp = Waypoint(self.unclaimed_samples[goal], 1.5)
        self.drive_waypoints([wp])

        # If waypoint is visible [Talk to Emily about how to determine this]
            # Drive to goal perpindicular to sample, 1m away
        # Else (waypoint is not visible)
            # Drive to goal 90 degrees around the circle from where we are
        # Turn towards the sample
        # If the waypoint is still not visible
            # Erase the unclaimed sample from the map/dictionary and return?
        # Drive towards sample, check off using the value of goal

        return Vector3()

    # TODO: This is untested on the robot
    def closest_sample():
        """
        Looks at the unclaimed_samples dictionary and returns the key for the
        closest sample
        """
        closest_key = self.unclaimed_samples.keys()[0]
        min_distance = pt_to_pt_distance(self.current_pos,
                                         self.unclaimed_samples[closest_key])
        for key in self.unclaimed_samples.keys():
            diff = pt_to_pt_distance(self.current_pos,
                                     self.unclaimed_samples[key])
            if diff < min_distance:
                closest_key = key
                min_distance = diff
        return closest_key
