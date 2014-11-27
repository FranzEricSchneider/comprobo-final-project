#!/usr/bin/env python

# Emily Wang, Eric Schneider
# Computational Robotics, Fall 2014, Olin College, taught by Paul Ruvolo
# This code is the main class code for a robot that is prototyping the NASA
# sample challenge on a Neato robotics platform

import rospy
from geometry_msgs.msg import Twist, Vector3, Point, PoseStamped, Pose, Quaternion
from nav_msgs.msg import OccupancyGrid
from copy import deepcopy
from math import copysign
from random import random
from tf.transformations import quaternion_from_euler

from waypoint import Waypoint
from vector_tools import *
from map_tools import *


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

        self.obstacle_avoid_vector_pub = rospy.Publisher('/obstacle_avoid_vector', PoseStamped, queue_size=1)
        self.cmd_vector_pub = rospy.Publisher('/cmd_vector', PoseStamped, queue_size=1)
        self.oa_cmd_combined_vector_pub = rospy.Publisher('/oa_cmd_combined_vector', PoseStamped, queue_size=1)

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

    def vector_point_orientation(self, v):
        """
        Returns the point and orientation required to make a Pose for input vector v.
        """
        v_point = Point(v.x, v.y, 0)
        v_orientation = quaternion_from_euler(0, 0, v.z) # roll, pitch, yaw
        v_orientation = Quaternion(v_orientation[0], v_orientation[1], v_orientation[2], v_orientation[3])
        return (v_point, v_orientation)

    def drive_robot(self, cmd_vector, avoid_obs=True):
        """
        Takes in a Vector3 direction in which to drive the robot, then layers
        the obstacle avoid vector on top of that, if asked to
        """
        print "obs_avoid_vector magnitude: ", vector_mag(self.obs_avoid_vector)
        print "cmd_vector magnitude: ", vector_mag(cmd_vector), "\n"

        combined_vector = vector_add(cmd_vector, self.obs_avoid_vector)
        
        # publish poses: obs, cmd, combined vector_add(cmd_vector, self.obs_avoid_vector)
        # self.obstacle_avoid_vector_pub.publish(Pose(self.obs_avoid_vector.x, self.obs_avoid_vector.y))
        # self.cmd_vector_pub.publish(Pose(cmd_vector.x, cmd_vector.y))
        # self.oa_cmd_combined_vector_pub.publish(Pose(combined_vector.x, combined_vector.y))

        obstacle_avoid_vector_po = self.vector_point_orientation(self.obs_avoid_vector)        
        cmd_vector_po = self.vector_point_orientation(cmd_vector)
        combined_vector_po = self.vector_point_orientation(combined_vector)

        oa_pose = PoseStamped()
        oa_pose.header.seq = 0 
        oa_pose.header.stamp = rospy.Time.now()
        oa_pose.header.frame_id = "obstacle_avoidance_pose"
        oa_pose.pose = Pose(obstacle_avoid_vector_po[0], obstacle_avoid_vector_po[1])

        self.obstacle_avoid_vector_pub.publish(oa_pose)
        # print type(obstacle_avoid_vector_po[0]), obstacle_avoid_vector_po[0]
        # print type(obstacle_avoid_vector_po[1]), obstacle_avoid_vector_po[1]
        print oa_pose
        print type(oa_pose)
        # print type(oa_pose.position)
        # print type(oa_pose.orientation)

        # self.cmd_vector_pub.publish(Pose(cmd_vector_po[0], cmd_vector_po[1]))
        # self.oa_cmd_combined_vector_pub.publish(Pose(combined_vector_po[0], combined_vector_po[1]))

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
