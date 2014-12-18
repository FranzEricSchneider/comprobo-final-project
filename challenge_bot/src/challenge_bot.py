#!/usr/bin/env python

# Emily Wang, Eric Schneider
# Computational Robotics, Fall 2014, Olin College, taught by Paul Ruvolo
# This code is the main class code for a robot that is prototyping the NASA
# sample challenge on a Neato robotics platform

import rospy
import tf
from geometry_msgs.msg import Twist, Vector3, Point
from nav_msgs.msg import OccupancyGrid
from copy import deepcopy
from math import copysign, pi, fabs, cos, sqrt, tan, atan2
from random import random

from waypoint import Waypoint
from map_tools import *
from publish_rviz_vector import *
from point_tools import *
from vector_tools import *
from challenge_msgs.srv import SendBool, OccupancyValue, OccupancyValueRequest


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
        self.map_value_clear = rospy.ServiceProxy("remove_specific_occupancy_val",
                                                  OccupancyValue)  

        self.display_obstacle_avoid = RVIZVector("ObstacleAvoid")
        self.display_command_vector = RVIZVector("CommandedValue")
        self.display_combined_vector = RVIZVector("CombinedValue")

        # key: value
        # fiducial's integer: point
        self.unclaimed_samples = {}
        self.claimed_samples = {}
        # Distance in meters from camera at which sample is considered claimed
        self.DISTANCE_TO_CLAIM = 0.40

        self.tf_listener = tf.TransformListener()
        self.SAMPLE_IDS = {50:'a', 60:'b', 70:'c', 80:'d', 90:'g', 100:'f'}
        # Stores the last tf transformation to check for new sightings
        self.last_tf = {50:(0, 0, 0), 60:(0, 0, 0), 70:(0, 0, 0),
                           80:(0, 0, 0), 90:(0, 0, 0), 100:(0, 0, 0)}

        # Logic for the SEEK behavior
        self.searched_areas = {'+x':(Point(1, 0, 0), 0.0),
                               '+y':(Point(0, 1, 0), 0.0),
                               '-x':(Point(-1, 0, 0), 0.0),
                               '-y':(Point(0, -1, 0), 0.0)}

        self.paused = False
        pause_s = rospy.Service('/pause_robot', SendBool, self.set_pause)

    def stop(self):
        """
        Publishes empty vector to stop robot
        """
        cmd = Twist()
        self.vector_pub.publish(cmd)

    def set_pause(self, req):
        self.paused = req.value
        return []

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

    def drive_angle(self, angle, scaling=1.0):
        """
        Drives a given angle, (CCW, CW) is (+/-). In radians
        """
        angle_cmd = Twist(angular=Vector3(0, 0, scaling*copysign(1.05, angle)))
        self.vector_pub.publish(angle_cmd)
        rospy.sleep(abs(angle)/float(scaling))
        self.stop()

    def point_robot_at_target(self, point):
        """
        Takes a Point on the map and tries to point the robot at it
        """
        delta_point = point_difference(self.current_pos, point)
        goal_angle = vector_ang(point_to_vector(delta_point))
        self.drive_angle(angle_difference(self.current_pos.z, goal_angle))

    def drive_robot(self, cmd_vector, avoid_obs=True):
        """
        Takes in a Vector3 direction in which to drive the robot, then layers
        the obstacle avoid vector on top of that, if asked to
        """
        combined_vector = vector_add(cmd_vector, self.obs_avoid_vector)
        self.display_vectors(deepcopy(cmd_vector), deepcopy(combined_vector))

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

        if self.paused:
            self.vector_pub.publish(Twist())
        else:
            self.vector_pub.publish(cmd)

    def drive_waypoints(self, waypoints):
        r = rospy.Rate(50)
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
        # rospy.loginfo("map_cb happening")
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
        self.display_command_vector.publish_marseekker(self.current_pos,
                                                   cmd)
        self.display_combined_vector.publish_marker(self.current_pos,
                                                    combine)

    # def seek(self):
    #     # TODO: Make smarter code that checks the map, finds the direction to
    #     # go, and adds a vector in that direction

    #     v = Vector3(random(), random()*2 - 1, 0)
    #     self.last_seek_cmd = create_unit_vector(vector_add(v,
    #                                                        self.last_seek_cmd))
    #     return self.last_seek_cmd

    def seek(self):
        seek_radius = 2
        seek_wp_radius = 0.25

        # point at waypoint, then drive to waypoint to prevent curvy arcs
 
        # drive to origin
        base_wp = Waypoint(Point(), seek_wp_radius)
        self.point_robot_at_target(base_wp.point)
        self.drive_waypoints([base_wp])
        drive_angle(2*pi, .5) # 360 degree turn, half speed so the bot can "look" at the surroundings

        # ERIC EXAMPLE, NOT FINISHED
        # area = self.get_least_explored_area()
        # out_wp = Waypoint(self.searched_areas[area][0], seek_wp_radius)
        # Then find some way to step the point out a bit?

        # testing just one corner first
        # once it works with one corner, do a for loop sort of thing
        # TODO: finish the details of this chunk of code:
        out_wp = Waypoint(Point(seek_radius, 0, 0), seek_wp_radius)
        self.point_robot_at_target(out_wp.point)
        self.drive_waypoints([out_wp])
        drive_angle(2*pi, .5) # 360 degree turn, half speed so the bot can "look" at the surroundings
        # check unclaimed_samples: is it empty or not? if not empty, let's grab it! otherwise, keep seeking!
        self.point_robot_at_target(base_wp.point)
        self.drive_waypoints([base_wp])

    def get_least_explored_area(self):
        least_area = '+x'
        least_radius = self.searched_areas['+x'][1]  # initializes search
        for area in self.searched_areas.keys():
            if self.searched_areas[area][1] < least_radius:
                least_radius = self.searched_areas[area][1]
                least_area = area
        return least_area

    def grab(self):
        # TODO: Flesh this case out
        goal = self.closest_sample()
        wp = Waypoint(self.unclaimed_samples[goal], 1)
        rospy.loginfo('Driving towards the nearest sample, %d, at \n%s',
                      goal, str(self.unclaimed_samples[goal]))
        self.drive_waypoints([wp])
        rospy.loginfo("Pointing robot at target after rough positioning")
        rospy.sleep(0.5)
        self.point_robot_at_target(wp.point)
        rospy.loginfo('Finished the rough positioning towards the sample')

        # Try for 10 cycles to find the fiducial
        sample_seen = False
        rospy.loginfo('Looking for %s sample after rough positioning',
                      self.SAMPLE_IDS[goal])

        for i in range(10):
            # try:
            rospy.sleep(0.5)
            sample_tf = self.tf_listener.lookupTransform('camera_frame',
                                                         self.SAMPLE_IDS[goal],
                                                         rospy.Time(0))
            if sample_tf == self.last_tf[goal]:
                rospy.logwarn("No sample %d seen after rough waypoint", i)
            else:
                rospy.loginfo("SUCCESS -- Saw the sample on cycle %d", i)
                sample_seen = True
                break
            # except:
            #     rospy.logwarn("Sample %d not seen before, try %d", goal, i)
        self.last_tf[goal] = deepcopy(sample_tf)

        if not sample_seen:
            # TODO: Implement avoid_point here
            reposition_wp = self.wp_around_sample(goal)
            rospy.loginfo('Driving around sample to waypoint \n%s',
                          str(wp.point))
            self.drive_angle(pi / 3.0)
            self.drive_waypoints([reposition_wp])
            rospy.loginfo("Pointing robot at target after driving around")
            rospy.sleep(0.5)
            self.point_robot_at_target(wp.point)
            rospy.loginfo('Finished driving around the waypoint')

        rospy.loginfo("Starting to reorient robot to the fiducial")
        rospy.sleep(0.25)
        if self.reorient_robot(goal):
            rospy.loginfo("Re-orient successful, trying to drive over")
            if self.drive_over(goal):
                self.drive_distance(0.75)
            else:
                self.stop()
        else:
            rospy.logwarn("Couldn't see a sample after reorienting")
            self.map_value_clear(OccupancyValueRequest(goal))
            self.unclaimed_samples.pop(goal)
            rospy.loginfo("Just cleared goal %d from the map", goal)

    def closest_sample(self):
        """
        Looks at the unclaimed_samples dictionary and returns the key for the
        closest sample
        """
        closest_key = self.unclaimed_samples.keys()[0]
        min_distance = pt_to_pt_distance(self.current_pos,
                                         self.unclaimed_samples[closest_key])
        for key in self.unclaimed_samples.keys():
            diff = pt_to_pt_distance(self.current_pos,
                                     self.unclaied_samples[key])
            if diff < min_distance:
                closest_key = key
                min_distance = diff
        return closest_key

    def wp_around_sample(self, goal_sample):
        """
        Creates a waypoint that is pi/4 radians around the sample from where
        the robot currently is, with a radius of RADIUS
        """
        RADIUS = 0.1

        # This point and vector juggling is silly, but easy :P
        offset_point = point_difference(self.current_pos,
                                        self.unclaimed_samples[goal_sample])
        offset_vector = point_to_vector(offset_point)
        new_vector = create_angled_vector(add_angles(vector_ang(offset_vector),
                                                     pi / 4),
                                          pow(2 * pow(vector_mag(offset_vector), 2), 0.5))
        return Waypoint(add_points(self.current_pos,
                                   vector_to_point(new_vector)),
                                   RADIUS)

    def reorient_robot(self, goal):
        """
        Function for testing information for the "sample seen" case
        Will be fused back into the grab function when testing is complete

        """
        try:
            sample_tf = self.tf_listener.lookupTransform('camera_frame',
                                                         self.SAMPLE_IDS[goal],
                                                         rospy.Time(0))
            if sample_tf == self.last_tf[goal]:
                rospy.loginfo("No sample %d, aborting re-orient", goal)
                self.last_tf[goal] = deepcopy(sample_tf)
                return False
            else:
                # how far away + how much turning needs to happen
                sample_trans = sample_tf[0]
                sample_rot = sample_tf[1]
                sample_rot = tf.transformations.euler_from_quaternion(sample_rot)

                rospy.logdebug("sample_trans:\n%s\nsample_rot:\n%s",
                                str(sample_trans), str(sample_rot))

                # turn to a point perpendicular to sample
                angle_to_perp = copysign(1,sample_rot[1]) * \
                                (pi/2 - abs(sample_rot[1]))
                self.drive_angle( angle_to_perp )

                x = sample_trans[2] + 0.17
                y = -sample_trans[0]
                theta = sample_rot[1]
                phi = angle_to_perp
                if x * tan(abs(theta)) < abs(y):
                    sign = -1
                else:
                    sign = 1

                # drive to a point perpendicular to sample
                distance_to_perp = sign * abs(sin(theta + atan2(y, x)) * \
                                   sqrt(x**2 + y**2))
                self.drive_distance(distance_to_perp)

                # turn towards sample (should be roughly pi/2)
                rospy.sleep(0.25)
                self.drive_angle( -copysign(1,sample_rot[1]) * pi/2 )
                self.last_tf[goal] = deepcopy(sample_tf)
                return True
        except:
            rospy.logwarn("Sample %d not seen before", goal)
            return False

    def drive_over(self, goal):
        """
        Takes a fiducial and drives the robot at that fiducial, trying to
        control the robot so that it is always parallel to the fiducial.
        Checks the fiducial as a claimed sample when the distance is under a
        certain amount. Returns a boolean for success
        """
        # Proportional control constant
        k_p = 2
        # The goal is to have the robot parallel to the fiducial
        goal_angle = 0.0

        if goal in self.claimed_samples.keys():
            rospy.loginfo("The sample was already claimed")
            return True
        elif goal not in self.unclaimed_samples.keys():
            rospy.logwarn("The sample was not in unclaimed_samples")
            return False

        sample_claimed = False
        failures = 0
        while not sample_claimed and failures < 20\
              and not rospy.is_shutdown():
            try:
                sample_tf = self.tf_listener.lookupTransform('camera_frame',
                                                                self.SAMPLE_IDS[goal],
                                                                rospy.Time(0))
                trans = sample_tf[0]
                rot = sample_tf[1]
                rot = tf.transformations.euler_from_quaternion(rot)
                if sample_tf == self.last_tf[goal]:
                    failures += 1
                    rospy.logwarn("Sample not seen in driveover! Failures: %d",
                                  failures)
                    self.stop()
                    rospy.sleep(0.25)
                else:
                    rospy.loginfo("Distance: %f", trans[2])
                    if trans[2] <= self.DISTANCE_TO_CLAIM:
                        rospy.loginfo("The sample is close enough to claim!")
                        self.stop()
                        self.claimed_samples[goal] = self.unclaimed_samples[goal]
                        self.unclaimed_samples.pop(goal)
                        sample_claimed = True
                    else:
                        rospy.loginfo("Driving robot at angle %f",
                                      k_p * (goal_angle - rot[1]))
                        v = create_angled_vector(k_p * (goal_angle - rot[1]))
                        self.drive(v)
                        rospy.sleep(0.1)
                self.last_tf[goal] = deepcopy(sample_tf)
            except:
                rospy.logwarn("Sample %d not seen before", goal)
        return sample_claimed
