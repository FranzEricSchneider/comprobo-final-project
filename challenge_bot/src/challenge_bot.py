#!/usr/bin/env python

# Emily Wang, Eric Schneider
# Computational Robotics, Fall 2014, Olin College, taught by Paul Ruvolo
# This code is the main class code for a robot that is prototyping the NASA
# sample challenge on a Neato robotics platform

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from copy import deepcopy
from math import pi, cos, sin, atan2, copysign


class ChallengeBot():
    def __init__(self):
        rospy.init_node('ChallengeBot')

        # self.MAX_LINEAR = 0.075
        self.MIN_LINEAR = 0.0
        self.MAX_ANGULAR = 0.25
        self.MIN_ANGULAR = 0.0
        self.drive_commands = {"Right": Vector3(1, -1, 0),
                               "Left": Vector3(1, 1, 0),
                               "Forward": Vector3(1, 0, 0),
                               "Back": Vector3(-1, 0, 0)}

        self.last_scan = LaserScan()
        self.scan_hist = []
        self.hist_length = 10
        self.valid_points = {}
        self.valid_hist = []

        # Computed by /odom
        self.last_pos = Odometry()
        self.odom_hist = []

        # Has a moving object been detected?
        self.movement_detected = False
        # If movement has been detected, where is it w/ reference to the robot?
        self.movement_position = (0.0, 0.0)

        # Has a wall been detected?
        self.wall_detected = False
        # Goal distance in meters
        # Coordinates and angle of closest wall
        self.closest_wall = Twist()
        self.last_wall_detection = rospy.get_time()
        # Seconds w/o wall detection before wall_detected gets set to false
        self.wall_timeout = 0.5
        self.last_wall_left = True

        # Cutoff magnitudes below which no drive command will be published
        self.cmd_cutoff = 0.01
        self.avoid_cutoff = 0.1

        self.laser_sub = rospy.Subscriber('/scan', LaserScan,
                                          self.laser_callback)
        self.wall_sub = rospy.Subscriber('/detected_walls', Twist,
                                         self.detect_walls)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist)

    def laser_callback(self, msg):
        self.last_scan = deepcopy(msg)
        self.compute_valid()
        self.scan_hist.append(self.last_scan)
        self.odom_hist.append(self.last_pos)
        while len(self.scan_hist) > self.hist_length:
            self.scan_hist.pop(0)
        while len(self.odom_hist) > self.hist_length:
            self.odom_hist.pop(0)

        self.detect_movement()

        if (rospy.get_time() - self.last_wall_detection) > self.wall_timeout:
            self.wall_detected = False

    def odom_callback(self, msg):
        self.last_pos = deepcopy(msg)

    def detect_movement(self):
        # ang = self.find_point_changes()
        self.movement_detected = False

    def detect_walls(self, msg):
        if not (msg.linear.x == 0 and msg.linear.y == 0):
            self.wall_detected = True
            self.closest_wall = deepcopy(msg)
            self.last_wall_detection = rospy.get_time()
            # rospy.loginfo("Recieved wall: %s", str(self.closest_wall))
            if self.closest_wall.linear.y > 0.0:
                self.last_wall_left = True
            else:
                self.last_wall_left = False
            # rospy.loginfo("\tIs wall left?: %s", str(self.last_wall_left))
            # rospy.loginfo("\tWhat is the y value?: %f", msg.linear.y)

    # Takes the laser scan and creates a dictionary of the valid points, with
    # degree = key and value = value
    def compute_valid(self):
        self.valid_points = {}
        for i in range(len(self.last_scan.ranges)):
            if not self.last_scan.ranges[i] <= 0 and\
                    not self.last_scan.ranges[i] > 4.0:
                self.valid_points[i] = self.last_scan.ranges[i]

        self.valid_hist.append(self.valid_points)
        while len(self.valid_hist) > self.hist_length:
            self.valid_hist.pop(0)

    # TODO(fschneider): Unfinished
    # def find_point_changes(self):
    #     # Cycles to search back through
    #     search_depth = 1
    #     # Change in meters to be considered suspicious
    #     dx = 0.25
    #     count = 0
    #     success_count

    #     for i in range(search_depth):
    #         index = -1 - search_depth
    #         for point in self.valid_points.keys():
    #             if point in self.valid_hist[index].keys():
    #                 p1 = point_pos(point, self.valid_hist[index][point],
    #                                 ODOM_DATA[index])
    #                 p2 = point_pos(point, valid_points[point],
    #                                 ODOM_DATA)
    #                 if vector_mag(p1, p2) > dx:
    #                     count += 1

    # Publishes an empty vector, stopping the robot
    def stop(self):
        cmd = Twist()
        self.vel_pub.publish(cmd)

    def command_motors(self, cmd_vector):
        avoid_vector = self.obstacle_avoid()
        self.drive(Vector3(-1.0, 1, 0))
        if vector_mag(avoid_vector) < self.avoid_cutoff\
                and vector_mag(cmd_vector) > self.cmd_cutoff:
            self.drive(cmd_vector)
            # rospy.loginfo("No obstacles, following command")
        elif vector_mag(cmd_vector) < self.avoid_cutoff\
                and vector_mag(avoid_vector) > self.cmd_cutoff:
            self.drive(avoid_vector)
            # rospy.loginfo("No commmand, avoiding obstacles")
        elif vector_mag(cmd_vector) < self.cmd_cutoff\
                and vector_mag(avoid_vector) < self.avoid_cutoff:
            # rospy.loginfo("No input, no msg published")
            self.stop()
            pass
        else:
            self.drive(vector_add(cmd_vector, avoid_vector))
            # rospy.loginfo("Following command and avoiding obstacles")

    # Computes a Vector3 that points away from sensed obstacles, kicking in
    # strongly at "obstacle_sensitivity" meters
    def obstacle_avoid(self):
        v = Vector3()
        points = deepcopy(self.valid_points)
        max_reaction = 0
        for point in points.keys():
            reaction = self.obstacle_sensitivity - points[point]
            if reaction > max_reaction:
                max_reaction = reaction
            unit_vector = [cos(point * (pi / 180.0)),
                           sin(point * (pi / 180.0))]
            x_val = max(reaction, 0.0) * -unit_vector[0]
            y_val = max(reaction, 0.0) * -unit_vector[1]
            v = vector_add(v, Vector3(x_val, y_val, 0.0))

        if max([abs(v.x), abs(v.y)]) > 0:
            v = vector_multiply(v, self.avoid_scale * max_reaction
                                / max([abs(v.x), abs(v.y)]))
        else:
            v = Vector3()
        return v

    # Publishes a given vector command to the cmd_vel topic
    def drive(self, vector):
        cmd = Twist()
        ang = vector_ang(vector)

        # Angle to purely turn to reach, no forward velocity
        turn_angle = 90
        # Angle to start decreasing the forard speed, tighter turn
        max_speed_angle = 45

        if abs(ang) > turn_angle:
            cmd.linear.x = self.min_linear
        else:
            # Linear fit, 2.0x at 0, 1.0x at 45 and 0.0x at 90 degrees
            cmd.linear.x = (1 - ((abs(ang) - max_speed_angle)
                            / max_speed_angle)) * self.max_linear
            cmd.linear.x = min(cmd.linear.x, self.max_linear)

        if abs(ang) < turn_angle:
            cmd.angular.z = ang / turn_angle * self.max_angular
        else:
            cmd.angular.z = copysign(1, ang) * self.max_angular

        # rospy.logwarn('publishing: \n%s', str(cmd))
        self.vel_pub.publish(cmd)


def vector_add(v1, v2):
    v = Vector3()
    v.x = (v1.x + v2.x)
    v.y = (v1.y + v2.y)
    v.z = (v1.z + v2.z)
    return v


def vector_multiply(v1, scalar):
    v = Vector3()
    v.x = v1.x * scalar
    v.y = v1.y * scalar
    v.z = v1.z * scalar
    return v


def vector_mag(v):
    mag = pow(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2), 0.5)
    return mag


# Asngle assumes 2D vector, returns in degrees
# Vertical (0, 1) is an angle of 0, sweeps (+/-) going (CCW/CW)
def vector_ang(v):
    ang = -atan2(-v.y, v.x)
    return ang * (180 / pi)


def create_unit_vector(v1):
    v = Vector3()
    if vector_mag(v1) == 0:
        v.x = 1.0
    else:
        v.x = v1.x / vector_mag(v1)
        v.y = v1.y / vector_mag(v1)
    return v
