#!/usr/bin/env python

# THERE'S A PROBLEM WITH THE CODE
# If you drive the robot forward, rotate the robot 90 degrees, and reset the
# position to the correct one, it tracks (x, y) incorrectly after that

# Emily Wang, Eric Schneider
# Computational Robotics, Fall 2014, Olin College, taught by Paul Ruvolo
# This code using a combination of odometry and human-provided data to publish
# an (x, y, theta) Point value for the robot
# Run from terminal like so: rosservice call /reset_pos '[1.0, 2.0, 3.1415]'

import numpy as np
from copy import deepcopy
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
from challenge_msgs.srv import ResetPosition, ResetPositionResponse
from vector_tools import add_angles, angle_difference
from point_tools import add_points, point_difference, array_to_point

class CurrentPosition():
    def __init__(self):
        rospy.init_node('odom_listener', anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.pos_pub = rospy.Publisher("/current_pos", Point, queue_size=1)

        self.raw_position = Point()
        self.position = Point()

        self.raw_position_at_update = Point()
        self.last_update_position = Point()
        self.latest_odom_to_actual = Point()
        # TODO: Remove the hard-coded values once this is live
        # self.last_update_position = Point(1, -1, 1.570796)
        # self.latest_odom_to_actual = Point(-1, 1, 2.35619449)
        # self.latest_odom_to_actual = Point(-0.25, 1, -0.5)
        # Correction matrix from odom at last update to the last update
        self.M = np.zeros((3, 3), dtype=np.float)
        self.calculate_correction_matrix()

        self.current_pos_server()

    def odom_cb(self, odom):
        """
        Takes in odom data, extracts (x, y, theta), and publishes a corrected
        version based on past publishing of corrected points
        """
        self.extract_raw_position_from_odom(odom)
        delta_odom = self.get_odom_change(self.raw_position_at_update,
                                          self.raw_position)
        print "Printing self.raw_position"
        print self.raw_position
        print "Printing self.raw_position_at_update"
        print self.raw_position_at_update
        print "Printing delta_odom"
        print delta_odom
        print ""
        self.pos_pub.publish(self.calculate_current_position(delta_odom))

    def extract_raw_position_from_odom(self, odom):
        """
        Takes in the odom data and extracts it to an (x, y, z) Point structure
        where z is equivalent to theta
        """
        pose = deepcopy(odom.pose.pose)
        self.raw_position.x = pose.position.x
        self.raw_position.y = pose.position.y
        rotation = (pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w)
        self.raw_position.z = euler_from_quaternion(rotation)[2]

    def get_odom_change(self, base_point, changed_point):
        """
        Calculates the change in (x, y, theta) from the current odom point to
        the last odom point at which the current_pos was updated
        """
        return point_difference(base_point, changed_point)

    def calculate_current_position(self, delta_odom):
        """
        Takes a given delta_odom from the last update point and translates the
        origin of the point to the last updated "correct" position
        """
        # TODO: Remove these helper calls
        # self.calculate_correction_matrix()
        # print "Printing the rotation matrix"
        # print self.M
        # print "Printing delta_odom"
        # print delta_odom
        actual_delta = np.dot(self.M,
                              np.array([delta_odom.x, delta_odom.y, 0.0]))
        actual_delta[2] = delta_odom.z
        self.position = add_points(array_to_point(actual_delta),
                                   self.last_update_position)
        # print "Printing actual_delta"
        # print actual_delta
        # print "Printing last_update_position"
        # print self.last_update_position
        # print "Printing current position"
        # print self.position
        return self.position

    def calculate_correction_matrix(self):
        """
        Construct the appropriate rotation matrix to correct from  the odometry
        at last update to the last correct update point. The Matrix is 3x3
        instead of 2x2 because later math requires the output to be 1x3
        """
        # TODO: Take out these print statements
        # print "Printing self.latest_odom_to_actual"
        # print self.latest_odom_to_actual
        self.M = np.zeros((3, 3), dtype=np.float)
        self.M[0, 0] = np.cos(self.latest_odom_to_actual.z)
        self.M[1, 1] = np.cos(self.latest_odom_to_actual.z)
        self.M[0, 1] = -np.sin(self.latest_odom_to_actual.z)
        self.M[1, 0] = np.sin(self.latest_odom_to_actual.z)

    def current_pos_server(self):
        service = rospy.Service('/reset_pos', ResetPosition,
                                self.set_current_offsets)
        rospy.loginfo("Set up the /reset_pos service")
        # TODO: Uncomment this spin
        rospy.spin()

    def set_current_offsets(self, req):
        """
        Saves human provided "actual" position. Used to correct odometry
        """
        self.raw_position_at_update = deepcopy(self.raw_position)
        self.last_update_position = deepcopy(req.point)
        self.latest_odom_to_actual = self.get_odom_change(self.raw_position,
                                                           req.point)
        self.calculate_correction_matrix()
        rospy.loginfo("Reset the offsets!")
        return ResetPositionResponse()


if __name__ == '__main__':
    cp = CurrentPosition()

