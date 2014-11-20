#!/usr/bin/env python

# THERE'S A PROBLEM WITH THE CODE
# If you drive the robot forward, rotate the robot 90 degrees, and reset the
# position to the correct one, it tracks (x, y) incorrectly after that

# Emily Wang, Eric Schneider
# Computational Robotics, Fall 2014, Olin College, taught by Paul Ruvolo
# This code using a combination of odometry and human-provided data to publish
# an (x, y, theta) Point value for the robot
# Run from terminal like so: rosservice call /reset_pos '[1.0, 2.0, 3.1415]'

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
from challenge_msgs.srv import ResetPosition, ResetPositionResponse
from vector_tools import increment_angle, angle_difference

class CurrentPosition():
    def __init__(self):
        rospy.init_node('odom_listener', anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.pos_pub = rospy.Publisher("/current_pos", Point, queue_size=1)

        self.offset = Point()
        self.position = Point()
        self.current_pos_server()

    def odom_cb(self, odom):
        """
        Takes in odom data, extracts (x, y, theta), and publishes it
        """
        self.position.x = odom.pose.pose.position.x + self.offset.x
        self.position.y = odom.pose.pose.position.y + self.offset.y

        rotation = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                    odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
        self.position.z = increment_angle(euler_from_quaternion(rotation)[2],
                                          self.offset.z)
        self.pos_pub.publish(self.position)

    def set_current_offsets(self, req):
        """
        Saves human provided "actual" position. Used to correct odometry
        """
        self.offset.x += req.point.x - self.position.x
        self.offset.y += req.point.y - self.position.y
        self.offset.z += angle_difference(self.position.z, req.point.z)
        rospy.loginfo("Reset the offsets!")
        return ResetPositionResponse()

    def current_pos_server(self):
        service = rospy.Service('/reset_pos', ResetPosition,
                                self.set_current_offsets)
        rospy.loginfo("Set up the /reset_pos service")
        rospy.spin()


if __name__ == '__main__':
    cp = CurrentPosition()
