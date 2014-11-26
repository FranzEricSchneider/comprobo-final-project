#!/usr/bin/env python

# Emily Wang, Eric Schneider
# Computational Robotics, Fall 2014, Olin College, taught by Paul Ruvolo
# This code will listen to the tf topic and call the service to update the map
# accordingly for samples.

import rospy
import tf
from geometry_msgs.msg import Point
from challenge_msgs.srv import PointRequest, PointRequestRequest

class SampleMapper():
    def __init__(self):
        rospy.init_node("sample_mapper")
        rospy.wait_for_service("add_sample_pos_to_map")
        self.add_sample_pos_service = rospy.ServiceProxy("add_sample_pos_to_map", PointRequest)
        self.pos_sub = rospy.Subscriber('/current_pos', Point, self.pos_cb)
        self.tf_listener = tf.TransformListener()     
        rospy.loginfo("starting up the sample mapper node!")
        # rospy.spin()

    def sample_pos_cb(self):
        # digesting tf information

        # calls the service to update the map
        pass

    def run(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (trans_a, rot_a) = self.tf_listener.lookupTransform('camera_frame', 'a', rospy.Time(0))
                (trans_b, rot_b) = self.tf_listener.lookupTransform('camera_frame', 'b', rospy.Time(0))
                (trans_c, rot_c) = self.tf_listener.lookupTransform('camera_frame', 'c', rospy.Time(0))
                (trans_d, rot_d) = self.tf_listener.lookupTransform('camera_frame', 'd', rospy.Time(0))
                (trans_g, rot_g) = self.tf_listener.lookupTransform('camera_frame', 'g', rospy.Time(0))
                (trans_f, rot_f) = self.tf_listener.lookupTransform('camera_frame', 'f', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("tf lookupTransform issues")

if __name__ == '__main__':
    # test that jazz!
    try:
        node = SampleMapper()
        node.run()
    except:
        rospy.logwarn("there's an issue with the sample mapper")