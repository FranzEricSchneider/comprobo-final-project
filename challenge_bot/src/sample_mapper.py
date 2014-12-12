#!/usr/bin/env python

# Emily Wang, Eric Schneider
# Computational Robotics, Fall 2014, Olin College, taught by Paul Ruvolo
# This code will listen to the tf topic and call the service to update the map
# accordingly for samples.

import rospy
import tf
import point_tools
import copy
from math import sin, cos
from geometry_msgs.msg import Point, Vector3
from challenge_msgs.srv import SamplePoint, SamplePointRequest
from vector_tools import *

class SampleMapper():
    def __init__(self):
        rospy.init_node("sample_mapper")
        rospy.loginfo("sample mapper node set up!")

        rospy.wait_for_service("add_sample_pos_to_map")
        self.add_sample_pos_service = rospy.ServiceProxy("add_sample_pos_to_map", SamplePoint)
        self.fiducials = {'a': None, 
                          'b': None, 
                          'c': None, 
                          'd': None, 
                          'g': None, 
                          'f': None}

        self.tf_listener = tf.TransformListener()
        self.pos_sub = rospy.Subscriber('/current_pos', Point, self.sample_pos_cb)

        rate = rospy.Rate(10.0)
        rospy.spin()

    def sample_lookupTransform(self, f_str, msg):
        """
        Does tf_listener.lookupTransform between camera_frame and f_str.
        Adds the sample onto the map if available (i.e. if that fiducial is seen in the camera).
        Returns nothing.
        """
        try: 
            # get info from tf
            (trans, rot) = self.tf_listener.lookupTransform('camera_frame', f_str, rospy.Time(0))

            # add tf values to current_pos message values to get the fiducial position
            # In the robot axes, the data comes in as (x_f, y_f, z_f) = (y_r, z_r, x_r)
            # in meters where x_f is x_fiducial and x_r is x_robot
            fiducial_pos = copy.deepcopy(msg)
            magnitude = vector_mag(Vector3(trans[2] + 0.17, trans[0], 0.0))
            local_angle = vector_ang(Vector3(trans[2] + 0.17, -trans[0], 0.0))
            global_angle = local_angle + fiducial_pos.z
            dx = cos(global_angle) * magnitude
            dy = sin(global_angle) * magnitude
            fiducial_pos.x += dx
            fiducial_pos.y += dy
            fiducial_pos.z = 0

            # add the fiducial to the map if it's not the most recent value
            if self.fiducials[f_str] != trans: # to prevent fiducial ghosting!! (i.e. last tf value will linger unless you check)
                # calls the service to update the map
                self.fiducials[f_str] = trans
                rospy.logdebug("Putting the sample %s's position on the map! (sample_lookupTransform)", f_str)
                sp = SamplePointRequest(fiducial_pos, f_str)
                self.add_sample_pos_service(sp)
        except:
            rospy.logdebug("You probably can't see %s in the camera frame. (sample_lookupTransform)", f_str)

    def sample_pos_cb(self, msg):
        for f in self.fiducials.keys():
            self.sample_lookupTransform(f, msg)

if __name__ == '__main__':
    # test that jazz!
    try:
        node = SampleMapper()
        node.run()
    except:
        rospy.logwarn("There's an issue with the sample mapper")
