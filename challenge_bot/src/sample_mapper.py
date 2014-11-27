#!/usr/bin/env python

# Emily Wang, Eric Schneider
# Computational Robotics, Fall 2014, Olin College, taught by Paul Ruvolo
# This code will listen to the tf topic and call the service to update the map
# accordingly for samples.

import rospy
import tf
import point_tools
import copy
from geometry_msgs.msg import Point
from challenge_msgs.srv import SamplePoint, SamplePointRequest

class SampleMapper():
    def __init__(self):
        rospy.init_node("sample_mapper")
        rospy.loginfo("sample mapper node setup!")

        rospy.wait_for_service("add_sample_pos_to_map")
        self.add_sample_pos_service = rospy.ServiceProxy("add_sample_pos_to_map", SamplePoint)
        self.fiducials = ['a', 'b', 'c', 'd', 'g', 'f']
        
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
            fiducial_pos = copy.deepcopy(msg)
            fiducial_pos.x += trans[2]
            fiducial_pos.y -= trans[0]
            fiducial_pos.z += trans[1]

            # calls the service to update the map
            rospy.loginfo("Putting the sample position on the map!")
            sp = SamplePointRequest(fiducial_pos, f_str)
            self.add_sample_pos_service(sp)
        except:
            rospy.loginfo("You probably can't see %s in the camera frame. (sample_lookupTransform)", f_str)

    def sample_pos_cb(self, msg):
        for f in self.fiducials:
            self.sample_lookupTransform(f, msg)

if __name__ == '__main__':
    # test that jazz!
    try:
        node = SampleMapper()
        node.run()
    except:
        rospy.logwarn("There's an issue with the sample mapper")
