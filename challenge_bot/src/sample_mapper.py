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
        
        self.tf_listener = tf.TransformListener()     
        self.pos_sub = rospy.Subscriber('/current_pos', Point, self.sample_pos_cb)

        rate = rospy.Rate(10.0)
        rospy.spin()

    def sample_pos_cb(self, msg):
        # digesting tf information
        # try:
        # get info from tf
        (trans, rot) = self.tf_listener.lookupTransform('camera_frame', 'f', rospy.Time(0))
        fiducial_str = "f"

        # add tf values to current_pos message values to get the fiducial position
        fiducial_pos = copy.deepcopy(msg)
        fiducial_pos.x += trans[2]
        fiducial_pos.y -= trans[0]
        fiducial_pos.z += trans[1]

        # # calls the service to update the map
        rospy.loginfo("Putting the sample position on the map!")
        # print type(fiducial_pos)
        # print type(fiducial_str)
        sp = SamplePointRequest(fiducial_pos, fiducial_str)
        print type(sp)
        print sp
        self.add_sample_pos_service(sp)

        # except:
        #     rospy.loginfo("sample_pos_cb issues")

if __name__ == '__main__':
    # test that jazz!
    try:
        node = SampleMapper()
        node.run()
    except:
        rospy.logwarn("there's an issue with the sample mapper")