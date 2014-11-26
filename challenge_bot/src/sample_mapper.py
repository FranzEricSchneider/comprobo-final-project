#!/usr/bin/env python

# Emily Wang, Eric Schneider
# Computational Robotics, Fall 2014, Olin College, taught by Paul Ruvolo
# This code will listen to the tf topic and call the service to update the map
# accordingly for samples.

import rospy

class SampleMapper():
    def __init__(self):
        rospy.init_node("sample_mapper")
        rospy.wait_for_service("add_sample_pos_to_map")
        add_sample_pos = rospy.ServiceProxy("add_sample_pos_to_map", PointRequest.srv)
        #tf stuff
        rospy.loginfo("sample mapper!")
        rospy.spin()

    def sample_pos_cb(self):
        # digesting tf information

        # calls the service to update the map


if __name__ == '__main__':
    # test that jazz!
    try:
        node = SampleMapper()
        node.run()
    except rospy.ROSInterruptException: pass