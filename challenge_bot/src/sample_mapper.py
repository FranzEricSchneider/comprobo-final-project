#!/usr/bin/env python

# Emily Wang, Eric Schneider
# Computational Robotics, Fall 2014, Olin College, taught by Paul Ruvolo
# This code will listen to the tf topic and call the service to update the map
# accordingly for samples.

import rospy

class SampleMapper():
    def __init__(self):
        pass

    def run(self):
        rospy.init_node("sample_mapper")
        

# run
# init node
# rospy.wait_for_service("add_sample_pos_to_map")
# add_sample_pos = rospy.ServiceProxy("add_sample_pos_to_map", )
# service proxy
# laser subscriber
# spin


if __name__ == '__main__':
    # test that jazz!
    try:
        node = SampleMapper()
        node.run()
    except rospy.ROSInterruptException: pass