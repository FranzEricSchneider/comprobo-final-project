#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from challenge_msgs.srv import PointRequest, PointRequestRequest

class PositionMapper():
    def __init__(self):
        rospy.init_node("PositionMapper")
        rospy.wait_for_service('add_pos_to_map')
        self.map_pos_service = rospy.ServiceProxy('add_pos_to_map', PointRequest)
        self.pos_sub = rospy.Subscriber('/current_pos', Point, self.pos_cb)
        rospy.loginfo("Beginning the current_pos mapping node")
        rospy.spin()

    def pos_cb(self, msg):
        """
        Grabs incoming /current_pos Point data and sends it to the map
        """
        try:
            self.map_pos_service(PointRequestRequest(msg))
        except Exception, e:
            print e


if __name__ == '__main__':
    try:
        pm = PositionMapper()
    except:
        rospy.logwarn('Experienced an error in pos_mapper')
