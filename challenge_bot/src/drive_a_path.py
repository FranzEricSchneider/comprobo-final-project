#!/usr/bin/env python

from challenge_bot import ChallengeBot
from waypoint import Waypoint
from geometry_msgs.msg import Point

# waypoints 
# forward is +x
# left is +y

# for comprobo design review demo
a = Waypoint(Point(0, 0, 0),.1)     #start
b = Waypoint(Point(1, 1, 0),.1)     #one forward
c = Waypoint(Point(1.5, 1, 0),.1)   #one forward - sees C
d = Waypoint(Point(1.5, 2, 0),.1)   #one left - sees B

cb = ChallengeBot()
cb.drive_waypoints([a, b, c, d])

