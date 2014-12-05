from challenge_bot import *
from geometry_msgs.msg import Point
from waypoint import Waypoint
a = Waypoint(Point(0.9, 0, 0), 0.1)
b = Waypoint(Point(0.9, 0.9, 0), 0.1)
c = Waypoint(Point(0, 0.9, 0), 0.1)
d = Waypoint(Point(0.9, 1.8, 0), 0.1)
cb = ChallengeBot()
cb.drive_waypoints([a, b, c, d])