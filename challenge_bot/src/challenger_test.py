#!/usr/bin/env python

from geometry_msgs.msg import Twist, Vector3
from challenge_bot import ChallengeBot
from vector_tools import *
robot = ChallengeBot()
robot.drive_distance(0.5)

# robot.drive_angle(1.57)

# a = Vector3(1, 0, 0)
# robot.drive(a)