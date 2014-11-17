#!/usr/bin/env python

from geometry_msgs.msg import Twist, Vector3
from challenge_bot import ChallengeBot
from vector_tools import *

if __name__ == '__main__':
    robot = ChallengeBot()
    a = Vector3(1, 0, 0)
    robot.drive(a)