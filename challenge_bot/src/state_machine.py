#!/usr/bin/env python

# Written by Eric Schneider for the CompRobo warmup project
# Computational Robotics, Fall 2014, Olin College, taught by Paul Ruvolo
# This code contains the upper level states for how our NASA sample challenge
# prototype chooses what to do and how it transitions from state to state.
# Most of the logic once the state has been chosen is in the robot class

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Twist, Vector3
from copy import deepcopy

from challenge_bot import ChallengeBot
from vector_tools import *
challenger = ChallengeBot()


class Seek(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['has-sample', 'timeout'],
                             input_keys=['seek'])
        self.result = ''
        # TODO: Replace this timeout with one that depends on distance
        # Time left (s) when SEEK will give up and return
        self.SEEK_TIMEOUT = 1.0 * 60.0

    def execute(self, userdata):
        global challenger
        rospy.loginfo('Executing state SEEK')
        r = rospy.Rate(15)

        while not rospy.is_shutdown():
            if len(challenger.unclaimed_samples) > 0:
                self.result = 'has-sample'
                break
            elif challenger.time_left() < self.SEEK_TIMEOUT:
                self.result = 'timeout'
                break
            else:
                drive_cmd = challenger.seek()
                challenger.drive_robot(drive_cmd)
                r.sleep()
        rospy.loginfo("Returning from %s with result %s",
                      self.__class__.__name__, self.result)
        return self.result


class Grab(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['no-sample', 'timeout'],
                             input_keys=['grab'])
        self.result = ''
        # TODO: Replace this timeout with one that depends on distance
        # Time left (s) when GRAB will give up and return
        self.GRAB_TIMEOUT = 0.75 * 60.0

    def execute(self, userdata):
        global challenger
        rospy.loginfo('Executing state GRAB')
        r = rospy.Rate(5)

        while not rospy.is_shutdown():
            # TODO: Change this structure so that grab is a self-contained
            # function that returns "timeout" or "no-sample"
            if len(challenger.unclaimed_samples) <= 0:
                self.result = 'no-sample'
                break
            elif challenger.time_left() < self.GRAB_TIMEOUT:
                self.result = 'timeout'
                break
            else:
                rospy.loginfo('Executing action GRAB')
                challenger.grab(self.GRAB_TIMEOUT)
                r.sleep()
        rospy.loginfo("Returning from %s with result %s",
                      self.__class__.__name__, self.result)      
        return self.result


class Return(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['has-sample', 'climb', 'finished'],
                             input_keys=['return'])
        self.result = ''
        # TODO: Link this to the GRAB timeout somehow legit
        # Time left (s) when GRAB will give up and return
        self.GRAB_TIMEOUT = 0.75 * 60.0

    def execute(self, userdata):
        global challenger
        rospy.loginfo('Executing state RETURN')
        self.result = 'finished'
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            # TODO: Add a case to the while loop that breaks on success
            if len(challenger.unclaimed_samples) > 0\
               and challenger.time_left() > self.GRAB_TIMEOUT:
                self.result = 'has-sample'
                break
            else:
                challenger.drive_robot(challenger.return_to_base())
                if challenger.base_wp.is_complete(challenger.current_pos):
                    break
                r.sleep()

        rospy.loginfo("Completed these fiducials!\n%s",
                      str([challenger.SAMPLE_IDS[pt] for pt in challenger.claimed_samples.keys()]))
        challenger.stop()
        rospy.loginfo("Returning from %s with result %s",
              self.__class__.__name__, self.result)
        return self.result


class Climb(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished'],
                             input_keys=['climb'])
        self.result = ''

    def execute(self, userdata):
        global challenger
        rospy.loginfo('Executing state CLIMB')
        r = rospy.Rate(5)
        self.result = 'finished'

        while not rospy.is_shutdown():
            # TODO: Add a case to the while loop that breaks on success
            # TODO: Write CLIMB logic
            rospy.loginfo("This is where the robot should climb!")
            r.sleep()

        rospy.loginfo("Returning from %s with result %s",
                      self.__class__.__name__, self.result)
        return self.result


def main():
    sm = smach.StateMachine(outcomes=['sm-finished'])

    with sm:
        smach.StateMachine.add('SEEK', Seek(),
                               transitions={'has-sample': 'GRAB',
                                            'timeout': 'RETURN'})
        smach.StateMachine.add('GRAB', Grab(),
                               transitions={'no-sample': 'SEEK',
                                            'timeout': 'RETURN'})
        smach.StateMachine.add('RETURN', Return(),
                               transitions={'has-sample': 'GRAB',
                                            'climb': 'CLIMB', 
                                            'finished': 'sm-finished'})
        smach.StateMachine.add('CLIMB', Climb(),
                               transitions={'finished': 'sm-finished'})

    outcome = sm.execute()
    rospy.spin()


if __name__ == '__main__':
    main()
