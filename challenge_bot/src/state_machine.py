#!/usr/bin/env python

# Written by Eric Schneider for the CompRobo warmup project
# Computational Robotics, Fall 2014, Olin College, taught by Paul Ruvolo
#
# Stuff about how it works...
#
# In the code below I couldn't get the input_keys methods to apprpriately
# pass a class object. The sm_robot class was either not passed into the
# states, or it was passed as a constant. I left the code in there for
# documentation and in case I could get it working later

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Twist, Vector3
from copy import deepcopy

from challenge_bot import ChallengeBot
from vector_tools import *
challenger = ChallengeBot()


# define state Return
class Seek(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempt-avoid', 'finished'],
                             input_keys=['follow_robot'])
        self.result = ''

    def execute(self, userdata):
        global challenger
        rospy.loginfo('Executing state SEEK')

        if challenger.movement_detected:
            self.result = 'preempt-avoid'
            return

        if challenger.last_wall_left:
            cmd = "Left"
        else:
            cmd = "Right"
        challenger.command_motors(challenger.drive_commands[cmd])
        rospy.loginfo("Sending bot seeking to the %s", cmd)
        rospy.sleep(1.0)
        challenger.command_motors(challenger.drive_commands["Forward"])
        rospy.sleep(0.5)
        challenger.stop()

        self.result = 'finished'
        rospy.loginfo("SEEK state is returning %s\n", self.result)
        challenger.stop()
        return self.result


# define state SeekAndGet
class SeekAndGet(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempt-avoid', 'preempt-follow',
                             'finished'], input_keys=['idle_robot'])
        self.result = ''

    def execute(self, userdata):
        global challenger
        rospy.loginfo('Executing state SEEK_AND_GET')

        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if challenger.movement_detected:
                self.result = 'preempt-avoid'
                break
            elif challenger.wall_detected:
                self.result = 'preempt-follow'
                break
            else:
                pass
                v = Vector3()
                challenger.command_motors(v)
                r.sleep()

        rospy.loginfo("SEEK_AND_GET state is returning %s\n", self.result)
        return self.result


# define state Return
class Return(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['preempt-avoid', 'finished'],
                             input_keys=['follow_robot'])
        self.result = ''

    def execute(self, userdata):
        global challenger
        rospy.loginfo('Executing state FOLLOW')

        cmd_align = Vector3()
        cmd_approach = Vector3()
        wall = Twist()
        r = rospy.Rate(10)

        while challenger.wall_detected and not rospy.is_shutdown():
            if challenger.movement_detected:
                self.result = 'preempt-avoid'
                break
            wall = deepcopy(challenger.closest_wall)

            cmd_align.x = cos(wall.angular.z)
            cmd_align.y = sin(wall.angular.z)
            error = vector_mag(wall.linear) - challenger.goal_distance
            cmd_approach = vector_multiply(create_unit_vector(wall.linear),
                                           error)
            # rospy.loginfo("cmd_align: \n%s", cmd_align)
            # rospy.loginfo("cmd_approach: \n%s", cmd_approach)

            challenger.command_motors(vector_add(cmd_align, cmd_approach))
            r.sleep()

        self.result = 'finished'
        rospy.loginfo("FOLLOW state is returning %s\n", self.result)
        challenger.stop()
        return self.result


# define state Climb
class Climb(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished'],
                             input_keys=['avoid_robot'])
        self.result = ''

    def execute(self, userdata):
        global challenger
        rospy.loginfo('Executing state CLIMB')

        self.result = 'finished'
        rospy.loginfo("CLIMB state is returning %s\n", self.result)
        return self.result


def main():
    sm = smach.StateMachine(outcomes=['sm-finished'])

    with sm:
        smach.StateMachine.add('SEEK', Seek(),
                               transitions={'preempt-avoid': 'CLIMB',
                                            'finished': 'SEEK_AND_GET'},
                               remapping={'follow_robot': 'sm_robot'})
        smach.StateMachine.add('SEEK_AND_GET', SeekAndGet(),
                               transitions={'preempt-avoid': 'CLIMB',
                                            'preempt-follow': 'FOLLOW',
                                            'finished': 'sm-finished'},
                               remapping={'idle_robot': 'sm_robot'})
        smach.StateMachine.add('FIND', Find(),
                               transitions={'preempt-avoid': 'CLIMB',
                                            'finished': 'FOLLOW'},
                               remapping={'find_robot': 'sm_robot'})
        smach.StateMachine.add('CLIMB', Climb(),
                               transitions={'alarm': 'CLIMB',
                                            'false-alarm': 'SEEK_AND_GET'},
                               remapping={'wary_robot': 'sm_robot'})

    outcome = sm.execute()
    rospy.spin()


if __name__ == '__main__':
    main()
