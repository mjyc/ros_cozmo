#! /usr/bin/env python3

import sys
import cozmo
import rospy
import actionlib
import actionlib_tutorials.msg


class CozmoAction(object):
    # create messages that are used to publish feedback/result
    _feedback = actionlib_tutorials.msg.FibonacciFeedback()
    _result = actionlib_tutorials.msg.FibonacciResult()

    def __init__(self, name, coz):
        self._robot = coz

        self._action_name = name

        self._goal = None

        self._as = actionlib.SimpleActionServer(
            self._action_name,
            actionlib_tutorials.msg.FibonacciAction,
            auto_start=False
        )
        self._as.register_goal_callback(self._goal_cb)
        self._as.register_preempt_callback(self._preempt_cb)
        self._as.start()

    def _goal_cb(self):
        rospy.loginfo("Goal accepted")
        self._goal = self._as.accept_new_goal()
        a = self._robot.say_text('hello world hello world hello world')
        a.wait_for_completed()
        print(a)
        # result = actionlib_tutorials.msg.FibonacciResult()
        self._as.set_succeeded(self._result)

    def _preempt_cb(self):
        if self._as.is_active():
            rospy.loginfo("People detector preempted")
            self._as.set_preempted()

    # def execute_cb(self, goal):
    #     # helper variables
    #     r = rospy.Rate(1)
    #     success = True

    #     # append the seeds for the fibonacci sequence
    #     self._feedback.sequence = []
    #     self._feedback.sequence.append(0)
    #     self._feedback.sequence.append(1)

    #     # publish info to the console for the user
    #     rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))

    #     # start executing the action
    #     for i in range(1, goal.order):
    #         # check that preempt has not been requested by the client
    #         if self._as.is_preempt_requested():
    #             rospy.loginfo('%s: Preempted' % self._action_name)
    #             self._as.set_preempted()
    #             success = False
    #             break
    #         self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i - 1])
    #         # publish the feedback
    #         self._as.publish_feedback(self._feedback)
    #         # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
    #         r.sleep()

    #     if success:
    #         self._result.sequence = self._feedback.sequence
    #         rospy.loginfo('%s: Succeeded' % self._action_name)
    #         self._as.set_succeeded(self._result)


def run(coz_conn):
    '''The run method runs once Cozmo is connected.'''
    coz = coz_conn.wait_for_robot()

    rospy.init_node('cozmo')
    CozmoAction(rospy.get_name(), coz)
    rospy.spin()


if __name__ == '__main__':
    try:
        cozmo.connect(run)
    except cozmo.ConnectionError as e:
        sys.exit("A connection error occurred: %s" % e)
