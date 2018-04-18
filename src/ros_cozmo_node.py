#! /usr/bin/env python3

import sys
from functools import partial
import cozmo
import actionlib
import rospy
import ros_cozmo.msg

# import time
import asyncio


class CozmoAction(object):

    def __init__(self, name, coz, Action):
        self._robot = coz
        self._action_name = name

        self._goal = None

        self._action = None

        self._as = actionlib.SimpleActionServer(
            self._action_name,
            Action,
            auto_start=False
        )
        self._as.register_goal_callback(self._goal_cb)
        self._as.register_preempt_callback(self._preempt_cb)
        self._as.start()

    def _goal_cb(self):
        rospy.loginfo("Goal accepted")
        goal = self._as.accept_new_goal()
        robot_action_args = dict((key, getattr(goal, key)) for key in goal.__slots__)
        if "in_parallel" not in robot_action_args:
            robot_action_args["in_parallel"] = True
        robot_action_name = self._action_name.strip("/")
        robot_action = getattr(self._robot, robot_action_name)
        # self._action =
        action = robot_action(**robot_action_args)

        def fx(evt, **kwargs):
            print(evt, kwargs)
            print("done!!")
        action.on_completed(fx)

        # self._action.on_completed()
        # print(self._action)

        # self._action.wait_for_completed()
        # print("dooone!")
        # time.sleep(1.5)
        # self._as.set_succeeded()

    def _preempt_cb(self):
        rospy.loginfo("preempt called")
        # print(self._action)
        # print(self._as.is_active())
        # print(self._as.is_active())
        pass
        # raise NotImplementedError()
        # if self._as.is_active():
        #     rospy.loginfo("Action preempted")
        #     self._as.set_preempted()


class Cozmo(object):
    def __init__(self, coz):
        self._robot = coz

        def gcb(self):
            goal = self._as.accept_new_goal()
            print(goal)
            g = goal
            args = dict((key, getattr(g, key)) for key in g.__slots__)
            self._robot.say_text(**args).wait_for_completed()
            self._as.set_succeeded()

        SayTextCozmoAction = type(
            "SayTextCozmoAction",
            (CozmoAction,),
            {}
        )
        server = SayTextCozmoAction("/say_text", coz, ros_cozmo.msg.SayTextAction)


def run(coz_conn):
    "The run method runs once Cozmo is connected."
    coz = coz_conn.wait_for_robot()

    rospy.init_node("cozmo", log_level=rospy.DEBUG)
    print("rospy.get_name()", rospy.get_name())
    Cozmo(coz)
    rospy.spin()


if __name__ == "__main__":
    try:
        cozmo.connect(run)
    except cozmo.ConnectionError as e:
        sys.exit("A connection error occurred: %s" % e)
