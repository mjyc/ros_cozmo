#! /usr/bin/env python3

import sys
import cozmo
import actionlib
import rospy
import ros_cozmo.msg

class CozmoBridge(object):
    def __init__(self, coz):
        self._robot = coz
        self._servers = {}
        self._actions = {}

    @property
    def servers(self):
        return self._servers

    def createActionServer(self, name, Action):
        self._servers[name] = actionlib.SimpleActionServer(
            name,
            Action,
            auto_start=False
        )

        '''Cancels the running goal on receiving a new goal'''
        def goal_cb():
            if self._servers[name].is_active():
                print("2")
                self._actions[name].abort()
            goal = self._servers[name].accept_new_goal()
            args = dict((key, getattr(goal, key)) for key in goal.__slots__)
            if "in_parallel" not in args:
                args["in_parallel"] = True
            robot_method = getattr(self._robot, name.strip("/"))
            self._actions[name] = robot_method(**args)

            def callback(evt, **kwargs):
                print("evt %s" % (evt))
                print("kwargs %s" % (kwargs))
                del self._actions[name]
                self._servers[name].set_succeeded()
            self._actions[name].on_completed(callback)

        def _preempt_cb(self):
            print("1")
            if self._servers[name].is_active():
                print("2")
                self._actions[name].abort()
                self._servers[name].set_preempted()

        self._servers[name].register_goal_callback(goal_cb)
        # server.register_preempt_callback(self._preempt_cb)

        return self._servers[name]

def run(coz_conn):
    '''The run method runs once Cozmo is connected.'''
    coz = coz_conn.wait_for_robot()

    rospy.init_node("cozmo", log_level=rospy.DEBUG)

    cozmo_bridge = CozmoBridge(coz)
    cozmo_bridge.createActionServer('/say_text', ros_cozmo.msg.SayTextAction)
    for name, server in cozmo_bridge.servers.items():
        server.start()

    rospy.spin()

if __name__ == "__main__":
    try:
        cozmo.connect(run)
    except cozmo.ConnectionError as e:
        sys.exit("A connection error occurred: %s" % e)
