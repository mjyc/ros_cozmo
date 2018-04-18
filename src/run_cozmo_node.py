#! /usr/bin/env python3

import sys
import cozmo
import actionlib
import rospy
import ros_cozmo.msg


class CozmoHandler(object):

    def __init__(self, coz):
        self._robot = coz
        self._servers = {}
        self._cozmo_actions = {}

    @property
    def servers(self):
        return self._servers

    def createActionServer(self, name, Action):
        self._servers[name] = actionlib.SimpleActionServer(
            name,
            Action,
            auto_start=False
        )

        def start_cozmo_action():
            server = self._servers[name]

            rospy.logdebug('before SimpleActionServer.accept_new_goal()=%s' % (server.current_goal.get_goal()))
            goal = server.accept_new_goal()
            rospy.logdebug('after SimpleActionServer.accept_new_goal()=%s' % (server.current_goal.get_goal()))
            args = dict((key, getattr(goal, key)) for key in goal.__slots__)
            if "in_parallel" not in args:
                args["in_parallel"] = True
            # TODO: wrap getattr & cozmo_action(**args) with try & catch
            cozmo_action = getattr(self._robot, name.strip("/"))
            self._cozmo_actions[name] = cozmo_action(**args)
            self._cozmo_actions[name].add_event_handler(
                cozmo.action.EvtActionCompleted,
                on_complete_cb
            )

        def on_complete_cb(evt, **kwargs):
            server = self._servers[name]

            rospy.logdebug("evt=%s, kwargs=%s" % (evt, kwargs))
            rospy.logdebug("SimpleActionServer.is_new_goal_available()=%s" % (server.is_new_goal_available()))
            rospy.logdebug("SimpleActionServer.is_preempt_requested()=%s" % (server.is_preempt_requested()))

            if server.is_new_goal_available():
                start_cozmo_action()
            else:
                if evt.state == cozmo.action.ACTION_SUCCEEDED:
                    server.set_succeeded()
                else:
                    if server.is_preempt_requested():
                        # NOTE: "set_preempted" does not reset "preempt_request"
                        #   fast enough
                        server.preempt_request = False
                        server.set_preempted()
                    else:
                        server.set_aborted()

        def goal_cb():
            server = self._servers[name]

            rospy.logdebug('SimpleActionServer.is_preempt_requested()=%s' % (self._servers[name].is_preempt_requested()))

            # If no action was running, preempt would not have been requested
            if not server.is_preempt_requested():
                start_cozmo_action()
            # Otherwise, "on_complete_cb" in "preempt_cb" will start a new action

        def preempt_cb():
            # NOTE: This function is called before "goal_cb" if the server is active
            server = self._servers[name]
            cozmo_action = self._cozmo_actions[name]

            if server.is_active():
                cozmo_action.remove_event_handler(
                    cozmo.action.EvtActionCompleted,
                    on_complete_cb
                )
                # TODO: try "preempt_cb" while an action is being aborted; it
                #   should be fine in SDK 1.3.2, see:
                #   https://github.com/anki/cozmo-python-sdk/blob/1.3.2/src/cozmo/action.py#L638-L644
                cozmo_action.abort()
                # "on_complete_cb" will call "set_preempted"
                cozmo_action.add_event_handler(
                    cozmo.action.EvtActionCompleted,
                    on_complete_cb
                )
                return

        self._servers[name].register_goal_callback(goal_cb)
        self._servers[name].register_preempt_callback(preempt_cb)

        return self._servers[name]


def run(coz_conn):
    '''The run method runs once Cozmo is connected.'''
    coz = coz_conn.wait_for_robot()

    # TODO: remove "log_level=rospy.DEBUG" when the code is more stable
    rospy.init_node("cozmo", log_level=rospy.DEBUG)

    cozmo_handler = CozmoHandler(coz)
    cozmo_handler.createActionServer('/say_text', ros_cozmo.msg.SayTextAction)
    for name, server in cozmo_handler.servers.items():
        server.start()

    rospy.spin()


if __name__ == "__main__":
    cozmo.setup_basic_logging()  # TODO: remove when the code is more stable
    try:
        cozmo.connect(run)
    except cozmo.ConnectionError as e:
        sys.exit("A connection error occurred: %s" % e)
