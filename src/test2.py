#! /usr/bin/env python3

import sys
import cozmo
import rospy
# import actionlib
# import actionlib_tutorials.msg


class CozmoTopics(object):
    def __init__(self, coz):

        self._robot = coz

        self._robot_state_handler = self._robot.add_event_handler(
            self._robot.EvtRobotStateUpdated,
            self.on_robot_state_update
        )


def run(coz_conn):
    '''The run method runs once Cozmo is connected.'''
    coz = coz_conn.wait_for_robot()

    rospy.init_node('cozmo')
    CozmoTopics(coz)
    rospy.spin()


if __name__ == '__main__':
    try:
        cozmo.connect(run)
    except cozmo.ConnectionError as e:
        sys.exit("A connection error occurred: %s" % e)
