#!/usr/bin/env python

import robot_api
import rospy
from web_teleop.srv import SetEyelids, SetEyelidsResponse


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class EyelidsServer(object):
    def __init__(self):
        self._head = robot_api.Head()

    def handle_set_eyelids(self, request):
        # TODO: move the torso to the requested height
        self._head.eyes_to(request.radians)
        response = SetEyelidsResponse()
        response.message = "Eyelids set to " + str(request.radians) + " rads"
        return response


def main():
    rospy.init_node('web_teleop_eyelids')
    wait_for_time()
    server = EyelidsServer()
    eyelid_service = rospy.Service('web_teleop/set_eyelids', SetEyelids,
                                  server.handle_set_eyelids)
    rospy.spin()


if __name__ == '__main__':
    main()
