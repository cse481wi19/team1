#!/usr/bin/env python

import robot_api
import rospy
from web_teleop.srv import SetHead, SetHeadResponse


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class HeadServer(object):
    def __init__(self):
        self._head = robot_api.Head()

    def handle_set_head(self, request):
        # TODO: move the torso to the requested height
        print(request)
        self._head.pan_and_tilt(request.pan, request.tilt)
        response = SetHeadResponse()
        response.message = "Head set to pan: " + str(request.pan) + ", tilt: " + str(request.tilt)
        return response


def main():
    rospy.init_node('web_teleop_head')
    wait_for_time()
    server = HeadServer()
    head_service = rospy.Service('web_teleop/set_head', SetHead,
                                  server.handle_set_head)
    rospy.spin()


if __name__ == '__main__':
    main()
