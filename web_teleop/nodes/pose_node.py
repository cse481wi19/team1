#!/usr/bin/env python

import robot_api
import rospy
from web_teleop.srv import ListPoses, ListPosesResponse
from web_teleop.srv import ManagePose, ManagePoseResponse


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class PosesServer(object):
    def __init__(self):
        pass

    def handle_manage_pose(self, request):
        response = ManagePoseResponse()
        response.message = "Invalid Command: " + request.cmd
        if request.cmd.lower() == 'create':
            response.message = "Created " + request.poseName
        elif request.cmd.lower() == 'delete':
            response.message = "Deleted " + request.poseName
        elif request.cmd.lower() == 'go':
            response.message = "Robot sent to " + request.poseName
        return response

    def handle_list_poses(self, request):
        response = ListPosesResponse()
        return response

def main():
    rospy.init_node('web_teleop_eyelids')
    wait_for_time()
    posesServer = PosesServer()
    list_poses_service = rospy.Service('web_teleop/list_poses', ListPoses,
                                  posesServer.handle_list_poses)
    manage_poses_service = rospy.Service('web_teleop/manage_pose', ManagePose,
                                  posesServer.handle_manage_pose)
    rospy.spin()

if __name__ == '__main__':
    main()
