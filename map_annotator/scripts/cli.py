#! /usr/bin/env python

import rospy
from map_annotator.srv import ManageMarker, ManageMarkerResponse
from map_annotator.msg import Markers


class MarkerServer(object):
    def __init__(self):
        self.markers = []
        self.sub = rospy.Subscriber("/map_annotator/marker_list", Markers, self.callback)

    def callback(self, data):
        self.markers = data.markers

def getName(cmd):
    name = cmd.split()[1:]
    return " ".join(name)

def main():
    rospy.init_node('map_annotator_cli')

    markerServer = MarkerServer()
    manage_marker = rospy.ServiceProxy('/map_annotator/manage_marker', ManageMarker)

    while not rospy.is_shutdown():
        cmd = raw_input('> ')
        if (cmd.startswith("list")):
            print(markerServer.markers)
        elif (cmd.startswith("create")):
            response = manage_marker(markerName=getName(cmd), cmd="create", newMarkerName="")
            print(response.message)
        elif (cmd.startswith("delete")):
            response = manage_marker(markerName=getName(cmd), cmd="delete", newMarkerName="")
            print(response.message)
        elif (cmd.startswith("goto")):
            response = manage_marker(markerName=getName(cmd), cmd="goto", newMarkerName="")
            print(response.message)
        elif (cmd.startswith("quit")):
            break

if __name__ == '__main__':
    main()
