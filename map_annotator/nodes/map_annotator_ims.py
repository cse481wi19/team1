#!/usr/bin/env python
import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster
from math import sin

server = None
br = None
counter = 0

def frameCallback( msg ):
    global counter, br
    time = rospy.Time.now()
    br.sendTransform( (0, 0, sin(counter/140.0)*2.0), (0, 0, 0, 1.0), time, "base_link", "moving_frame" )
    counter += 1

def processFeedback(feedback):
    s = "Feedback from marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"

    mp = ""
    if feedback.mouse_point_valid:
        mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id

    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo( s + ": button click" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        rospy.loginfo( s + ": pose changed")
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        rospy.loginfo( s + ": mouse down" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        rospy.loginfo( s + ": mouse up" + mp + "." )
    server.applyChanges()

def saveMarker( int_marker ):
    server.insert(int_marker, processFeedback)

def makeMarker(name):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = name
    int_marker.description = name
    int_marker.pose.position.x = 1
    int_marker.pose.orientation.w = 1

    name_marker = Marker()
    name_marker.type = Marker.TEXT_VIEW_FACING
    name_marker.text = name
    name_marker.pose.orientation.w = 1
    name_marker.pose.position.z = .2
    name_marker.scale.x = 0.3
    name_marker.scale.y = 0.3
    name_marker.scale.z = 0.3
    name_marker.color.r = 0.0
    name_marker.color.g = 0.0
    name_marker.color.b = 0.0
    name_marker.color.a = 1.0

    
    marker = Marker()
    marker.type = Marker.CYLINDER
    marker.pose.orientation.w = 1
    marker.scale.x = 0.25
    marker.scale.y = 0.25
    marker.scale.z = 0.05
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    control.always_visible = True
    control.markers.append(marker)
    control.markers.append(name_marker)
    int_marker.controls.append(control)

    return int_marker

if __name__=="__main__":
    rospy.init_node("map_annotator_ims")
    # br = TransformBroadcaster()
    # rospy.Timer(rospy.Duration(0.01), frameCallback)

    server = InteractiveMarkerServer("map_annotator_ims")
    saveMarker(makeMarker("Test Marker"))
    server.applyChanges()

    rospy.spin()

