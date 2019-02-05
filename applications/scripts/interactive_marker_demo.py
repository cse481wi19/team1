#!/usr/bin/env python

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import Odometry

import rospy
import robot_api

def handle_viz_input_forward(input):
	if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
		print('Moving forward 0.5 m')
		base = robot_api.Base()
		base.go_forward(0.5, 0.6)

def handle_viz_input_clockwise(input):
	if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
		print('Rotating clockwise 90 degrees')
		base = robot_api.Base()
		base.turn(-1.57, 0.7)

def handle_viz_input_counterclockwise(input):
	if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
		print('Rotating counter-clockwise 90 degrees')
		base = robot_api.Base()
		base.turn(1.57, 0.7)

def main():
	rospy.init_node('interactive_marker_demo')
	server = InteractiveMarkerServer("simple_marker")

	# Marker to move forward
	int_marker_forward = InteractiveMarker()
	int_marker_forward.header.frame_id = "base_link"
	int_marker_forward.name = "forward_marker"
	int_marker_forward.description = "Move Forward"
	int_marker_forward.pose.position.x = 1
	int_marker_forward.pose.orientation.w = 1

	box_marker_forward = Marker()
	box_marker_forward.type = Marker.CUBE
	box_marker_forward.pose.orientation.w = 1
	box_marker_forward.scale.x = 0.3
	box_marker_forward.scale.y = 0.3
	box_marker_forward.scale.z = 0.3
	box_marker_forward.color.r = 0.0
	box_marker_forward.color.g = 0.5
	box_marker_forward.color.b = 0.5
	box_marker_forward.color.a = 1.0

	button_control_forward = InteractiveMarkerControl()
	button_control_forward.interaction_mode = InteractiveMarkerControl.BUTTON
	button_control_forward.always_visible = True
	button_control_forward.markers.append(box_marker_forward)
	int_marker_forward.controls.append(button_control_forward)

	# Marker to rotate clockwise
	int_marker_clock = InteractiveMarker()
	int_marker_clock.header.frame_id = "base_link"
	int_marker_clock.name = "clock_marker"
	int_marker_clock.description = "Turn Clockwise"
	int_marker_clock.pose.position.x = 0
	int_marker_clock.pose.position.y = -1
	int_marker_clock.pose.orientation.w = 1

	box_marker_clock = Marker()
	box_marker_clock.type = Marker.CUBE
	box_marker_clock.pose.orientation.w = 1
	box_marker_clock.scale.x = 0.3
	box_marker_clock.scale.y = 0.3
	box_marker_clock.scale.z = 0.3
	box_marker_clock.color.r = 0.0
	box_marker_clock.color.g = 0.5
	box_marker_clock.color.b = 0.5
	box_marker_clock.color.a = 1.0

	button_control_clock = InteractiveMarkerControl()
	button_control_clock.interaction_mode = InteractiveMarkerControl.BUTTON
	button_control_clock.always_visible = True
	button_control_clock.markers.append(box_marker_clock)
	int_marker_clock.controls.append(button_control_clock)

	# Marker to rotate counter-clockwise
	int_marker_counter = InteractiveMarker()
	int_marker_counter.header.frame_id = "base_link"
	int_marker_counter.name = "counter_marker"
	int_marker_counter.description = "Turn Counter-clockwise"
	int_marker_counter.pose.position.x = 0
	int_marker_counter.pose.position.y = 1
	int_marker_counter.pose.orientation.w = 1

	box_marker_counter = Marker()
	box_marker_counter.type = Marker.CUBE
	box_marker_counter.pose.orientation.w = 1
	box_marker_counter.scale.x = 0.3
	box_marker_counter.scale.y = 0.3
	box_marker_counter.scale.z = 0.3
	box_marker_counter.color.r = 0.0
	box_marker_counter.color.g = 0.5
	box_marker_counter.color.b = 0.5
	box_marker_counter.color.a = 1.0

	button_control_counter = InteractiveMarkerControl()
	button_control_counter.interaction_mode = InteractiveMarkerControl.BUTTON
	button_control_counter.always_visible = True
	button_control_counter.markers.append(box_marker_counter)
	int_marker_counter.controls.append(button_control_counter)

	# Add markers to the server
	server.insert(int_marker_forward, handle_viz_input_forward)
	server.insert(int_marker_counter, handle_viz_input_counterclockwise)
	server.insert(int_marker_clock, handle_viz_input_clockwise)
	server.applyChanges()
	rospy.spin()

if __name__ == '__main__':
  main()

