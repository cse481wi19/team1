#!/usr/bin/env python

import rospy
import robot_api

from vision_msgs.msg import FrameResults, ImageClustering
from nav_msgs.msg import Odometry

# The FaceChange class handles all Face Detection related Kuri changes.
class FaceChange(object):
	def __init__(self):
		self._val = 0

	def face_below_middle_of_frame(self, msg):
		print('TODO: Check msg object center pt')
		return True

	def face_above_middle_of_frame(self, msg):
		print('TODO: Check msg object center pt')
		return True

	def _updateLights(self, msg):
		print('TODO: Update Kuri Lights based on detections.')
		lights = robot_api.Lights()
		
		num_faces = len(msg.faces.faces)
		size = msg.faces.faces.size
		pos = msg.faces.faces.center
		# TODO: Other confidence detection exists @ msg.all_detections.objects.confidence
		confidence = msg.faces.faces.confidence
		
		# TODO: Change the light color to visualize some aspect of the detections
		if num_faces == 0:
			lights.put_pixels(Lights.RED)
		else:
			lights.put_pixels(Lights.GREEN)

	def _servoFace(self, msg):
		print('TODO: Visual Servoing: Making Kuri look at a face')
		head = robot_api.Head()

		# TODO: Figure out what the pan and tilt values should be
		pan = 0
		tilt = 0
		
		if face_below_middle_of_frame(msg):
			head.pan_and_tilt(pan, tilt)
		elif face_above_middle_of_frame(msg):
			head.pan_and_tilt(pan, tilt)

	def callback(self, msg):
		rospy.loginfo(msg)
		print('Callback reached')
		self._updateLights(msg)
		self._servoFace(msg)
                
def main():
	rospy.init_node('face_detection_demo')
	vision = robot_api.Vision()
	face_change = FaceChange()

	# Activate Vision API's Face Detector
	# TODO: See if a skip ratio is needed for req_mods. (Suggested:3) 
	vision.req_mods([["activate", "face_detector", {"fps": 6}, {}]], [])

	# Trigger callback
	rospy.Subscriber('vision/results', FrameResults, face_change.callback) 
	rospy.spin()

if __name__ == '__main__':
	main()
