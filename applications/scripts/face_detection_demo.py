#!/usr/bin/env python

import rospy
import robot_api

# Message type of captured vision topic: vision_msgs/ImageClustering
from vision_msgs.msg import ImageClustering

class FaceChange(object):
	def __init__(self):
		self._val = 0

	def _checkFaceBelowOrAbove(self, msg):
		print('TODO: Implement logic to check whether the face is above or below middle of the frame')

		lights = robot_api.Lights()
		head = robot_api.Head()

		# lights.all_leds(cls, (r,g,b)) 
		# lights.off()
		
		# if face_below_middle_of_frame:
		#       tilt_up()
		# else if face_above_middle_of_frame:
		#       tilt_down()

	def callback(self, msg):
		rospy.loginfo(msg)
		print('Callback reached')
		# self._checkFaceBelowOrAbove(msg)
                
def main():
	rospy.init_node('face_detection_demo')
	vision = robot_api.Vision()
	face_change = FaceChange()

	# TODO: See if a skip ratio is needed for req_mods. (Suggested:3) 
	vision.req_mods([["activate", "face_detector", {"fps": 6}, {}]], [])

	# Trigger callback
	rospy.Subscriber('vision/captured', ImageClustering, face_change.callback) 
	
	rospy.spin()

if __name__ == '__main__':
	main()
