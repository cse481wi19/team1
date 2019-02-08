#!/usr/bin/env python

# from robot_api import Head
# from robot_api import Lights
# from robot_api import Vision 

import rospy
import robot_api

# def on_face_change():
# Tilt head up or down
# TODO: change the light color to visualize some aspect of the detections
# lights = Lights()
# lights.all_leds(cls, (r,g,b)) 
# lights.off()

def main():
	rospy.init_node('face_detection_demo')
	vision = robot_api.Vision()

	# TODO: See if a skip ratio is needed for req_mods. (Suggested:3) 
	vision.req_mods([["activate", "face_detector", {"fps": 6}, {}]], [])

	# rospy.Subscriber('vision/captured', Vision, on_face_change)
	# rospy.spin()

if __name__ == '__main__':
	main()
