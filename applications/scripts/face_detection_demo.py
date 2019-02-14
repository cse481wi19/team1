#!/usr/bin/env python

import rospy
import robot_api

from vision_msgs.msg import FrameResults, ImageClustering
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Int8
from geometry_msgs.msg import PointStamped, Point

# The FaceChange class handles all Face Detection related Kuri changes.
class FaceChange(object):
	def __init__(self):
		self._val = 0
		self.face_pub = rospy.Publisher('vision/most_confident_face_pos', PointStamped, queue_size=10)
		self.num_faces_pub = rospy.Publisher('vision/face_count', PointStamped, queue_size=10)

	# Point point = the point in 3d space
	# Frame f = the frame of the point p
	def publishPoint(self, point, frame):
		h = Header()
		h.frame_id = frame
		h.stamp = rospy.Time().now()

		ps = PointStamped()
		ps.header = h
		ps.point = point

		self.face_pub.publish(ps)

	def face_pan_amnt(self, confident_face):
		print('TODO: Check msg object center pt')
		return confident_face.x

	def face_tilt_amnt(self, confident_face):
		print('TODO: Check msg object center pt')
		return confident_face.y

	def _updateLights(self, msg):
		print('TODO: Update Kuri Lights based on detections.')
		lights = robot_api.Lights()
		
		num_faces = len(msg.faces.faces)
		# size = msg.faces.faces[0].size
		# pos = msg.faces.faces[0].center
		# confidence = msg.faces.faces[0].confidence
		
		if num_faces == 0:
			lights.put_pixels([(255,0,0)]*15)
		else:
			lights.put_pixels([(0,num_faces*25+5,0)]*15)

	def _servoFace(self, msg):
		print('TODO: Visual Servoing: Making Kuri look at a face')
		head = robot_api.Head()

		faces = msg.faces.faces 
		confident_face = None
		for face in faces:
			if confident_face == None or confident_face.confidence < face.confidence:
				confident_face = face

		# TODO: test this
		if confident_face == None: return
		pan = face_pan_amnt(confident_face)
		tilt = face_tilt_amnt(confident_face)
		
		head.pan_and_tilt(pan, tilt)
		

	def callback(self, msg):
		rospy.loginfo(msg)
		print('Callback reached')
		self._updateLights(msg)
		self._servoFace(msg)

		# self.publishPoint(point) do this here, this will move to and look at the person

		# this should always go last
		i = Int8()
		i.data = len(msg.faces.faces)
		self.num_faces_pub.publish(i)
                
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
