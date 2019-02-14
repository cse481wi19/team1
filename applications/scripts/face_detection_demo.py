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
		self.face_pub = rospy.Publisher('vision/most_confident_face_pos', PointStamped, queue_size=10)
		self.num_faces_pub = rospy.Publisher('vision/face_count', Int8, queue_size=10)

	# Publishes the number of faces found in a frame.
	# Params:
	#		msg: A FrameResults message
	# Returns:
	# 		None
	def publishNumFaces(self, msg):
		i = Int8()
		i.data = len(msg.faces.faces)
		self.num_faces_pub.publish(i)

	# Publishes the location of a face, as a PointStamped message.
	# Params:
	#		msg: A FrameResults message
	# Returns:
	# 		None
	def publishMostConfidentFacePosition(self, msg):
		faces = msg.faces.faces 
		confident_face = None
		for face in faces:
			if confident_face == None or confident_face.confidence < face.confidence:
				confident_face = face
		if confident_face is not None:
			self.face_pub.publish(self.getFaceLocation(confident_face))

	# Get a face's location in 3D space.
	#  Params:
	#  		face: A Face message.
	#  Returns:
	# 		A PointStamped message
	def getFaceLocation(self, face):
		result = PointStamped()
		result.point = face.center
		result.header = face.header
		return result

	def _updateLights(self, msg):
		lights = robot_api.Lights()
		
		num_faces = len(msg.faces.faces)
		# size = msg.faces.faces[0].size
		# pos = msg.faces.faces[0].center
		if num_faces == 0:
			lights.put_pixels([(255,0,0)]*15)
		else:
			confidence = msg.faces.faces[0].confidence
			# pixels = [(0,confidence*255,0)]*num_faces
			pixels = [(0,255,0)]*num_faces
			if num_faces < 15:
				for x in range(15-num_faces):
					pixels.append((0,0,255))
			lights.put_pixels(pixels)
					

	def _servoFace(self, msg):
		head = robot_api.Head()

		faces = msg.faces.faces 
		confident_face = None
		for face in faces:
			if confident_face == None or confident_face.confidence < face.confidence:
				confident_face = face

		# TODO: test this
		if confident_face == None: return
		head.look_at(self.getFaceLocation(confident_face), False)
		

	def callback(self, msg):
		rospy.loginfo(msg)
		self._updateLights(msg)
		self._servoFace(msg)
			
		self.publishMostConfidentFacePosition(msg)
		self.publishNumFaces(msg) # this should always go last
                
def main():
	rospy.init_node('face_detection_demo')
	vision = robot_api.Vision()
	face_change = FaceChange()

	# Activate Vision API's Face Detector
	# TODO: See if a skip ratio is needed for req_mods. (Suggested:3) 
	vision.req_mods([["activate", "face_detector", {"fps": 6}, {"skip_ratio": 3}]], [])

	# Trigger callback
	rospy.Subscriber('vision/results', FrameResults, face_change.callback) 
	rospy.spin()

if __name__ == '__main__':
	main()
