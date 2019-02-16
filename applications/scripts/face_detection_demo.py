#!/usr/bin/env python

import rospy
import robot_api
import math

from vision_msgs.msg import FrameResults, ImageClustering
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Int8
from geometry_msgs.msg import PointStamped, Point

# The FaceChange class handles all Face Detection related Kuri changes.
class FaceChange(object):
	# height and width in pixels, dist in feet
	# REF_HEAD_WIDTH = 72.26519775390625		#7 inches
	# REF_HEAD_HEIGHT = 424.66326904296875   #10 inches
	# REF_HEAD_SIZE_PERCENT = 0.00112760916818
	# REF_HEAD_SIZE = REF_HEAD_HEIGHT * REF_HEAD_WIDTH * REF_HEAD_SIZE_PERCENT * 100.0
	# REF_HEAD_DIST_FT = 2.0
	# PIXELS_PER_INCH = (REF_HEAD_SIZE/70) ** 0.5
	# REF_HEAD_DIST = PIXELS_PER_INCH * REF_HEAD_DIST_FT * 12.0
	
	def __init__(self):
		self.face_pub = rospy.Publisher('vision/most_confident_face_pos', PointStamped, queue_size=10)
		self.num_faces_pub = rospy.Publisher('vision/face_count', Int8, queue_size=10)
		rospy.sleep(0.5)

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

	# Indicates whether the face is centered by checking if the center's x/y coordinates are
	# close to 0.5 (within some epsilon TODO: 0.05 may need to be tweaked)
	# Params:
	#		face: A Face message
	# Returns:
	# 		bool: whether the given face is centered relative to Kuri's center of frame
	def isCentered(face):
		return math.abs(face.center.x - 0.5) < 0.05 and math.abs(face.center.y - 0.5) < 0.05

	# Get a face's location in 3D space.
	#  Params:
	#  		face: A Face message.
	#  Returns:
	# 		A PointStamped message
	def getFaceLocation(self, face):
		result = PointStamped()
		face_size = face.bb[3] * face.bb[2] * face.size * 100.0
		ratio = face_size / FaceChange.REF_HEAD_SIZE
		point = Point(FaceChange.REF_HEAD_DIST / ratio, face.center.x, face.center.y)

		result.point = point
		result.header = face.header
		result.header.frame_id = robot_api.Head.EYES_FRAME
		return result	

	def _updateLights(self, msg):
		lights = robot_api.Lights()
		
		num_faces = len(msg.faces.faces)
		if num_faces == 0:
			lights.put_pixels([(255,0,0)]*15)
		else:
			confidence = msg.faces.faces[0].confidence
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

		if confident_face == None or self.isCentered(confident_face): return
		point = self.getFaceLocation(confident_face)
		rospy.loginfo(point)
		head.look_at(point, False)		

	def callback(self, msg):
		self._updateLights(msg)
		self._servoFace(msg)
		self.publishMostConfidentFacePosition(msg)
		self.publishNumFaces(msg) # this should always go last
                
def main():
	rospy.init_node('face_detection_demo')
	vision = robot_api.Vision()
	face_change = FaceChange()

	# Activate Vision API's Face Detector
	vision.req_mods([["activate", "face_detector", {"fps": 6}, {"skip_ratio": 3}]], [])

	# Trigger callback
	rospy.Subscriber('vision/results', FrameResults, face_change.callback) 
	rospy.spin()

if __name__ == '__main__':
	main()
