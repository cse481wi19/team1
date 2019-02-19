#!/usr/bin/env python

import rospy
import robot_api
import math
from vision_msgs.msg import FrameResults, ImageClustering
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Int8
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Point, Pose, Quaternion
from threading import Thread, RLock
from robot_api import Head, Expressions
from tf import TransformListener

# The FaceCommand class handles the logic for Week 5-6 Post #2
class FaceCommand(object):
	
	def __init__(self):
		self.face_pub = rospy.Publisher('vision/most_confident_face_pos', PointStamped, queue_size=10)
		self.num_faces_pub = rospy.Publisher('vision/face_count', Int8, queue_size=10)
		self.lights = robot_api.Lights()
		self.head = robot_api.Head()
		self.lock = RLock()
		self.base = robot_api.Base()
		self.expressions = robot_api.Expressions()
		self.follow = False			# Indicates if Luci should follow someone
		self.face_exists = False	# Indicates if Luci confidently detects a face
		self.face_detection_counter = 0
		rospy.sleep(0.5)			# Adding sleep to make sure there's enough to initialize other objects

	# Publishes the number of faces found in a frame.
	# Params:
	#		msg: A FrameResults message
	# Returns:
	# 		None
	def publishNumFaces(self, msg):
		i = Int8()
		i.data = len(msg.faces)
		self.num_faces_pub.publish(i)

	# Publishes the location of a face, as a PointStamped message.
	# Params:
	#		msg: A FrameResults message
	# Returns:
	# 		None
	def publishMostConfidentFacePosition(self, msg):
		faces = msg.faces
		confident_face = None
		for face in faces:
			if confident_face is None or confident_face.confidence < face.confidence:
				confident_face = face
		if confident_face is not None:
			self.face_pub.publish(self.getFaceLocation(confident_face))

	# Indicates whether the face is centered by checking if the center's x/y coordinates are
	# close to 0.5 (within some epsilon TODO: 0.05 may need to be tweaked)
	# Params:
	#		face: A Face message
	# Returns:
	# 		bool: whether the given face is centered relative to Kuri's center of frame
	def isCentered(self, face):
		return abs(face.center.x - 0.5) < 0.05 and abs(face.center.y - 0.5) < 0.05

	# Get a face's location in 3D space.
	#  Params:
	#  		face: A Face message.
	#  Returns:
	# 		A PointStamped message
	def getFaceLocation(self, face):
		face_size1 = face.bb[3] * face.bb[2] * face.size * 100.0
		result = PointStamped(header=face.header, point=Point((face.size **-0.4) * 0.3, -0.75*(face.center.x - 0.5), -1*(face.center.y - 0.5)))
		result.header.frame_id = robot_api.Head.EYES_FRAME
		return result

	# Acquiring a lock to make sure another thread isn't currently moving the head at the same time
	def _swag(self, msg):
		if (self.lock.acquire(blocking=False)):
			self._updateFace(msg)
			self._servoFace(msg)
			self.lock.release()

	def _updateFace(self, msg):
		num_faces = len(msg.faces)
		rospy.loginfo(num_faces)

		if (num_faces == 0):
			if(self.face_detection_counter == 0 and self.face_exists == True):
				self.expressions.be_sad()
				self.face_exists = False
			if(self.face_detection_counter > -5):
				self.face_detection_counter -= 1
		else:
			if(self.face_detection_counter == 0):
				self.expressions.be_happy()
				self.face_exists = True
			if(self.face_detection_counter < 5):
				self.face_detection_counter += 1



	# TODO Add logic to turn base when searching for face
	def _servoFace(self, msg):
		#self.lights.put_pixels([(255,0,0)]*15)
		faces = msg.faces

		# Find the most confident face
		confident_face = None
		for face in faces:
			if confident_face is None or confident_face.confidence < face.confidence:
				confident_face = face
		if confident_face is None: 
			return
		if self.isCentered(confident_face):
			return

		point = self.getFaceLocation(confident_face)
		if (self.follow == True):
			# Only move Luci's body if we are following someone
			if point.point.x > 1:
				self.base.go_forward(0.2)
			elif point.point.x < 0.6:
				self.base.go_forward(-0.2)
		self.head.look_at(point, False)

	def callback(self, msg):
		# self._updateFace(msg.faces)
		t1 = Thread(target=self._swag, args=[msg.faces])
		t1.start()

	# Custom callback for command line prompt that publishes boolean to 'vision/commands/follow'
	def command_callback(self, msg):
		if msg is 0:
			self.follow = False
		else:
			self.follow = True
		self.expressions.nod_head()
                
def main():
	rospy.init_node('face_emotion_demo')
	vision = robot_api.Vision()

	# Activate Vision API's Face Detector
	vision.activate("face_detector", config={"fps": 3})
	rospy.sleep(0.5)
	vision.wait_until_ready(timeout=10)
	vision.req_mods([["activate", "face_detector", {"fps": 3}, {"skip_ratio": 3}]], [])

	face_command = FaceCommand()

	# Trigger callback
	rospy.Subscriber('vision/results', FrameResults, face_command.callback)
	rospy.Subscriber('vision/commands/follow', Int8, face_command.command_callback)
	rospy.spin()

if __name__ == '__main__':
	main()
