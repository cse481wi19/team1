import rospy
import math
import robot_api
from vision_msgs.msg import FrameResults, ImageClustering
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Int8
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Point, Pose, Quaternion
from threading import Thread, RLock
from tf import TransformListener

class Servo(object):
	
	def __init__(self):
		self.face_pub = rospy.Publisher('vision/most_confident_face_pos', PointStamped, queue_size=10)
		self.num_faces_pub = rospy.Publisher('vision/face_count', Int8, queue_size=10)
		self.lights = robot_api.Lights()
		self.head = robot_api.Head()
		self.lock = RLock()
		self.base = robot_api.Base()
		self.lastFacePos = None
		self.vision = robot_api.Vision()

		# Activate Vision API's Face Detector
		self.vision.activate("face_detector", config={"fps": 3})
		rospy.sleep(0.5)
		self.vision.wait_until_ready(timeout=10)
		self.vision.req_mods([["activate", "face_detector", {"fps": 3}, {"skip_ratio": 3}]], [])

		# Trigger callback
		rospy.Subscriber('vision/results', FrameResults, self.callback) 
		rospy.Subscriber('vision/most_confident_face_pos', PointStamped, self.onFacePos) 
		rospy.Subscriber('come_here_kuri', Int8, self.navigateTo) 

		rospy.sleep(0.5)

		rospy.spin()

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
		# rospy.loginfo(face)
		result.header.frame_id = robot_api.Head.EYES_FRAME
		return result

	def _servoThread(self, msg):
		if (self.lock.acquire(blocking=False)):
			self._servoFace(msg)
			self.lock.release()

	def _updateLights(self, msg):
	    num_faces = len(msg.faces)
	    if num_faces == 0:
		self.lights.put_pixels([(255,0,0)]*15)
	    else:
		confidence = msg.faces[0].confidence
		pixels = [(0,255,0)]*min(num_faces,15)
		if num_faces < 15:
			for x in range(15-num_faces):
				pixels.append((0,0,255))
            self.lights.put_pixels(pixels)

    # def _servoFace(self, msg):
    #     faces = msg.faces

    #     # Find the most confident face
    #     confident_face = None
    #     for face in faces:
    #         if confident_face is None or confident_face.confidence < face.confidence:
    #             confident_face = face
    #         if confident_face is None: 
    #             return
    #         if self.isCentered(confident_face):
    #             return

    #     point = self.getFaceLocation(confident_face)
    #     if point.point.x > 1:
    #         self.base.go_forward(0.2)
    #     elif point.point.x < 0.6:
    #         self.base.go_forward(-0.2)
    #         self.head.look_at(point, False)

    def navigateTo(self, num):
		if (self.lastFacePos is not None and self.lock.acquire(blocking=False)):
			self.expressions.nod_head()
			pointStamped = self.getPointStampedInFrame(pointStamped, "base_link")
			distance_to_base = sqrt(pointStamped.point.x ** 2 + pointStamped.point.y ** 2 + pointStamped.point.z ** 2)
			unit_vector = { "x": pointStamped.point.x / distance_to_base,
							"y": pointStamped.point.y / distance_to_base }

			pointStamped.point.x = unit_vector["x"] * (distance_to_base - 0.5)
			pointStamped.point.y = unit_vector["y"] * (distance_to_base - 0.5)

			quaternion = Quaternion()
			quaternion.w = 1

			pose = Pose()
			pose.position = pointStamped.point
			pose.orientation = quaternion

			poseStamped = PoseStamped()
			poseStamped.header = pointStamped.header
			pointStamped.pose = pose

			self.move_pub.publish(poseStamped)
			self.lock.release()

	def onFacePos(self, posStamped):
		self.lastFacePos = posStamped

	def callback(self, msg):
		rospy.loginfo("Face changed. Callback triggered")
		self._updateLights(msg.faces)
		t1 = Thread(target=self._servoThread, args=[msg.faces])
		t1.start()

		self.publishMostConfidentFacePosition(msg.faces)
		self.publishNumFaces(msg.faces) # this should always go last
		# self.move_pub.publish(1)
		# Move Kuri Base forward/backward
