#!/usr/bin/env python

import rospy
import robot_api
import math
import time
import alasaaudio
import pyttsx

from vision_msgs.msg import FrameResults, ImageClustering
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Int8, String
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Point, Pose, Quaternion
from threading import Thread, RLock
from robot_api import Head, Expressions
from tf import TransformListener
from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *

class FaceInteractionDemo(object):
    def __init__(self):
		self.move_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
		self.lock = RLock()
		self.head = Head()
		self.expressions = Expressions()
		self.tf = TransformListener()
		self.lastFacePos = None
		rospy.sleep(0.5)

    # Gets an equivalent point in the reference frame "destFrame"
    def getPointStampedInFrame(self, pointStamped, destFrame):
		# Wait until we can transform the pointStamped to destFrame
		self.tf.waitForTransform(pointStamped.header.frame_id, destFrame, rospy.Time(), rospy.Duration(4.0))
		# Set point header to the last common frame time between its frame and destFrame
		pointStamped.header.stamp = self.tf.getLatestCommonTime(pointStamped.header.frame_id, destFrame)
		# Transform the point to destFrame
		return self.tf.transformPoint(destFrame, pointStamped)

    def onFacePos(self, posStamped):
		self.lastFacePos = posStamped

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

    def onFaceCount(self, int8):
        if (int8.data > 0):
            self.expressions.be_happy()
        else:
            self.expressions.be_sad()
            self.expressions.be_neutral()

# The FaceChange class handles all Face Detection related Kuri changes.
class FaceChange(object):
	
	def __init__(self):
		self.face_pub = rospy.Publisher('vision/most_confident_face_pos', PointStamped, queue_size=10)
		self.num_faces_pub = rospy.Publisher('vision/face_count', Int8, queue_size=10)
		self.lights = robot_api.Lights()
		self.head = robot_api.Head()
		self.lock = RLock()
		self.base = robot_api.Base()
		self.facesDemo = FaceInteractionDemo()
		rospy.sleep(0.5)

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

	def _swag(self, msg):
		if (self.lock.acquire(blocking=False)):
			self._servoFace(msg)
			self.lock.release()

	def _updateLights(self, msg):
		num_faces = len(msg.faces)
		# rospy.loginfo(num_faces)
		if num_faces == 0:
			self.lights.put_pixels([(255,0,0)]*15)
		else:
			confidence = msg.faces[0].confidence
			pixels = [(0,255,0)]*min(num_faces,15)
			if num_faces < 15:
				for x in range(15-num_faces):
					pixels.append((0,0,255))
			self.lights.put_pixels(pixels)

	def _servoFace(self, msg):
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
		if point.point.x > 1:
			self.base.go_forward(0.2)
		elif point.point.x < 0.6:
			self.base.go_forward(-0.2)
		self.head.look_at(point, False)
		
		#rospy.loginfo(point.point.x)
		#rospy.loginfo("after go_forward")

	def callback(self, msg):
		self._updateLights(msg.faces)
		t1 = Thread(target=self._swag, args=[msg.faces])
		t1.start()
		# self.publishMostConfidentFacePosition(msg.faces)
		# self.publishNumFaces(msg.faces) # this should always go last
		# self.move_pub.publish(1)
		# Move Kuri Base forward/backward
		# self.facesDemo.navigateTo(1)

def start_speech(self):
    LuciControl()

def main():
    rospy.init_node('interaction_demo')

    speechThread = Thread(target=self.start_speech)
    speechThread.start()

	vision = robot_api.Vision()

	# Activate Vision API's Face Detector
	vision.activate("face_detector", config={"fps": 3})
	rospy.sleep(0.5)
	vision.wait_until_ready(timeout=10)
	vision.req_mods([["activate", "face_detector", {"fps": 3}, {"skip_ratio": 3}]], [])

	face_change = FaceChange()

	# Trigger callback
	rospy.Subscriber('vision/results', FrameResults, face_change.callback) 

	# setup follow faces
	facesDemo = FaceInteractionDemo()

	# Trigger callback
	rospy.Subscriber('vision/face_count', Int8, facesDemo.onFaceCount)
	rospy.Subscriber('vision/most_confident_face_pos', PointStamped, facesDemo.onFacePos) 
	rospy.Subscriber('come_here_kuri', Int8, facesDemo.navigateTo) 
	rospy.spin()

class LuciControl(object):
    """Class to control Luci's speech interactions """

    def __init__(self):

        # Initialize node
        rospy.init_node("luci_control")
        rospy.on_shutdown(self.shutdown)

        # Create a publisher for the grammar data and raw audio
        self.pub_grammar = rospy.Publisher("grammar_data", String, queue_size=10)
        self.pub_audio = rospy.Publisher("raw_audio", String, queue_size=10)
   
        # Subscribe to grammar data output
        rospy.Subscriber("grammar_data", String, self.parse_results)

        # Set language model and dictionary
        self.class_lm = 'corpus/luci.lm'
        self.dict = 'corpus/luci.dic'

        # Used in process_audio (from asr_test.py)
        self.in_speech_bf = False

        # Set this file from "/usr/share/pocketsphinx/model/hmm/en_US/hub4wsj_sc_8k"
        self.hmm = 'corpus/hmm'

        # Intializing robot API
        self.lights = robot_api.Lights()
        self.expressions = robot_api.Expressions()
        self.sound_source = robot_api.SoundSource('Luci')

        # Initialize alsa audio PCM: (Using 16000Hz, 16 Bit Encoding, 1 Channel)
        self.inp = alsaaudio.PCM(alsaaudio.PCM_CAPTURE, alsaaudio.PCM_NONBLOCK)
        self.inp.setchannels(1)
        self.inp.setrate(16000)
        self.inp.setformat(alsaaudio.PCM_FORMAT_S16_LE)
        self.inp.setperiodsize(160)

        # All params satisfied. Starting recognizer
        t1 = Thread(target=self.start_recognizer)
        t1.start()

        # # Spin up new thread to listen for raw audio input
        t2 = Thread(target=self._listen)
        t2.start()

        rospy.spin()

    def _listen(self):
        """Function to loop indefinitely while listening for audio"""
        rospy.loginfo("Listening.......")
        while True:
            l, data = self.inp.read()
            if l:
                self.pub_audio.publish(data)
                time.sleep(.001)

    def _play(self, file):
        """Function to neutrally respond to given words"""
        self.expressions.be_neutral()
        sleep_time = 4
        if (file[0] == '1'):
            # Conversation 1 (about sunny weather)
            self.lights.put_pixels([(255, 255, 7)]*15)  # YELLOW
        elif (file[0] == '2'):
            # Conversation 2 (about going to bingo night)
            self.lights.put_pixels([(222, 7, 255)]*15)  # PURPLE
        elif (file[0] == '3'):
            # Conversation 3 (about alerting the nurse)
            self.lights.put_pixels([(255, 7, 7)]*15)    # RED
            if file[0] == '3_1':
                sleep_time = 5
            else:
                sleep_time = 4
        elif (file[0] == '4'):
            # Conversation 4 (about going home)
            self.lights.put_pixels([(0, 75, 173)]*15)   # BLUE
            if file[0] == '4_3':
                sleep_time = 30
            else:
                sleep_time = 4
        elif (file[0] == '5'):
            # Conversation 5 (about winning at bingo)
            self.lights.put_pixels([(12, 255, 0)]*15)   # GREEN

        sound = self.sound_source.play('/home/team1/catkin_ws/src/sound_effects/interactions/' + file)
        rospy.sleep(sleep_time)
        self.sound_source.cancel(sound)
        rospy.sleep(1)

    def _isDetected(self, detected_words, words):
        """Function to check if any of the given words are detected"""
        for word in words:
            if word in detected_words.data:
                return True
        return False

    def parse_results(self, detected_words): #pylint: disable=too-many-branches
        """Function to perform action (change colors/expression) on detected word"""
        if self._isDetected(detected_words, ['WEATHER']):
            self._play('1_1.wav')
        elif self._isDetected(detected_words, ['OUTSIDE']):
            self._play('1_2.wav')
        elif self._isDetected(detected_words, ['DAY']):
            self._play('2_1.wav')
        elif self._isDetected(detected_words, ['GREAT']):
            self._play('2_2.wav')
        elif self._isDetected(detected_words, ['HELP']):
            self._play('3_1.wav')
        elif self._isDetected(detected_words, ['CANNOT']):
            self._play('3_2.wav')
        elif self._isDetected(detected_words, ['HOME']):
            self._play('4_1.wav')
        elif self._isDetected(detected_words, ['NO']):
            self._play('4_2.wav')
        elif self._isDetected(detected_words, ['YES']):
            self._play('4_3.wav')
        elif self._isDetected(detected_words, ['GUESS']):
            self._play('5_1.wav')
        elif self._isDetected(detected_words, ['WON']):
            self._play('5_2.wav')
        # else:
        #     self._play('0_1.wav')

    def start_recognizer(self):
        """Function to handle lm or grammar processing of audio."""

        config = Decoder.default_config()
        rospy.loginfo("Done initializing pocketsphinx")

        # Setting configuration of decoder using provided params
        config.set_string('-hmm', self.hmm)
        config.set_string('-dict', self.dict)

        # Using language model. See asr_test.py for how to use grammar
        rospy.loginfo('Language Model Found.')
        config.set_string('-lm', self.class_lm)
        self.decoder = Decoder(config)

        # Start processing input audio
        self.decoder.start_utt()
        rospy.loginfo("Decoder started successfully")

        # Subscribe to audio topic
        rospy.Subscriber("raw_audio", String, self.process_audio)
        rospy.spin()

    def process_audio(self, data):
        """Audio processing based on decoder config."""
        
        # Check if input audio has ended
        self.decoder.process_raw(data.data, False, False)
        if self.decoder.get_in_speech() != self.in_speech_bf:
            self.in_speech_bf = self.decoder.get_in_speech()
            if not self.in_speech_bf:
                self.decoder.end_utt()
                if self.decoder.hyp() != None:
                    rospy.loginfo('OUTPUT: \"' + self.decoder.hyp().hypstr + '\"')
                    # NOTE Consider removing grammar_data topic and calling parse_results directly 
                    self.pub_grammar.publish(self.decoder.hyp().hypstr)
                self.decoder.start_utt()

    def shutdown(self):
        """
        command executed after Ctrl+C is pressed
        """
        rospy.loginfo("Stop LuciControl")
        rospy.sleep(1)

if __name__ == '__main__':
	main()
