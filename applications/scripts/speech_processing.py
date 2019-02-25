#!/usr/bin/env python

import os
import rospy
import robot_api

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *

class SpeechProcessor(object):

	def __init__(self):
		self.lights = robot_api.Lights()

	def _updateLights(self, msg):
		# TODO: Have Kuri change colors based on response to speech
		# if emergency/nurse needs to be alerted
		# RED: self.lights.put_pixels([(255,0,0)]*15)
		# if happy: YELLOW
		# if sad: BLUE

	def _showExpression(self, msg):
		# TODO: Have Kuri show an expression as a response to speeech
		# if happy: expressions happy
		# if sad: expressions sad

	def callback(self, msg):
		self._updateLights(msg)
		self._showExpressions(msg)

def main():
	rospy.init_node('speech_processing_demo')
	speech_processor = SpeechProcessor()
	rospy.Subscriber('TODO_some_audio_microphone_input', TODO_AudioInputType, speech_processor.callback)
	rospy.spin()

if __name__ == '__main__':
	main()