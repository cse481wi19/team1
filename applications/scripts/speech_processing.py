#!/usr/bin/env python

import rospy
import robot_api

class SpeechProcessor(object):
	def __init__(self):
		self.val = 0

def main():
	rospy.init_node('speech_processing_demo')
	rospy.spin()

if __name__ == '__main__':
	main()
