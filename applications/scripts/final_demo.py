#!/usr/bin/env python

# Demo file for week 8 

import rospy
import robot_api

from vision_msgs.msg import FrameResults
from threading import Thread

def main():
    rospy.init_node('final_demo')

    # Initialize objects
    speech = robot_api.Speech()
    vision = robot_api.Vision()
    servo = robot_api.Servo()

    # Start objects
    vision.start()

    # TODO Figure out if needed, Trigger servo's callback
    # rospy.Subscriber('vision/results', FrameResults, servo.callback) 

    rospy.spin()

if __name__ == '__main__':
	main()
