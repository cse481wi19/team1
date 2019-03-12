#!/usr/bin/env python

# Demo file for week 8 

import rospy
import robot_api
import numpy
from vision_msgs.msg import FrameResults
from threading import Thread

speech = None
servo = None

def main():
    rospy.init_node('final_demo')

    # Initialize speech and servo nodes
    speech_thread = Thread(target=_init_speech)
    speech_thread.start()

    servo_thread = Thread(target=_init_servo)
    servo_thread.start()
 
    # TODO: Implement demos below
    # TODO: add necessary words to corpus

    # Bingo Demo
        # Convo
        # Luci turns around after saying grab my bingo card
        # Luci - be_happy

    # Med Reminder Demo
        # Convo
        # Nurse adds reminder through web

    # Alert Demo
        # Convo
        # When convo is triggered: Use servoing code to get closer/back up 

    rospy.spin()

def _init_speech():
    speech = robot_api.Speech()

def _init_servo():
    servo = robot_api.Servo()

if __name__ == '__main__':
	main()
