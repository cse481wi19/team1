#!/usr/bin/env python

# Demo file for week 8 

import rospy
import robot_api

from vision_msgs.msg import FrameResults
from threading import Thread

def main():
    rospy.init_node('final_demo')

    # Initialize objects

    self.speech = None
    self.servo = None

    speech_thread = Thread(target=self._init_speech)
    speech_thread.start()

    servo_thread = Thread(target=self._init_servo)
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
    self.speech = robot_api.Speech()

def _init_servo():
    self.servo = robot_api.Servo()

if __name__ == '__main__':
	main()
