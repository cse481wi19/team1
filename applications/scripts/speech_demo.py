#!/usr/bin/python
import rospy
import robot_api

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry

class LuciControl(object):
    """Class to handle turtlebot simulation control using voice"""

    def __init__(self):

        # initialize node
        rospy.init_node("luci_control")
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("Initializing")

        # Default values for turtlebot_simulator
        self.speed = 0.2

        # Intializing message type
        self.msg = Twist()
        self.base = robot_api.Base()
        self.lights = robot_api.Lights()
        self.sound_source = robot_api.SoundSource()

        # Initializing publisher with buffer size of 10 messages
        self.pub_ = rospy.Publisher("mobile_base/commands/velocity", Twist, queue_size=10)

        # Subscribe to kws output
        rospy.Subscriber("grammar_data", String, self.parse_results)
        rospy.spin()

    def _greeting(self):
        self.lights.put_pixels([(255, 255, 0)]*15) # Turn YELLOW

    def _alert_nurse(self):
        self.lights.put_pixels([(255,0,0)]*15) # Turn RED
        self.base.go_forward(0.5)

    def _find(self, phrase):
        return detected_words.data.find(phrase) > -1

    def parse_results(self, detected_words): #pylint: disable=too-many-branches
        """Function to perform action on detected word"""
        self.lights.put_pixels([(255,0,0)]*15)
        self.base.go_forward(0.5)

        if self._find("HI") or self._find("HELLO") or self._find("HEY"): 
            self._greeting()

        if self._find("NURSE") or self._find("HELP"):
            self._alert_nurse()

        if detected_words.data.find("FRANK SINATRA") > -1:
            rospy.loginfo("Found Frank. Should be FULL SPEED")
            self._alert_nurse()

        elif detected_words.data.find("BINGO") > -1:
            rospy.loginfo("Found Bingo. Should be HALF SPEED")
            self.lights.put_pixels([(102,255,102)]*15) # Turn GREEN

        elif detected_words.data.find("YUMMY") > -1:
            rospy.loginfo("Found Yummy. Should be moving forward")
            self.lights.put_pixels([(0, 0, 255)]*15) # Turn BLUE   
            self.base.go_forward(10)

        elif detected_words.data.find("LUCI") > -1:
            rospy.loginfo("Found Luci. Should be turning left")
            self.lights.put_pixels([(255, 255, 0)]*15) # Turn YELLOW
            self.base.go_forward(1)

        # Publish required message
        self.pub_.publish(self.msg)

    def shutdown(self):
        """
        command executed after Ctrl+C is pressed
        """
        rospy.loginfo("Stop LuciControl")
        self.pub_.publish(Twist())
        rospy.sleep(1)

if __name__ == "__main__":
    LuciControl()
