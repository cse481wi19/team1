#!/usr/bin/python
import rospy
import robot_api
import alsaaudio 

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry

class LuciControl(object):
    """Class to handle turtlebot simulation control using voice"""

    def __init__(self):

        # initialize node
        rospy.init_node("luci_control")
        rospy.on_shutdown(self.shutdown)
        # Initializing publisher with buffer size of 10 messages
        self.pub_grammar = rospy.Publisher("grammar_data", String, queue_size=10)

        # TODO: Add flexibility to pass in custom language model
        # TODO: Find filepaths for each of these
        self.class_lm = 'corpus/luci.lm'
        self.dict = 'corpus/luci.dic'

        # TODO: Set this file from "/usr/share/pocketsphinx/model/hmm/en_US/hub4wsj_sc_8k"
        # self.hmm = hmm

        # All params satisfied. Starting recognizer
        self.start_recognizer()

        # TODO Remove after testing
        # Default values for turtlebot_simulator
        self.speed = 0.2

        # Intializing message type
        self.msg = Twist()
        self.base = robot_api.Base()
        self.lights = robot_api.Lights()
        self.sound_source = robot_api.SoundSource()

        # Initializing publisher with buffer size of 10 messages
        self.pub_move = rospy.Publisher("mobile_base/commands/velocity", Twist, queue_size=10)

        # Initialize alsa audio PCM: (Using 16000Hz, 16 Bit Encoding, 1 Channel)
        self.inp = alsaaudio.PCM(alsaaudio.PCM_CAPTURE, alsaaudio.PCM_NONBLOCK)
        self.inp.setchannels(1)
        self.inp.setrate(16000)
        self.inp.setformat(alsaaudio.PCM_FORMAT_S16_LE)
        self.inp.setperiodsize(160)

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
        self.pub_move.publish(self.msg)

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
        rospy.Subscriber("jsgf_audio", String, self.process_audio)
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
        self.pub_move.publish(Twist())
        rospy.sleep(1)

if __name__ == "__main__":
    LuciControl()
