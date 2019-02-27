#!/usr/bin/python

import time
import rospy
import robot_api
import alsaaudio 
import pyttsx

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
from threading import Thread

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

    def _greeting(self):
        """Function to greet people upon happy words"""
        rospy.loginfo("Received a Greeting.")
        self.expressions.be_happy()
        self.lights.put_pixels([(255, 255, 0)]*15) # YELLOW
        # engine = pyttsx.init()
        # engine.setProperty('voice', 'english+f1')
        # message = "hello how are you"
        # engine.say(message)
        # engine.runAndWait()
        sound = self.sound_source.play('/home/team1/catkin_ws/src/sound_effects/interactions/1_1.wav')
        rospy.sleep(1)
        self.sound_source.cancel(sound)
        rospy.sleep(2)
 
    def _alert(self):
        """Function to alert nurse upon distress words"""
        rospy.loginfo("Received an Alert.")
        self.expressions.be_sad()
        self.lights.put_pixels([(255,0,0)]*15) # RED
        # engine = pyttsx.init()
        # engine.setProperty('voice', 'english+f1')
        # message = "calling a nurse"
        # engine.say(message)
        # engine.runAndWait()
        sound = self.sound_source.play('/home/team1/catkin_ws/src/sound_effects/interactions/3_2.wav')
        rospy.sleep(1)
        self.sound_source.cancel(sound)
        rospy.sleep(2)

    def _show_agreement(self):
        """Function to communicate agreement with given words"""
        rospy.loginfo("Received an Agreement.")
        self.expressions.nod_head()
        self.lights.put_pixels([(102,255,102)]*15) # GREEN
        # engine = pyttsx.init()
        # engine.setProperty('voice', 'english+f1')
        # message = "yes"
        # engine.say(message)
        # engine.runAndWait()
        sound = self.sound_source.play('/home/team1/catkin_ws/src/sound_effects/interactions/2_1.wav')
        rospy.sleep(1)
        self.sound_source.cancel(sound)
        rospy.sleep(2)


    def _neutral(self):
        """Function to neutrally respond to given words"""
        rospy.loginfo("Received a Neutral.")
        self.expressions.be_neutral()
        self.lights.put_pixels([(0, 0, 255)]*15) # BLUE
        # engine = pyttsx.init()
        # engine.setProperty('voice', 'english+f1')
        # message = "okay"
        # engine.say(message)
        # engine.runAndWait()
        sound = self.sound_source.play('/home/team1/catkin_ws/src/sound_effects/interactions/5_1.wav')
        rospy.sleep(1)
        self.sound_source.cancel(sound)
        rospy.sleep(2)

    def _isDetected(self, detected_words, words):
        """Function to check if any of the given words are detected"""
        for word in words:
            if word in detected_words.data:
                return True
        return False

    def parse_results(self, detected_words): #pylint: disable=too-many-branches
        """Function to perform action (change colors/expression) on detected word"""
        if self._isDetected(detected_words, ['HI', 'HELLO', 'HEY', 'LUCI']):
            self._greeting()
        elif self._isDetected(detected_words, ['NURSE', 'HELP', 'SAD', 'MAD']):       
            self._alert()
        elif self._isDetected(detected_words, ['FRANK SINATRA', 'BINGO', 'FRIEND', 'WEATHER']):
            self._show_agreement()
        else:
            self._neutral()

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

if __name__ == "__main__":
    LuciControl()
