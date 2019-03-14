import time
import rospy
import robot_api
import alsaaudio 
import pyttsx

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
from threading import Thread
from patient_monitor.msg import Alert
from patient_monitor.srv import AddAlert

class Speech(object):
    """Class to control Luci's speech interactions """

    def __init__(self):

        rospy.on_shutdown(self.shutdown)

        # Create a publisher for the grammar data and raw audio
        self.pub_grammar = rospy.Publisher("grammar_data", String, queue_size=10)
        self.pub_audio = rospy.Publisher("raw_audio", String, queue_size=10)

        self.alert_pub = rospy.Publisher('patient_monitor/alerts', Alert, queue_size=10)
        self.alert_client(1)
        #self.alert_client(2)
        #self.alert_client(3)
        # Subscribe to grammar data output
        rospy.Subscriber("grammar_data", String, self.parse_results)

        # Set language model and dictionary
        self.class_lm = 'corpus/final.lm'
        self.dict = 'corpus/final.dic'

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

        start_thread = Thread(target=self._start)
        start_thread.start()

    def _start(self):
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
        #self.expressions.be_neutral()
        sleep_time = 4

        if (file[0] == 'reminder_2'):
            sleep_time = 10
        elif (file[0] == 'alert_2'):
            sleep_time = 10
        elif (file[0] == 'alert_frank'):
            sleep_time = 30

        # TODO rsync over final_interactions WONT WORK WITHOUT THAT
        sound = self.sound_source.play('/home/team1/catkin_ws/src/sound_effects/final_interactions/' + file)
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
        if self._isDetected(detected_words, ['DAY']):
            # PWD: Hey Luci, what day is it?
            # Luci: Today is Wednesday, it's also bingo night!
            self._play('bingo_1.wav')
        elif self._isDetected(detected_words, ['NEED']):
            # PWD: Great, do I need anything!
            # Luci: Yes! Grab your bingo card from my bag
            self._play('bingo_2.wav')
        elif self._isDetected(detected_words, ['WANT']):
            # PWD: I don't want to.
            # Luci: I understand your frustration, but you have to. Please don't
            # make me call the nurse.
            self._play('reminder_2.wav')
        elif self._isDetected(detected_words, ['HELP']):
            # PWD: Help...where am I?
            # Luci: What is wrong? I have alerted the nurse
            self._play('alert_1.wav')
            self.alert_pub.publish(Alert([], 1, 'Patient Needs Help'))
        elif self._isDetected(detected_words, ['HOME']):
            # PWD: I don't know where I am, I want to go home
            # Luci: Stay calm, here's some music while we wait for the nurse.
            self._play('alert_2.wav')
            self._play('alert_frank.wav')
        
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

    def alert_client(self):
        rospy.wait_for_service('/patient_monitor/alerts/add_alert')
        try:
            add_alert = rospy.ServiceProxy('/patient_monitor/alerts/add_alert', AddAlert)
            resp = add_alert(id, 'patient_monitor/alerts')
            return resp.success
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def shutdown(self):
        """
        command executed after Ctrl+C is pressed
        """
        rospy.loginfo("Stop Speech")
        rospy.sleep(1)