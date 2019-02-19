import rospy
from robot_api import Head, Lights, SoundSource

class Expressions(object):

    def __init__(self):
        self._head = Head()
        self._lights = Lights()
        self._sound_source = SoundSource('Luci')

    def nod_head(self):
        # Tilt Limits: -0.92 and 0.29
        duration = 0.05
        self.be_neutral()
        for x in range(3):
            self._head.pan_and_tilt(0, -0.3, duration)
            rospy.sleep(0.2)
            self._head.pan_and_tilt(0, 0.1, duration)
            rospy.sleep(0.2)
        self.be_neutral()

    def shake_head(self):
        # Pan Limits: -0.78 and 0.78
        duration = 0.05
        self.be_neutral()
        for x in range(2):
            self._head.pan_and_tilt(0.5, 0, duration)
            rospy.sleep(0.2)
            self._head.pan_and_tilt(-.5, 0, duration)
            rospy.sleep(0.2)
        self.be_neutral()
    
    def be_happy(self):
        # Happy eyes
        self._head.eyes_to(-0.16)

        # Yellow Lights
        self._lights.put_pixels(
            [(255, 255, 12)] * 15
        )

        # Happy Sound
        sound = self._sound_source.play('/home/team1/catkin_ws/src/sound_effects/happy_mario.wav')
        rospy.sleep(1)
        self._sound_source.cancel(sound)

    def temp_happy(self):
        # Happy eyes
        self._head.eyes_to(-0.16)
        self._head.pan_and_tilt(0, -0.2, 0.05)

        # Yellow Lights
        self._lights.put_pixels(
            [(255, 255, 12)] * 15
        )

        # Happy Sound
        sound = self._sound_source.play('/home/team1/catkin_ws/src/sound_effects/happy_mario.wav')
        rospy.sleep(1)
        self._sound_source.cancel(sound)
        rospy.sleep(2)

        # Reset attributes except eyes
        self.be_neutral()
    
    def be_sad(self):
         # Sad eyes
        self._head.eyes_to(0.15)

        # Blue Lights
        self._lights.put_pixels(
            [(0, 0, 224)] * 15
        )

        # Sad Sound
        sound = self._sound_source.play('/home/team1/catkin_ws/src/sound_effects/sad_mario.wav')
        rospy.sleep(1)
        self._sound_source.cancel(sound)

    def temp_sad(self):
         # Sad eyes
        self._head.eyes_to(0.15)
        self._head.pan_and_tilt(0, 0.2, 0.05)

        # Blue Lights
        self._lights.put_pixels(
            [(0, 0, 224)] * 15
        )

        # Sad Sound
        sound = self._sound_source.play('/home/team1/catkin_ws/src/sound_effects/sad_mario.wav')
        rospy.sleep(1)
        self._sound_source.cancel(sound)
        rospy.sleep(2)

        # Reset attributes except eyes
        self.be_neutral()

    def be_neutral(self):
        self.reset_head()
        self.reset_eyes()
        self.reset_lights()

    def reset_head(self):
        self._head.pan_and_tilt(0, 0, 0.05)

    def reset_eyes(self):
        self._head.eyes_to(0.05)

    def reset_lights(self):
        # No Lights
        self._lights.put_pixels(
            [(0, 0, 0)] * 15
        )
