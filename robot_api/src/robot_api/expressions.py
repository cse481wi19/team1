import rospy
from robot_api import Head
from robot_api import Lights

class Expressions(object):

    def __init__(self):
        self._head = Head()
        self._lights = Lights()

    def nod_head(self):
        # Tilt Limits: -0.92 and 0.29
        duration = 0.05
        for x in range(3):
            self._head.pan_and_tilt(0, -0.3, duration)
            rospy.sleep(0.2)
            self._head.pan_and_tilt(0, 0.1, duration)
            rospy.sleep(0.2)
        self._head.pan_and_tilt(0, 0, duration)
        rospy.sleep(1)

    def shake_head(self):
        # Pan Limits: -0.78 and 0.78
        duration = 0.05
        for x in range(2):
            self._head.pan_and_tilt(0.5, 0, duration)
            rospy.sleep(0.2)
            self._head.pan_and_tilt(-.5, 0, duration)
            rospy.sleep(0.2)
        self._head.pan_and_tilt(0, 0, duration)
        rospy.sleep(1)
    
    def be_happy(self):
        # Happy eyes
        self._head.eyes_to(-0.16)

        # Yellow Lights
        self._lights.put_pixels(
            [(255, 255, 137)] * 15
        )

        # Happy Sound
        # TODO - implement

        return
    
    def be_sad(self):
         # Sad eyes
        self._head.eyes_to(0.15)

        # Blue Lights
        self._lights.put_pixels(
            [(136, 190, 224)] * 15
        )

        # Sad Sound
        # TODO - implement

        return

    def be_neutral(self):
         # Neutral eyes
        self._head.eyes_to(0.1)

        #  Lights
        self._lights.put_pixels(
            [(232, 186, 255)] * 15
        )

        return