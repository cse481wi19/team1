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


        # eyelids
        # lights
        # sound
        return
    
    # def be_sad(self):

    # def be_neutral(self):