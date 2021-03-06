import rospy
import robot_api
from mobile_base_driver.msg import Led # the ChestLeds is an array of 15 LED, not sure if this is necessary
from mobile_base_driver.msg import ChestLeds

class Lights(object):
    """
    Controls state (on/off + color) of chest LEDs.
    
    LED indices are as follows (if looking at the robot straight on):
             [12]       [13]
        [11]                 [14]
                [4]   [5]
             [3]   [0]   [6]
                [2]   [1]
        [10]                 [7]
              [9]       [8]
    
    
    .. code::python
    
        import robot_api
    
        chest_leds = robot_api.Lights()
        chest_leds.put_pixels(
            [Lights.RED] * Lights.NUM_LEDS
        )
    
    """
    OFF = (0, 0, 0)
    ON = (255, 255, 255)
    HALF = (127, 127, 127)
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    ORANGE = (245, 184, 0)
    BLUE = (0, 0, 255)
    YELLOW = (255, 255, 51)
    NUM_LEDS = 15
    IDX_CENTER = 0
    IDX_INNER_BOTTOM_LEFT = 1
    IDX_INNER_BOTTOM_RIGHT = 2
    IDX_INNER_RIGHT = 3
    IDX_INNER_UPPER_RIGHT = 4
    IDX_INNER_UPPER_LEFT = 5
    IDX_INNER_LEFT = 6
    IDX_OUTER_BOTTOM_MID_LEFT = 7
    IDX_OUTER_BOTTOM_LOW_LEFT = 8
    IDX_OUTER_BOTTOM_LOW_RIGHT = 9
    IDX_OUTER_BOTTOM_MID_RIGHT = 10
    IDX_OUTER_UPPER_MID_RIGHT = 11
    IDX_OUTER_UPPER_TOP_RIGHT = 12
    IDX_OUTER_UPPER_TOP_LEFT = 13
    IDX_OUTER_UPPER_MID_LEFT = 14
    LED_ALL = range(NUM_LEDS)
    LED_CENTER = range(IDX_CENTER + 1)
    LED_MID_RING = range(IDX_INNER_BOTTOM_LEFT, IDX_INNER_LEFT + 1)
    LED_OUTER_RING = range(IDX_OUTER_BOTTOM_MID_LEFT, IDX_OUTER_UPPER_MID_LEFT + 1)
    ALL_OFF = [
     OFF] * NUM_LEDS
    ALL_ON = [ON] * NUM_LEDS
    ALL_HALF = [HALF] * NUM_LEDS

    @classmethod
    def all_leds(cls, color):
        # TODO: Turn all LEDS to `color`
        # this was taken from red comments above
        chest_leds = robot_api.Lights()
        chest_leds.put_pixels(
            [color] * Lights.NUM_LEDS
        )
	rospy.loginfo(cls)

    pass

    def __init__(self):
        self._light_pub = rospy.Publisher("/mobile_base/commands/chest_leds", ChestLeds, queue_size=10, latch=True)
        self.off()

    def shutdown(self):
        self._light_pub.unregister()

    def put_pixels(self, pixels):
        """ set the LEDs to the values in pixels. 
            :param pixels: an array of 3-ary tuples
        """
        self._last_pixels = list(pixels)
        msg = ChestLeds()

        for x in range(15):
            msg.leds[x].red = int(self._last_pixels[x][0])
            msg.leds[x].green = int(self._last_pixels[x][1])
            msg.leds[x].blue = int(self._last_pixels[x][2])

        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(0.25):
            self._light_pub.publish(msg)


    def get_pixels(self):
        return list(self._last_pixels)

    def off(self):
        self.put_pixels(Lights.ALL_OFF)
# okay decompiling ./opt/gizmo-master-cbb8e0c/lib/python2.7/dist-packages/mobile_base/chest_light_client.pyc
