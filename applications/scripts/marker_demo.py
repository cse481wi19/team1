#!/usr/bin/env python

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

import rospy

def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass

def show_text_in_rviz(marker_publisher, text):
    marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=0,
                lifetime=rospy.Duration(15),
                pose=Pose(Point(0.5, 0.5, 0.45), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.6, 0.6, 0.6),
                header=Header(frame_id='base_link'),
                color=ColorRGBA(1.0, 1.0, 0.5, 0.8),
                text=text)
    marker_publisher.publish(marker)

def main():
	rospy.init_node('marker_demo')
	marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
	rospy.sleep(0.5)
	show_text_in_rviz(marker_publisher, 'Hello geoffrey')
	wait_for_time()

if __name__ == '__main__':
  main()

class NavPath(object):
    def __init__(self):
        self._path = []
            
    def callback(self, msg):
        rospy.loginfo(msg)
        if SOME_CONDITION:
            self._path.append(msg.foo.bar)

