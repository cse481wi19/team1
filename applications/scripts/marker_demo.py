#!/usr/bin/env python

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import Odometry

import rospy

class NavPath(object):
	def __init__(self):
		self._path = []
            
	# Only appends point if Kuri is no current location recorded | Kuri has moved more than 0.1 x/y direction
	def _appendPoint(self, msg):
		currLoc = msg.pose.pose.position
		currPoint = Point(currLoc.x, currLoc.y, 0)

		if not self._path:
			self._path.append(currPoint)
		else: 
			oldLoc = self._path[-1]
			if (abs(currLoc.x - oldLoc.x) > 0.01 or (abs(currLoc.y - oldLoc.y) > 0.01)):
				self._path.append(currPoint)
		

	def callback(self, msg, marker_publisher):
		rospy.loginfo(msg)
		self._appendPoint(msg)
		draw_path_in_rviz(marker_publisher, self._path)

def draw_path_in_rviz(marker_publisher, points):
    marker = Marker(
                type=Marker.LINE_STRIP, 
                id=0,
				points=points,
				pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.01, 0, 0),
                header=Header(frame_id='odom'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                )
    marker_publisher.publish(marker)

def main():
	rospy.init_node('marker_demo')
	nav_path = NavPath()
	marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
	rospy.Subscriber('odom', Odometry, nav_path.callback, marker_publisher)
	rospy.spin()

if __name__ == '__main__':
  main()

