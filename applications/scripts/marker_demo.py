#!/usr/bin/env python

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import Odometry

import rospy

class NavPath(object):
	def __init__(self):
		# TODO: Should be Luci's starting point not 0,0,0
		initPoint = Point(0, 0, 0)
		self._path = [initPoint]
            
	def _computeDistance(self, msg):
		oldLoc = self._path[-1]
		# TODO: Generalize line to oldLoc = self._path[-1]
		currLoc = msg.pose.pose.position
		currPoint = Point(currLoc.x, currLoc.y, currLoc.z)
		if (abs(currLoc.x - oldLoc.x) > 0.1 or (abs(currLoc.y - oldLoc.y) > 0.1 or abs(currLoc.z - oldLoc.z) > 0.1)):
			self._path.append(currPoint)
			return True
		return False

	def callback(self, msg, marker_publisher):
		rospy.loginfo(msg)
		hasMovedAway = self._computeDistance(msg)
		if hasMovedAway: # TODO Replace condition
			# self._path.append(msg.pose.pose.position) # TODO Uncomment and build up list
			# show_text_in_rviz(marker_publisher, 'HELLO!')
			draw_path_in_rviz(marker_publisher, self._path)
		
def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass

def draw_path_in_rviz(marker_publisher, points):
    marker = Marker(
                type=Marker.LINE_STRIP, 
                id=0,
                lifetime=rospy.Duration(60),
				points=points,
                # pose=pose,
				pose=Pose(Point(0.5, 0.5, 0.45), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.01, 0, 0),
                header=Header(frame_id='base_link'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                )
    marker_publisher.publish(marker)

def show_text_in_rviz(marker_publisher, text):
    marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=0,
                lifetime=rospy.Duration(3),
                pose=Pose(Point(0.5, 0.5, 0.45), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.6, 0.6, 0.6),
                header=Header(frame_id='base_link'),
                color=ColorRGBA(1.0, 1.0, 0.5, 0.8),
                text=text)
    marker_publisher.publish(marker)

def main():
	rospy.init_node('marker_demo')
	nav_path = NavPath()
	marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
	# wait_for_time() # TODO Do we need this?
	rospy.Subscriber('odom', Odometry, nav_path.callback, marker_publisher)
	rospy.spin()

if __name__ == '__main__':
  main()

