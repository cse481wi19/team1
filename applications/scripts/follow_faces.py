#!/usr/bin/env python

import rospy
import robot_api
from geometry_msgs.msg import PointStamped, PoseStamped, Point, Pose, Quaternion
from std_msgs.msg import Int8
from tf import TransformListener
from threading import RLock
from robot_api import Head, Expressions
from math import sqrt

class FaceInteractionDemo(object):
    def __init__(self):
        self.move_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.lock = RLock()
        self.head = Head()
        self.expressions = Expressions()
        self.tf = TransformListener()
        self.lastFacePos = None
        rospy.sleep(0.5)

    # Gets an equivalent point in the reference frame "destFrame"
    def getPointStampedInFrame(self, pointStamped, destFrame):
        # Wait until we can transform the pointStamped to destFrame
        self.tf.waitForTransform(pointStamped.header.frame_id, destFrame, rospy.Time(), rospy.Duration(4.0))
        # Set point header to the last common frame time between its frame and destFrame
        pointStamped.header.stamp = self.tf.getLatestCommonTime(pointStamped.header.frame_id, destFrame)
        # Transform the point to destFrame
        return self.tf.transformPoint(destFrame, pointStamped)

    def onFacePos(self, posStamped):
        self.lastFacePos = posStamped

    def navigateTo(self):
        if (self.lastFacePos is not None and self.lock.acquire(blocking=False)):
            self.expressions.nod_head()
            pointStamped = self.getPointStampedInFrame(pointStamped, "base_link")
            distance_to_base = sqrt(pointStamped.point.x ** 2 + pointStamped.point.y ** 2 + pointStamped.point.z ** 2)
            unit_vector = { "x": pointStamped.point.x / distance_to_base,
                            "y": pointStamped.point.y / distance_to_base }

            pointStamped.point.x = unit_vector["x"] * (distance_to_base - 0.5)
            pointStamped.point.y = unit_vector["y"] * (distance_to_base - 0.5)
            
            quaternion = Quaternion()
            quaternion.w = 1

            pose = Pose()
            pose.position = pointStamped.point
            pose.orientation = quaternion

            poseStamped = PoseStamped()
            poseStamped.header = pointStamped.header
            pointStamped.pose = pose

            self.move_pub.publish(poseStamped)
            self.lock.release()

    def onFaceCount(self, int8):
        if (int8.data > 0):
            self.expressions.be_happy()
        else:
            self.expressions.be_sad()
            self.expressions.be_neutral()

def main():
    rospy.init_node('follow_faces')
    facesDemo = FaceInteractionDemo()

    # Trigger callback
    rospy.Subscriber('vision/face_count', Int8, facesDemo.onFaceCount)
    rospy.Subscriber('vision/most_confident_face_pos', PointStamped, facesDemo.onFacePos) 
    rospy.Subscriber('come_here_kuri', Int8, facesDemo.navigateTo) 
    rospy.spin()

if __name__ == '__main__':
    main()
