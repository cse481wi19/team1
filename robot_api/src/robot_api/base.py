#! /usr/bin/env python

import rospy
import math
import tf.transformations as tfs
from geometry_msgs.msg import Twist


class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = fetch_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        # Create publisher
        # Prod: mobile_base/commands/velocity
        self.pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist)
        # self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist)
	# Create subscriber to odom
	self._odom_sub = rospy.Subscriber('odom', nav_msgs.msg.Odometry, callback=self._odom_callback)
        pass
    def _odom_callback(self, msg)
	# TODO
    
    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        # Create Twist Message
        msg = Twist()

        # Fill out message
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed

        # Publish message
        self.pub.publish(msg)

        # rospy.logerr('Not implemented.')

    def stop(self):
        """Stops the mobile base from moving.
        """
        # Publish 0 velocity
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        self.pub.publish(msg)
        # rospy.logerr('Not implemented.')

    def quaternion_to_yaw(q):
	m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
	x = m[0, 0]
	y = m[1, 0]
	theta_rads = math.atan2(y, x)
	theta_degs = theta_rads * 180 / math.pi
	return theta_degs
