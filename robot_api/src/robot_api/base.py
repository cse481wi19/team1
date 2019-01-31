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

    LATEST_ODOM = null
    NO_MSG = True

    def __init__(self):
        # Create publisher
        # Prod: mobile_base/commands/velocity
        self.pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist)
        # self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist)

        # Create subscriber to odom
        self._odom_sub = rospy.Subscriber('odom', nav_msgs.msg.Odometry, callback=self._odom_callback)
        pass

    def _odom_callback(self, msg)
	    NO_MSG = False
        LATEST_ODOM = msg
    
    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance.

        It's recommended that the robot move slowly. If the robot moves too
        quickly, it may overshoot the target. Note also that this method does
        not know if the robot's path is perturbed (e.g., by teleop). It stops
        once the distance traveled is equal to the given distance or more.

        Args:
            distance: The distance, in meters, to move. A positive value
                means forward, negative means backward.
            speed: The speed to travel, in meters/second.
        """
        # TODO: rospy.sleep until the base has received at least one message on /odom
        while NO_MSG:
            rospy.sleep(0.025)

        # TODO: record start position, use Python's copy.deepcopy
        start = copy.deepcopy(LATEST_ODOM)

        rate = rospy.Rate(10)
        # TODO: CONDITION should check if the robot has traveled the desired distance
        absDistance = abs(distance)
        # TODO: Be sure to handle the case where the distance is negative!
        while absDistance > 0:
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            direction = -1 if distance < 0 else 1
            self.move(direction * speed, 0)
            absDistance -= direction * speed
            rate.sleep()

    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.

        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The angular speed to rotate, in radians/second.
        """
        # TODO: rospy.sleep until the base has received at least one message on /odom
        # TODO: record start position, use Python's copy.deepcopy
        start = copy.deepcopy(LATEST_ODOM)
        # TODO: What will you do if angular_distance is greater than 2*pi or less than -2*pi?
        rate = rospy.Rate(10)
        # TODO: CONDITION should check if the robot has rotated the desired amount
        # TODO: Be sure to handle the case where the desired amount is negative!
        while CONDITION:
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            direction = -1 if angular_distance < 0 else 1
            self.move(0, direction * speed)
            rate.sleep()

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
