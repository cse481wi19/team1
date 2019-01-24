#! /usr/bin/env python

import rospy
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
        # TODO: Create publisher
        # rospy.init_node('robot_cleaner', anonymous=True)
        # Prod: mobile_base/commands/velocity
        self.pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist)
        pass

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
        # TODO: Create Twist msg
        # TODO: Fill out msg
        # TODO: Publish msg

        # Create Twist Message
        msg = Twist()
        msg.linear.x = linear_speed
        msg.linear.y = linear_speed
        msg.linear.z = linear_speed

        msg.angular.x = angular_speed
        msg.angular.y = angular_speed
        msg.angular.z = angular_speed

        self.pub.publish(msg)

        # rospy.logerr('Not implemented.')

    def stop(self):
        """Stops the mobile base from moving.
        """
        # TODO: Publish 0 velocity
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        self.pub.publish(msg)
        # rospy.logerr('Not implemented.')
