import rospy, actionlib

from actionlib_msgs.msg import GoalStatus
from actionlib.action_client import CommState

from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

from tf import TransformListener
from joint_state_reader import JointStateReader
from robot_api import Base

from math import atan2, ceil

""" 
This is set up for Kuri, but you can take inspiration for Fetch if you like.
"""
class Head(object):
    JOINT_PAN = 'head_1_joint'
    JOINT_TILT = 'head_2_joint'
    JOINT_EYES = 'eyelids_joint'
    JOINT_HEIGHT = 0.405
    PAN_LEFT = 0.78
    PAN_NEUTRAL = 0
    PAN_RIGHT = -PAN_LEFT
    TILT_UP = -0.92
    TILT_NEUTRAL = 0.0
    TILT_DOWN = 0.29
    EYES_OPEN = 0.0
    EYES_NEUTRAL = 0.1
    EYES_CLOSED = 0.41
    EYES_HAPPY = -0.16
    EYES_SUPER_SAD = 0.15
    EYES_CLOSED_BLINK = 0.35
    HEAD_NS = '/head_controller/follow_joint_trajectory' # found on real robot, previously thought to use /command
    EYES_NS = '/eyelids_controller/follow_joint_trajectory'
    EYES_FRAME = "/head_2_link"
    BASE_FRAME = "/base_link"

    # JS was set to none to try to get demo to work, probably shouldn't be
    # Js could stand for something like joint state controller or joint system
    def __init__(self, js=None, head_ns=None, eyes_ns=None):
        self._js = js
        self._tf = TransformListener()
        self._head_gh = None
        self._head_goal = None
        self._head_ac = actionlib.ActionClient(head_ns or self.HEAD_NS, FollowJointTrajectoryAction)
        self._eyes_gh = None
        self._eyes_goal = None
        self._eyes_ac = actionlib.ActionClient(eyes_ns or self.EYES_NS, FollowJointTrajectoryAction)
        self.jointReader = JointStateReader()
        self.base = Base()
        rospy.sleep(0.5)
        return

    def cancel(self):
        head_gh = self._head_gh
        eyes_gh = self._eyes_gh
        if head_gh:
            head_gh.cancel()
        self._head_goal = None
        self._head_gh = None
        if eyes_gh:
            eyes_gh.cancel()
        self._eyes_goal = None
        self._eyes_gh = None
        return

    def eyes_to(self, radians, duration=1.0, feedback_cb=None, done_cb=None):
        """
        Moves the robot's eye lids to the specified location in the duration
        specified
        
        :param radians: The eye position.  Expected to be between
        HeadClient.EYES_HAPPY and HeadClient.EYES_CLOSED
        
        :param duration: The amount of time to take to get the eyes to
        the specified location.
        
        :param feedback_cb: Same as send_trajectory's feedback_cb
        
        :param done_cb: Same as send_trajectory's done_cb
        """
        if(radians < self.EYES_HAPPY or radians > self.EYES_CLOSED):
            return
        point = JointTrajectoryPoint()
        point.positions.append(float(radians))
        point.time_from_start = rospy.Duration(duration)
        trajectory = JointTrajectory()
        trajectory.points.append(point)
        trajectory.joint_names.append(self.JOINT_EYES)
        return self.send_trajectory(traj=trajectory, feedback_cb=feedback_cb, done_cb=done_cb)

    def is_done(self):
        active = {
         GoalStatus.PENDING, GoalStatus.RECALLING,
         GoalStatus.ACTIVE, GoalStatus.PREEMPTING}
        if self._head_gh:
            if self._head_gh.get_goal_status() in active:
                return False
        if self._eyes_gh:
            if self._eyes_gh.get_goal_status() in active:
                return False
        return True

    def pan_and_tilt(self, pan, tilt, duration=1.0, feedback_cb=None, done_cb=None):
        """
        Moves the robot's head to the point specified in the duration
        specified
        
        :param pan: The pan - expected to be between HeadClient.PAN_LEFT
        and HeadClient.PAN_RIGHT
        
        :param tilt: The tilt - expected to be between HeadClient.TILT_UP
        and HeadClient.TILT_DOWN
        
        :param duration: The amount of time to take to get the head to
        the specified location.
        
        :param feedback_cb: Same as send_trajectory's feedback_cb
        
        :param done_cb: Same as send_trajectory's done_cb
        """
        if(pan < self.PAN_RIGHT or pan > self.PAN_LEFT):
            return
        if(tilt > self.TILT_DOWN or tilt < self.TILT_UP):
            return
        point = JointTrajectoryPoint()
        point.positions = [float(pan), float(tilt)]
        point.time_from_start = rospy.Duration(duration)
        trajectory = JointTrajectory()
        trajectory.points = [point]
        trajectory.joint_names = [self.JOINT_PAN, self.JOINT_TILT]
        return self.send_trajectory(traj=trajectory, feedback_cb=feedback_cb, done_cb=done_cb)

    def send_trajectory(self, traj, feedback_cb=None, done_cb=None):
        """
        Sends the specified trajectories to the head and eye controllers
        
        :param traj: A trajectory_msgs.msg.JointTrajectory.  joint_names
        are expected to match HeadClient.JOINT_PAN, JOINT_TILT and JOINT_EYES
        
        :param feedback_cb: A callable that takes one parameter - the feedback
        
        :param done_cb: A callable that takes two parameters - the goal status
        the goal handle result
        """
        for point in traj.points:
            for k in ('velocities', 'accelerations', 'effort'):
                if getattr(point, k) is None:
                    setattr(point, k, [])

            if isinstance(point.time_from_start, (int, float)):
                point.time_from_start = rospy.Duration(point.time_from_start)

        goal = FollowJointTrajectoryGoal(trajectory=traj)

        def _handle_transition(gh):
            gh_goal = gh.comm_state_machine.action_goal.goal
            if done_cb is not None and (id(self._eyes_goal) == id(gh_waitForTransformgoal) or id(self._head_goal) == id(gh_goal)):
                if gh.get_comm_state() == CommState.DONE:waitForTransform
                    if (id(self._head_goal) == id(gh_goal)):waitForTransform
                        self.currentPan = self._head_goal.trajectory.waitForTransformpoints[0].positions[0]
                        self.currentTilt = self._head_goal.trajectory.points[0].positions[1]
                    done_cb(gh.get_goal_status(), gh.get_result())
            return

        def _handle_feedback(gh, feedback):
            gh_goal = gh.comm_state_machine.action_goal.goal
            if feedback_cb is not None and (id(self._eyes_goal) == id(gh_goal) or id(self._head_goal) == id(gh_goal)):
                feedback_cb(feedback)
            return

        if self.JOINT_EYES in traj.joint_names:
            if not self._eyes_ac:
                return False
            self._eyes_goal = goal
            self._eyes_ac.wait_for_server()
            self._eyes_gh = self._eyes_ac.send_goal(goal, _handle_transition, _handle_feedback)
        else:
            if not self._head_ac:
                return False
            self._head_goal = goal
            self._head_ac.wait_for_server()
            self._head_gh = self._head_ac.send_goal(goal, _handle_transition, _handle_feedback)
        return True

    # Gets an equivalent point in the reference frame "destFrame"
    def getPointInFrame(self, pointStamped, destFrame):
        # Wait until we can transform the pointStamped to destFrame
        self._tf.waitForTransform(pointStamped.header.frame_id, destFrame, rospy.Time(), rospy.Duration(4.0))
        # Set point header to the last common frame time between its frame and destFrame
        pointStamped.header.stamp = self._tf.getLatestCommonTime(pointStamped.header.frame_id, destFrame)
        # Transform the point to destFrame
        return self._tf.transformPoint(destFrame, pointStamped).point

    # Move body to face this point (does not change head joints)
    def point_base_at(self, pointStamped, fullBody = False):
        # Transform the point to the base frame
        transformedPoint = self.getPointInFrame(pointStamped, self.BASE_FRAME)
        # Get the radians we need to rotate the base in order to point correctly
        rotation_rads = atan2(transformedPoint.y, transformedPoint.x)
        # Do the rotation
        ROTATION_SPEED = 0.5
        rospy.loginfo(rotation_rads)
        self.base.turn(rotation_rads)
        rospy.sleep(ceil(rotation_rads / ROTATION_SPEED)) # Wait for rotation to complete

    # Move head to look at a point; move the body if point if out of range of head (and fullBody = true)
    # Returns false if point is out of range, true if successful
    def look_at(self, pointStamped, fullBody = False):
        # Transform the point to the eye frame
        eyeFramePoint = self.getPointInFrame(pointStamped, self.EYES_FRAME)

        # Get the radians we need to rotate the head
        pan_rads = self.jointReader.get_joint(self.JOINT_PAN) + atan2(eyeFramePoint.y, eyeFramePoint.x)
        
        # If the rotation is out of the head's range of rotation
        if pan_rads > self.PAN_LEFT or pan_rads < self.PAN_RIGHT:
            if not fullBody: return False
            
            # Rotate the body to point toward the point
            self.point_base_at(pointStamped)
            
            # Then pan the head to look towards it
            eyeFramePoint = self.getPointInFrame(pointStamped, self.EYES_FRAME)
            pan_rads = self.jointReader.get_joint(self.JOINT_PAN) + atan2(eyeFramePoint.y, eyeFramePoint.x)

        # Get the amount we should tilt the head
        tilt_rads = self.jointReader.get_joint(self.JOINT_TILT) - atan2(eyeFramePoint.z, eyeFramePoint.x)
        
        # Return if we aren't allowed to move the body and the tilt is out of range
        if (not fullBody) and (tilt_rads < self.TILT_UP or tilt_rads > self.TILT_DOWN): return False
        
        # Otherwise, if the tilt is out of range, move the body backward until the point is in tilt range
        while fullBody and (tilt_rads < self.TILT_UP or tilt_rads > self.TILT_DOWN):
            self.base.go_forward(-0.1)
            rospy.sleep(1) # Wait for movement to complete
            eyeFramePoint = self.getPointInFrame(pointStamped, self.EYES_FRAME)
            tilt_rads = self.jointReader.get_joint(self.JOINT_TILT) - atan2(eyeFramePoint.z, eyeFramePoint.x)

        self.pan_and_tilt(pan_rads, tilt_rads)
        return True

    def shutdown(self):
        self.cancel()
        self._head_ac = None
        self._eyes_ac = None
        return

    def wait_for_server(self, timeout=rospy.Duration(0.0)):
        return self._head_ac.wait_for_server(timeout) and self._eyes_ac.wait_for_server(timeout)

    def wait_for_done(self, timeout):
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(timeout):
            if self.is_done():
                return True
            rate.sleep()

        return False
