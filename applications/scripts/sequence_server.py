#! /usr/bin/env python

from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers
import robot_api
import rospy
import tf
import numpy as np
from math import floor
import sensor_msgs.msg._CameraInfo
from threading import Thread, RLock
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ArTagReader(object):
    def __init__(self):
        self.markers = []

    def callback(self, msg):
        self.markers = msg.markers

    def getTagPose(self, tag):
        for marker in self.markers:
            if marker.id is int(tag):
                out = PoseStamped()
                out.header = marker.header
                out.pose = marker.pose.pose
                return out
        return None

class SequenceServer(object):
    def __init__(self):
        # Setup
        self.tag_reader = ArTagReader()
        self.ar_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback=self.tag_reader.callback)
        self.odom_sub = rospy.Subscriber("odom", Odometry, callback=self.saveCurrentPose)
        # self.move_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.head = robot_api.Head()
        self.base = robot_api.Base()
        self.expressions = robot_api.Expressions()
        self.tf = tf.TransformListener()
        self.currentPoseStamped = None
        self.store = SequenceStore()
        self.command_sequence = []
        print("Waiting for ActionClient Server...")
        self.ac.wait_for_server()  
        print("Waiting for AR markers...")
        while len(self.tag_reader.markers) is 0: pass
        print("Waiting to read current pose...")
        while self.currentPoseStamped is None: pass
        # Set current tag for saving (first one we see by default)
        self.in_progress_seq_tag = self.tag_reader.markers[0].id

    # Callbacks
    def saveCurrentPose(self, msg):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.currentPoseStamped = ps

    # Sequence Creation API
    def saveSequence(self, name):
        self.store.saveSequence(name, self.command_sequence)
    
    def emptySequence(self):
        self.command_sequence = []

    def setin_progress_seq_tag(self, tag):
        if (self.tag_reader.getTagPose(tag) != None):
            self.in_progress_seq_tag = tag
            return
        print("Unknown Tag!")

    def addCurrentPositionToSequence(self):
        # We know everything is in odom, so just save the relative position to the tag
        currPoseStamped = self.currentPoseStamped
        tagPoseStamped = self.tag_reader.getTagPose(self.in_progress_seq_tag)
        if not tagPoseStamped:
            print("Can't find active tag: " + str(self.in_progress_seq_tag))
        else:
            self.command_sequence.append(["goto", [self.in_progress_seq_tag, getOffsetFromPoses(tagPoseStamped.pose, currPoseStamped.pose)]])

    def addLookAtARTagToSequence(self):
        self.command_sequence.append(["look_at_tag", self.in_progress_seq_tag])

    def addFacialExpressionToSequence(self, expression):
        if expression not in ["nod_head", "shake_head", "happy", "sad", "neutral"]:
            print("Invalid facial expression: " + expression)
            return
        self.command_sequence.append(["facial_expression", expression])

    def addMovementToSequence(self, direction, length):
        if direction not in ["forward", "backward"]:
            print("Invalid direction: " + direction)
            return
        if length is not float or length is not int:
            print("Invalid length to move: " + str(length))
            return
        self.command_sequence.append(["move", [direction, length]])

    def addRotationToSequence(self, rads):
        if rads is not float or rads is not int:
            print("Invalid radians to rotate: " + str(rads))
            return
        self.command_sequence.append(["rotate", rads])

    # Sequence Loading API
    def executeSequence(self, name):
        seq = self.store.getSequence(name)
        if (seq == None):
            print("Sequence " + name + " does not exist or hasn't been saved.")
        for el in seq:
            cmd = el[0]
            if cmd is "goto":
                self.__goto(el[1][0], el[1][1])
            elif cmd is "look_at_tag":
                self.__look_at_tag(el[1])
            elif cmd is "facial_expression":
                self.__face_exp(el[1])
            elif cmd is "move":
                self.__move(el[1][0], el[1][1])
            elif cmd is "rotate":
                self.__rotate(el[1])
            else:
                print("Invalid sequence command:" + str(cmd))

    # Sequence Execution API (private, ideally)
    def __goto(self, tag, offset):
        tag_pose = self.tag_reader.getTagPose(tag)
        if not tag_pose:
            print("Tag does not exist: " + str(tag))
            return
        
        dest_pose = getPoseFromOffset(tag_pose.pose, offset)
        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = "odom"

        # Create goal pose
        dest_ps = PoseStamped()
        dest_ps.pose = dest_pose
        dest_ps.header = h

        # Publish goal to rviz
        goal = MoveBaseGoal()
        goal.target_pose = dest_ps
        self.ac.send_goal(goal)

        # Wait for move completion
        self.ac.wait_for_result()
        
        #while not poseEqual(self.currentPoseStamped.pose, dest_pose): pass
        #rospy.sleep(1)
    
    def __look_at_tag(self, tag):
        tag_pose = self.tag_reader.getTagPose(tag)
        if not tag_pose:
            print("Tag does not exist: " + str(tag))
            return
        self.head.look_at(tag_pose)

    def __face_exp(self, expression):
        if expression == "nod_head":
            self.expressions.nod_head()
        elif expression == "shake_head":
            self.expressions.shake_head()
        elif expression == "happy":
            self.expressions.be_happy()
        elif expression == "sad":
            self.expressions.be_sad()
        elif expression == "neutral":
            self.expressions.be_neutral()
        else:
            print("Invalid stored facial expression: " + str(expression))

    def __move(self, direction, length):
        if direction is "backward":
            direction = -direction
        self.base.go_forward(length)

    def __rotate(self, rads):
        SPEED = 0.5
        self.base.rotate(rads, SPEED)
        rospy.sleep(rads / SPEED + 1)

def poseEqual(pose, pose2):
    posX = floor(pose.position.x) is floor(pose2.position.x)
    posY = floor(pose.position.y) is floor(pose2.position.x)
    posZ = floor(pose.position.z) is floor(pose2.position.x)
    print(pose)
    print(pose2)
    orienX = True #floor(pose.orientation.x) is floor(pose2.orientation.x)
    orienY = True #floor(pose.orientation.y) is floor(pose2.orientation.x)
    orienZ = True #floor(pose.orientation.z) is floor(pose2.orientation.x)
    return posX and posY and posZ and orienX and orienY and orienZ

def getTransformFromPose(pose):
    orientation = pose.orientation
    m = tf.transformations.quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
    m[0, 3] = pose.position.x
    m[1, 3] = pose.position.y
    m[2, 3] = pose.position.z
    return m

def getPoseFromTransform(transform):
    out = Pose()
    orientation_out = tf.transformations.quaternion_from_matrix(transform)
    out.orientation = Quaternion(orientation_out[0], orientation_out[1], orientation_out[2], orientation_out[3])
    translation_out = tf.transformations.translation_from_matrix(transform)
    out.position = Point(translation_out[0], translation_out[1], translation_out[2])
    return out

def getOffsetFromPoses(base_pose, relative_pose):
    return np.dot(np.linalg.inv(getTransformFromPose(base_pose)), getTransformFromPose(relative_pose))

def getPoseFromOffset(base_pose, offset_pose):
    base_transform = getTransformFromPose(base_pose)
    base_offset_transform = np.dot(base_transform, offset_pose)
    return getPoseFromTransform(base_offset_transform)

def makeCamInfoPub():
    cam_info_pub = rospy.Publisher('/upward_looking_camera/camera_info', sensor_msgs.msg.CameraInfo, queue_size=10)
    msg = sensor_msgs.msg.CameraInfo()
    msg.header = Header()
    msg.header.seq = 0
    msg.header.stamp = rospy.get_time()
    msg.height = 480
    msg.width = 640
    msg.distortion_model = "plumb_bob"
    msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.K = [504.78216005824515, 0.0, 320.5, 0.0, 504.78216005824515, 240.5, 0.0, 0.0, 1.0]
    msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.P = [504.78216005824515, 0.0, 320.5, -0.0, 0.0, 504.78216005824515, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
    msg.binning_x = 0
    msg.binning_y = 0
    msg.roi = sensor_msgs.msg.RegionOfInterest()
    msg.roi.x_offset = 0
    msg.roi.y_offset = 0
    msg.roi.height = 0
    msg.roi.width = 0
    msg.roi.do_rectify = False

    while True:
        msg.header.seq = msg.header.seq + 1
        msg.header.stamp = rospy.get_time()
        cam_info_pub.publish(msg)
        rospy.sleep(0.1)

class SequenceStore(object):
    def __init__(self):
        self.store = {"sequences": {}}

    def saveSequence(self, name, sequence):
        self.store["sequences"][name] = sequence

    def getSequence(self, name):
        return self.store["sequences"][name]

def print_usage():
    print('Usage:')
    print('    list tags            # Lists all seen tags')
    print('    list seq_commands    # Lists commands in current sequence')
    print('    list sequences       # Lists names of saved sequences')
    print('    saveseq <name>       # Save current move sequence as <name>')
    print('    runseq  <name>       # Run saves sequence with name "name"')
    print('    reset                # Reset current move sequence')
    print('    settag <tag>         # Set active tag (ex. where the movement should be relative to)')
    print('    add <cmd> <params>   # Add a command the current move sequence')
    print('           Command <exp options>:')
    print('                expression <happy/sad/neutral/nod_head/shake_head>')
    print('                goto       # Go to current location')
    print('                lookAt     # Look at active AR tag')
    print('                rotate     <rads>')
    print('                move       <forward/backward> <length>')

def main():
    rospy.init_node("Sequence_Server")
    wait_for_time()
    t1 = Thread(target=makeCamInfoPub, args=[])
    t1.start()
    ss =  SequenceServer()
    while(True):
        input = raw_input("> ").split(" ")
        cmd = input[0]
        if cmd == "add":
            if len(input) < 2:
                print("what should I add?")
                return
            if input[1] == 'expression':
                if len(input) is not 3:
                    print("What expression?")
                    return
                ss.addFacialExpressionToSequence(input[2])
            elif input[1] == 'goto':
                ss.addCurrentPositionToSequence()
            elif input[1] == 'lookAt':
                ss.addLookAtARTagToSequence()
            elif input[1] == 'rotate':
                if len(input) is not 3:
                    print("Rotate by how much?")
                    return
                ss.addRotationToSequence(input[2])
            elif input[1] is 'move':
                if len(input) is not 4:
                    print("Correct args: <direction> <length>")
                ss.addMovementToSequence(input[2], int(input[3]))
            else:
                print('Can\'t add ' + str(input[1]))
        elif cmd == "list":
            if len(input) is not 2:
                print("what should I list?")
                return
            if input[1] == "tags":
                for tag in ss.tag_reader.markers:
                    print('   ' + str(tag.id))
            elif input[1] == "sequences":
                for name in ss.store.store["sequences"]:
                    print('   ' + name)
            elif input[1] == "seq_commands":
                for item in ss.command_sequence:
                    print(item)
            else:
                print('Can\'t list ' + str(input[1]))
        elif cmd == "saveseq":
            if len(input) is not 2:
                print("need the sequence name")
                return
            ss.saveSequence(input[1])
        elif cmd == "runseq":
            if len(input) is not 2:
                print("need the sequence name")
                return
            ss.executeSequence(input[1])
        elif cmd == "reset":
            ss.emptySequence()
        elif cmd == "settag":
            if len(input) is not 2:
                print("need the tag name")
                return
            ss.setin_progress_seq_tag(input[1])
        elif cmd == "help":
            print_usage()
        else:
            print("Invalid sequence command:" + str(cmd))
            print("Type help for a list")


if __name__ == '__main__':
    main()
