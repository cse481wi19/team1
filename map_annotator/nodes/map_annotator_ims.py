#!/usr/bin/env python
import rospy
import pickle
import os.path
import robot_api #Travis added this
import math # Travis added this
import nav_msgs.msg._Odometry
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from map_annotator.srv import ManageMarker, ManageMarkerResponse
from map_annotator.msg import Markers
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

def processFeedback(feedback):
    s = "Feedback from marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"

    mp = ""
    if feedback.mouse_point_valid:
        mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id

    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo( s + ": button click" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        rospy.loginfo( s + ": pose changed")
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        rospy.loginfo( s + ": mouse down" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        rospy.loginfo( s + ": mouse up" + mp + "." )

def makeMarker(name):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "map"
    int_marker.name = name
    int_marker.description = name
    int_marker.pose.position.x = 1
    int_marker.pose.orientation.w = 1

    name_marker = Marker()
    name_marker.type = Marker.TEXT_VIEW_FACING
    name_marker.text = name
    name_marker.pose.orientation.w = 1
    name_marker.pose.position.z = .2
    name_marker.scale.x = 0.3
    name_marker.scale.y = 0.3
    name_marker.scale.z = 0.3
    name_marker.color.r = 0.0
    name_marker.color.g = 0.0
    name_marker.color.b = 0.0
    name_marker.color.a = 1.0

    
    marker = Marker()
    marker.type = Marker.CYLINDER
    marker.pose.orientation.w = 1
    marker.scale.x = 0.25
    marker.scale.y = 0.25
    marker.scale.z = 0.05
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0

    arr_marker = Marker()
    arr_marker.type = Marker.ARROW
    arr_marker.pose.orientation.w = 1
    arr_marker.pose.position.z = .1
    arr_marker.scale.x = 0.25
    arr_marker.scale.y = 0.20
    arr_marker.scale.z = 0.05
    arr_marker.color.r = 1.0
    arr_marker.color.g = 0.5
    arr_marker.color.b = 0.5
    arr_marker.color.a = 1.0
    

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    control.always_visible = True
    control.markers.append(marker)
    control.markers.append(arr_marker)
    control.markers.append(name_marker)
    int_marker.controls.append(control)

    return int_marker

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class MarkersServer(object):
    ## Initialization
    def __init__(self):
        # Init Server
        self.server = InteractiveMarkerServer("map_annotator")
        self.list_pub = rospy.Publisher('map_annotator/marker_list', Markers, queue_size=10, latch=True)
        self.move_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.markers = []
        '''# Travis added
        self._odom_sub = rospy.Subscriber('odom', nav_msgs.msg.Odometry, callback=self._odom_callback)
        self.base = robot_api.Base()
        self.LATEST_ODOM = None
        '''# end of Travis added

    # Travis added this function
    #def _odom_callback(self, msg):
    #    self.LATEST_ODOM = msg

    def load_markers_from_file(self, path):
        if not os.path.isfile(path): return False
        file_in = open(path, "rb")
        markers = pickle.load(file_in)
        for marker in markers:
            self.addMarker(marker)
        return True

    def save_markers_to_file(self, path):
        markers = []
        for name in self.markers:
            markers.append(self.server.get(name))
        file_out = open(path, "wb")
        pickle.dump(markers, file_out)
        file_out.close()

    ## Services
    def handle_manage_marker(self, request):
        response = ManageMarkerResponse()
        response.message = "ERROR_INVALID_CMD"
        if request.cmd.lower() == 'create':
            if (self.createAndAddMarker(request.markerName)):
                response.message = "SUCCESS_CREATE"
            else:
                response.message = "ERROR_MARKER_EXISTS"
        elif request.cmd.lower() == 'delete':
            if (self.deleteMarker(request.markerName)):
                response.message = "SUCCESS_DELETE"
            else:
                response.message = "ERROR_MARKER_DOES_NOT_EXIST"
        elif request.cmd.lower() == 'goto':
            if (self.gotoMarker(request.markerName)):
                response.message = "SUCCESS_GOTO"
            else:
                response.message = "ERROR_MARKER_DOES_NOT_EXIST"
        elif request.cmd.lower() == 'rename':
            if (self.renameMarker(request.markerName, request.newMarkerName)):
                response.message = "SUCCESS_RENAME"
            else:
                response.message = "ERROR_MARKER_DOES_NOT_EXIST"
        return response

    ## Marker Support Code
    def createAndAddMarker(self, name):
        if name in self.markers: return False
        self.server.insert(makeMarker(name), processFeedback)
        self.server.applyChanges()
        self.markers.append(name)
        self.list_pub.publish(self.markers)
        return True

    def addMarker(self, marker):
        if marker.name in self.markers: return False
        self.server.insert(marker, processFeedback)
        self.server.applyChanges()
        self.markers.append(marker.name)
        self.list_pub.publish(self.markers)
        return True

    def deleteMarker(self, name):
        if not name in self.markers: return False
        ret = self.server.erase(name)
        self.server.applyChanges()
        self.markers.remove(name)
        self.list_pub.publish(self.markers)
        return ret

    def gotoMarker(self, name):
        if not name in self.markers: return False
        #while (self.LATEST_ODOM is None):
        #    rospy.sleep(2.0)
        # TODO: implement this
        # Get the latest orientation in radians
        '''pose = self.LATEST_ODOM.pose.pose
        LAST_RAD = self.base.q_to_yaw(pose.orientation) % (2 * math.pi)
        # get the latest position
        LAST_POS = pose.position
	    # get the markers orientation in radians
        MARK_POS = self.server.get(name).pose
        MARK_RAD = (self.base.q_to_yaw(MARK_POS.orientation)) % (2 * math.pi)
        MARK_x = MARK_POS.position.x
        MARK_y = MARK_POS.position.y
        MARK_z = MARK_POS.position.z
        self.base.turn(LAST_RAD * -1)
        dist = math.sqrt(((MARK_x - LAST_POS.x) ** 2)+((MARK_y - LAST_POS.y) ** 2) +  ((MARK_z - LAST_POS.z) ** 2))
        self.base.go_forward(MARK_x - LAST_POS.x)
        self.base.turn(math.pi / 2)
        self.base.go_forward(MARK_y - LAST_POS.y)'''
        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = "map"
        poseStamped = PoseStamped()
        poseStamped.header = h
        poseStamped.pose = self.server.get(name).pose
        self.move_pub.publish(poseStamped)
        return True

    def renameMarker(self, oldname, newname):
        if not oldname in self.markers: return False

        # Get marker obj from server
        marker = self.server.get(oldname)

        # Delete marker
        self.deleteMarker(oldname)

        # Add new marker
        marker.name = newname
        marker.controls[0].markers[1].text = newname
        self.addMarker(marker)

        return True

if __name__=="__main__":
    rospy.init_node("map_annotator")

    markerSavePath = "saved_markers.db"

    # Start Marker Service / Server
    wait_for_time()
    markersServer = MarkersServer()
    markersServer.load_markers_from_file(markerSavePath)
    manage_markers_service = rospy.Service('map_annotator/manage_marker', ManageMarker,
                                  markersServer.handle_manage_marker)

    rospy.spin()

    # Save markers to file
    markersServer.save_markers_to_file(markerSavePath)
