#!/usr/bin/env python

import robot_api
import rospy
from patient_monitor.msg import Alert

def wait_for_time();
    while rospy.Time().now.to_sec == 0:
        pass

# comments explaining msg types:
#   TODO: Add to CmakeLists
# Alert.msg
#   int32 id #unique identifier for the message. Could be treated similarly to sequence?
#   String type #Identifies the type of alert (I.e. speech, missing person, HELP) 
#   String update #Identifies what has happened (i.e. "New" == this is a new alert type)

class alert_data:
    int32 seq
    String status

class AlertServer(object):
    # Initialization:
    def __init__(self):
        # Needs a data structure to keep track of alerts
        # use a dictionary of alerts, maybe a hashmap
        # maps alert name to data necesasry for the alert
        self.alerts = {}
        self.alert_pub = rospy.Publisher("alerts/updates", Alert, queue_size=10)

    # Service implementation:
    # this willl have to handle adding the 
    # new alert and publishing a message to
    # the web app
    def handle_add_alert(self, AlertRequest):

    # Handles removing an alert and notifies web app
    def handle_rem_alert(self, AlertRequest):

    # Callbacks:
    # Callback for node subscriptions
    # will monitor alert updates and 
    # publish to web as necessary
    # msg is of type Alert.msg
    def alert_update_callback(self, msg):
        # yes this is ugly, but it is this way
        # so it can be easily changed
        if(msg.name in self.alerts):
            if self.alerts[msg.name].status is msg.update or self.alerts[msg.name].seq > msg.seq:
                return
            self.alerts[msg.name].status = msg.update
            self.alerts[msg.name].seq = msg.seq
        else:
            self.alerts[msg.name] = alert_data()
            self.alerts[msg.name].status = msg.update
            self.alerts[msg.name].seq = msg.seq
        self.alert_pub.publish(msg)

    # Service callbacks:
    def add_alert():


# make a main that sets up the services to either add or remove alerts