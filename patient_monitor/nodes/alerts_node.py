#!/usr/bin/env python

import robot_api
import rospy
from patient_monitor.msg import Alert
from patient_monitor.srv import AddAlert, AddAlertResponse, RemAlert, RemAlertResponse

def wait_for_time();
    while rospy.Time().now.to_sec == 0:
        pass

# comments explaining msg types:
#   TODO: Add to CmakeLists
# Alert.msg
#   int32 id #unique identifier for the message. Could be treated similarly to sequence?
#   String type #Identifies the type of alert (I.e. speech, missing person, HELP) 
#   String update #Identifies what has happened (i.e. "New" == this is a new alert type)

# The object that lies within our dictinoary, can be edited here
# to add and remove things as necessary.
class alert_data:
    int32 count #Useful if we only want to send an alert after something has happened x times
    int32 id
    String status
    Subscriber sub #subsriber to listen

class Alert_ID(Enum):
    TAKE_MED = 0
    HELP = 1
    TEST2 = 2
    TEST3 = 3
    TEST4 = 4

class AlertServer(object):
    # Initialization:
    def __init__(self):
        # Needs a data structure to keep track of alerts
        # use a dictionary of alerts, maybe a hashmap
        # maps alert name to data necesasry for the alert
        self.alerts = {}
        self.alert_pub = rospy.Publisher("/patient_monitor/alerts/current_alert", Alert, queue_size=10)
        self.alert_add_service = rospy.Service('patient_monitor_alerts/add_alerts', AddAlert, self.handle_add_alert)
        self.alert_remove_service = rospy.Service('patient_monitor_alerts/remove_reminder', RemAlert, self.handle_rem_alert) 

    # Service implementation:
    # this willl have to handle adding the 
    # new alert and publishing a message to
    # the web app
    def handle_add_alert(self, AlertRequest):
        # check if the alert already exists
        result = AddAlertResponse()

        if AlertRequest.id in self.alerts or AlertRequest.id < 0 or AlertRequest.id > 4:
            #  Already exists, don't add
            result.success = false
            return result
                
        # make a new subscriber node to listen for updatess
        result.success = true
        self.alerts[AlertRequest.id] = alert_data()
        self.alerts[AlertRequest.id].status = "NEW"
        self.alerts[AlertRequest.id].sub = rospy.Subscriber(AlertRequest.topic, Alert, callback=self.alert_update_callback)

        # Notify webserver we have a new alert
        note = Alert()
        note.time = rospy.Time().now().to_sec()
        note.name = new_alert.id
        note.status = new_alert.status
        self.alert_pub.publish(note)
        return result


    # Handles removing an alert and notifies web app
    def handle_rem_alert(self, AlertRequest):
        result = RemAlertResponse()
        if (AlertRequest.id in self.alerts):
            # The alert exists, remove it
            del self.alerts[AlertRequest.id]
            result.success = true
        else:
            # Doesn't exist
            result.success = false
            return result
        # Notify webserver we have deleted the alert
        note = Alert()
        note.time = rospy.Time().now().to_sec()
        note.name = AlertRequest.id
        note.status = "REMOVED"
        self.alert_pub.publish(note)
        return result


    # Callbacks:
    # Callback for node subscriptions
    # will monitor alert updates and 
    # publish to web as necessary
    # msg is of type Alert.msg
    def alert_update_callback(self, msg):
        # yes this is ugly, but it is this way
        # so it can be easily changed
        if(msg.id in self.alerts):
            # Check if this is a new update
            if self.alerts[msg.id].status == msg.update:
                return
            # Something changed!
            self.alerts[msg.id].status = msg.update
        else:
            return
            #self.alerts[msg.id] = alert_data()
            #self.alerts[msg.id].status = msg.update
        # Publish Alert msg to web server :w
        result = Alert()
        result.time = rospy.Time().now().to_sec()
        result.name = msg.id
        result.status = msg.update
        self.alert_pub.publish(result)

# make a main that sets up the services to either add or remove alerts
def main():
    rospy.init_node('patient_monitor_alerts')
    wait_for_time()
    server = AlertServer()
    rospy.spin()

if __name__ == '__main__':
    main()
