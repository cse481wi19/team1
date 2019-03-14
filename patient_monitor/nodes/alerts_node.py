#!/usr/bin/env python


import robot_api
import rospy
import time
from patient_monitor.msg import Alert, Alerts
from patient_monitor.srv import AddAlert, AddAlertResponse, RemAlert, RemAlertResponse
from enum import Enum

def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass

# comments explaining msg types:
#   TODO: Add to CmakeLists
# Alert.msg
#   int32 id #unique identifier for the message. Could be treated similarly to sequence?
#   String type #Identifies the type of alert (I.e. speech, missing person, HELP) 
#   String update #Identifies what has happened (i.e. "New" == this is a new alert type)

# The object that lies within our dictinoary, can be edited here
# to add and remove things as necessary.
class alert_data():
    def __init__(self, subscriber):
        self.times = []
        self.id = 0
        self.status = ""
        self.sub = subscriber # subsriber to listen

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
        self.alert_list_pub = rospy.Publisher("/patient_monitor/alerts/alert_list", Alerts, queue_size=10, latch=True)
        self.alert_add_service = rospy.Service('patient_monitor/alerts/add_alert', AddAlert, self.handle_add_alert)
        self.alert_remove_service = rospy.Service('patient_monitor/alerts/remove_alert', RemAlert, self.handle_rem_alert) 
        
        

    # Service implementation:
    # this willl have to handle adding the 
    # new alert and publishing a message to
    # the web app
    def handle_add_alert(self, AlertRequest):
        # check if the alert already exists
        result = AddAlertResponse()

        if AlertRequest.id in self.alerts or AlertRequest.id < 0 or AlertRequest.id > 4:
            #  Already exists, don't add
            result.success = False
            return result

        add_time = time.ctime(rospy.Time().now().to_sec())
                
        # make a new subscriber node to listen for updatess
        result.success = True
        self.alerts[AlertRequest.id] = alert_data(rospy.Subscriber(AlertRequest.topic, Alert, callback=self.alert_update_callback))
        self.alerts[AlertRequest.id].status = "NEW"
        self.alerts[AlertRequest.id].id = AlertRequest.id
        self.alerts[AlertRequest.id].times.append(time.ctime(rospy.Time().now().to_sec()))

        # Notify webserver we have a new alert
        note = Alert()
        note.time.append(add_time)
        note.id = AlertRequest.id
        note.update = "NEW"
        self.alert_pub.publish(note)
        self.publish_alerts()
        return result


    # Handles removing an alert and notifies web app
    def handle_rem_alert(self, AlertRequest):
        result = RemAlertResponse()
        if (AlertRequest.id in self.alerts):
            # The alert exists, remove it
            self.alerts[AlertRequest.id].times = []
            self.alerts[AlertRequest.id].status = "Cleared"
            result.success = True
        else:
            # Doesn't exist
            result.success = False
            return result
        # Notify webserver we have deleted the alert
        #note = Alert()
        #note.time = rospy.Time().now().to_sec()
        #note.name = AlertRequest.id
        #note.status = "Cleared"
        #self.alert_pub.publish(note)
        self.publish_alerts()
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
            self.alerts[msg.id].times.append(time.ctime(rospy.Time().now().to_sec()))
            # Something changed!
            self.alerts[msg.id].status = msg.update
        else:
            return
            #self.alerts[msg.id] = alert_data()
            #self.alerts[msg.id].status = msg.update
        # Publish Alert msg to web server :w
        #result = Alert()
        #result.time = rospy.Time().now().to_sec()
        #result.name = msg.id
        #result.status = msg.update
        #self.alert_pub.publish(result)
        self.publish_alerts()

    def publish_alerts(self):
        result = Alerts()
        for key, val in self.alerts.items():
            new_alert = Alert()
            new_alert.time = val.times
            new_alert.id = val.id
            new_alert.update = val.status
            result.alerts.append(new_alert)
        self.alert_list_pub.publish(result)

# make a main that sets up the services to either add or remove alerts
def main():
    rospy.loginfo("main")
    rospy.init_node('patient_monitor_alerts')
    wait_for_time()
    server = AlertServer()
    
    #testing
    rospy.wait_for_service('patient_monitor/alerts/add_alert')
    request = AddAlert()
    request.topic = 'patient_monitor/alerts'
    request.id = 1
    server.handle_add_alert(request)
    da_pub = rospy.Publisher('patient_monitor/alerts', Alert, queue_size=10)
    da_pub.publish(Alert([],1,"Damn boi he thicc"))

    rospy.spin()

if __name__ == '__main__':
    rospy.loginfo("main")
    main()