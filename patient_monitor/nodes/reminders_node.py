#!/usr/bin/env python

import robot_api
import rospy
import pickle
import os.path
from enum import Enum
from web_teleop.srv.reminders import AddReminder, AddReminderResponse, RemoveReminder, RemoveReminderResponse

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

''' SERVICE DEFINITION '''
class ReminderMessage:
    MEDICINE = 0
    BINGO = 1
    SOCIAL_INTERACTION = 2
    EAT = 3
    WHO_AM_I = 4

class Response(Enum):
    SUCCESS = 0
    UNKNOWN_ERROR = 1
    UNKNOWN_ID = 2
    INVALID_PARAMS = 3

''' REMINDER DEFINITION'''
class Reminder(object):
    def __init__(self, id, message, times):
        if not id or type(id) is not int:
            raise ValueError("The ID must be an int!")
        if not message or type(message) is not ReminderMessage:
            raise ValueError("Message must be a reminder message!")
        if not times or type(times) is not list or len(times) < 1:
            raise ValueError("Times must be a list!")
        if len(times) < 1 or type(times[0]) is not type(rospy.Time().now()):
            raise ValueError("Times must be a list of rospy times!")
        self.id = id
        self.message = message
        self.times = times

class RemindersServer(object):
    ## Initialization
    def __init__(self, path=None):
        # Init Server
        self.reminders = []
        if path: self.load_reminders_from_file(path)

    def load_reminders_from_file(self, path):
        if not os.path.isfile(path): return False
        file_in = open(path, "rb")
        self.reminders = pickle.load(file_in)
        return True

    def save_reminders_to_file(self, path):
        file_out = open(path, "wb")
        pickle.dump(self.reminders, file_out)
        file_out.close()

    ## Services
    def handle_add_reminder(self, request):     
        response = AddReminderResponse()   
        response.status = Response.SUCCESS
        message = None
        try:
            message = ReminderMessage(request.message)
        except:
            response.status = Response.INVALID_MESSAGE
        if len(request.times) is 0:
            response.status = Response.INVALID_PARAMS
        if response.status is Response.SUCCESS:
            res = self.addReminder(message, request.times)
            if not res:
              response.status = Response.UNKNOWN_ERROR  
        return response

    def handle_remove_reminder(self, request):
        response = RemoveReminderResponse()
        response.status = Response.SUCCESS
        if self.removeReminder(request.id) is False:
            response.status = Response.UNKNOWN_ID
        return response

    ## Reminder Support Code
    def addReminder(self, message, times):
        id = 0
        if len(self.reminders) > 0:
            id = self.reminders[-1].id + 1
        try:            
            self.reminders.insert(Reminder(id, message, times()))
        except ValueError:
            return False
        return True

    def removeReminder(self, id):
        reminders = [reminder for reminder in self.reminders if reminder.id is not id]
        if len(reminders) is len(self.reminders):
            return False
        self.reminders = reminders
        return True

def main():
    rospy.init_node('patient_monitor_reminders')
    wait_for_time()
    server = RemindersServer()
    reminder_add_service = rospy.Service('patient_monitor_reminders/add_reminder', AddReminder,
                                  server.handle_add_reminder)
    reminder_remove_service = rospy.Service('patient_monitor_reminders/remove_reminder', RemoveReminder,
                                  server.handle_remove_reminder)
    rospy.spin()

if __name__ == '__main__':
    main()
