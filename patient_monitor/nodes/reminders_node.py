#!/usr/bin/env python

import robot_api
import rospy
import pickle
import os.path
from enum import Enum
from patient_monitor.srv import AddReminder, AddReminderResponse, RemoveReminder, RemoveReminderResponse
from patient_monitor.msg import Reminder, Reminders
from datetime import datetime, timedelta
from dateutil import tz
import time

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def hour_to_time(base_time, hour):
    return base_time + rospy.Duration(60 * 60 * hour)

def get_start_of_today():
    today = datetime.utcnow().date()
    start_of_today = datetime(today.year, today.month, today.day, tzinfo=tz.tzutc()).timetuple()
    start_of_today_time = time.mktime(start_of_today)
    return rospy.Time.from_sec(start_of_today_time)

def get_end_of_today():
    today = datetime.utcnow().date()
    end_of_today = datetime(today.year, today.month, today.day, tzinfo=tz.tzutc()) + timedelta(1)
    end_of_today_time = time.mktime(end_of_today.timetuple())
    return rospy.Time.from_sec(end_of_today_time)

''' SERVICE DEFINITION '''
class ReminderMessage(Enum):
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
    INVALID_MESSAGE = 4

''' REMINDER DEFINITION'''
class ReminderStore(object):
    def __init__(self, id, message, hours):
        if id is None or type(id) is not int:
            raise ValueError("The ID must be an int!")
        if message is None or type(message) is not ReminderMessage:
            raise ValueError("Message must be a reminder message!")
        if hours is None or len(hours) < 1:
            raise ValueError("hours must be a list!")
        self.id = id
        self.message = message
        self.hours = hours
        self.fired_today = [0] * len(hours)

class RemindersServer(object):
    ## Initialization
    def __init__(self, path=None):
        # Init Server
        self.reminders = []
        self.list_pub = rospy.Publisher('patient_monitor/reminders/reminder_list', Reminders, queue_size=10, latch=True)
        self.reminder_timer = rospy.Timer(rospy.Duration(2), self.reminder_loop_callback)
        self.reminder_reset_timer = rospy.Timer(rospy.Duration(60 * 60 * 24), self.start_of_the_day_callback)
        self.reminder_add_service = rospy.Service('patient_monitor/reminders/add_reminder', AddReminder, self.handle_add_reminder)
        self.reminder_remove_service = rospy.Service('patient_monitor/reminders/remove_reminder', RemoveReminder, self.handle_remove_reminder) 
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

    def publish_reminders(self):
        out = Reminders()
        out.reminders = []
        for x in self.reminders:
            r = Reminder()
            r.id = x.id
            r.message_id = x.message.value
            r.hours = x.hours
            out.reminders.append(r)
        self.list_pub.publish(out)

    def reminder_loop_callback(self, event):
        start_of_the_day = get_start_of_today()
        for r in self.reminders:
            for i in range(0, len(r.hours)):
                if rospy.Time().now() > hour_to_time(start_of_the_day, r.hours[i]) and r.fired_today[i] == 0:
                    r.fired_today[i] = 1
                    self.fireReminderWithObject(r)
    
    def start_of_the_day_callback(self, event):
        for r in self.reminders:
            r.fired_today = [0] * len(r.hours)

    ## Services
    def handle_add_reminder(self, request):     
        response = AddReminderResponse()   
        response.status = Response.SUCCESS
        message = None
        try:
            message = ReminderMessage(request.message_id)
        except:
                response.status = Response.INVALID_MESSAGE
        if len(request.hours) is 0:
            response.status = Response.INVALID_PARAMS
        if response.status is Response.SUCCESS:
            res = self.addReminder(message, request.hours)
            if not res:
              response.status = Response.UNKNOWN_ERROR
	    response.status = response.status.value  
        self.publish_reminders()
        return response

    def handle_remove_reminder(self, request):
        response = RemoveReminderResponse()
        response.status = Response.SUCCESS.value
        if self.removeReminder(request.id) is False:
            response.status = Response.UNKNOWN_ID.value
        self.publish_reminders()
        return response

    ## Reminder Support Code
    def addReminder(self, message, hours):
        identifier = 0
        if len(self.reminders) != 0:
            identifier = self.reminders[-1].id + 1
        # try:            
        self.reminders.append(ReminderStore(identifier, message, hours))
        #except ValueError:
        #    return False
        return True

    def removeReminder(self, id):
        reminders = [reminder for reminder in self.reminders if reminder.id is not id]
        if len(reminders) is len(self.reminders):
            return False
        self.reminders = reminders
        return True

    def fireReminder(self, id):
        reminder = None
        for r in Reminders:
            if r.id == id:
                reminder = r
                break
        self.fireReminderWithObject(reminder)

    def fireReminderWithObject(self, reminder):
        pass

def main():
    rospy.init_node('patient_monitor_reminders')
    wait_for_time()
    server = RemindersServer("reminders.store")
    rospy.spin()

if __name__ == '__main__':
    main()
