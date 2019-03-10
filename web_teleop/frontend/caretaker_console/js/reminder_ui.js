clients = {}

document.addEventListener("DOMContentLoaded", function(event) {
    clients.reminderList = new ROSLIB.Topic({
        ros : ros,
        name : '/patient_monitor/reminders/reminder_list',
        messageType : 'patient_monitor/Reminders'
    });
    
    clients.addReminder = new ROSLIB.Service({
        ros : ros,
        name : '/patient_monitor/reminders/add_reminder',
        serviceType : '/patient_monitor/reminders/add_reminder'
    });

    clients.removeReminder = new ROSLIB.Service({
       ros : ros,
       name : '/patient_monitor/reminders/remove_reminder',
       serviceType : '/patient_monitor/reminders/remove_reminder'
    });

    clients.fireReminder = new ROSLIB.Service({
        ros : ros,
        name : '/patient_monitor/reminders/fire_reminder',
        serviceType : '/patient_monitor/reminders/fire_reminder'
     });

    clients.reminderList.subscribe(function(message) {
        content = "<table><tr><th>Patient</th><th>Message</th><th>Time(s) of Day</th><th></th><th></th></tr>"
        if (message.reminders.length == 0) {
            content += "<tr><th>Nothing to see here!</th><th></th><th></th><th></th><th></th></tr>"
        } else {
            message.reminders.forEach(function(reminder) {
                content += "<tr>"
                    content += "<th>Patient 2085</th>"
                    content += "<th>" + get_message_by_id(reminder.message_id) + "</th>"
                    content += "<th>" + reminder.hours + "</th>"
                    content +="<th><button onClick=\"remindPWDOf(" + reminder.id + ")\">Remind Now</th>"
                    content +="<th><button onClick=\"deleteReminder(" + reminder.id + ")\">Delete</th>"
                content += "</tr>"
            })
        }
        content += "</table>"
        document.getElementById("reminderList").innerHTML = content
    })
})

var get_message_by_id = function(id){
    if (id == 0) {
        return "Take your medicine!"
    } else if (id == 1) {
        return "Go to Bingo!"
    } else if (id == 2) {
        return "Talk to someone!"
    } else if (id == 3) {
        return "Time to eat!"
    } else if (id == 4) {
        return "Remind PWD who they are!"
    }
    return "Unknown Message :("
}


var createReminder = function(message_id, hour) {
    clients.addReminder.callService(
        new ROSLIB.ServiceRequest({
            message_id: message_id,
            hours: [hour],
        }),
    function(result) {
        console.log(result)
    });
}

var createReminderUI = function(evt) {
    createReminder(parseInt(event.target[0].value), parseInt(event.target[1].value))
    event.target[0].value = ""
    event.target[1].value = ""
}

var deleteReminder = function(id) {
    clients.removeReminder.callService(
    new ROSLIB.ServiceRequest({
        id: id,
    }),
    function(result) {
        console.log(result)
    });
}

var deleteReminderUI = function(evt) {
    deleteReminder(event.target[0].value)
    event.target[0].value = ""
}

var remindPWDOf = function(id) {
    clients.fireReminder.callService(
    new ROSLIB.ServiceRequest({
        id: id,
    }),
    function(result) {
        console.log(result)
    });
}

var remindPWDOfUI = function(evt) {
    remindPWDOf(event.target[0].value)
    event.target[0].value = ""
}