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
        serviceType : '/patient_monitor/reminders/AddReminder'
    });

    clients.addReminder = new ROSLIB.Service({
        ros : ros,
        name : '/patient_monitor/reminders/remove_reminder',
        serviceType : '/patient_monitor/reminders/RemoveReminder'
    });

    clients.reminderList.subscribe(function(message) {
        content = ""
        message.reminders.forEach(function(reminder) {
            content += "<tr>"
                content += "<th>" + get_message_by_id(reminder.message_id) + "</th>"
                content += "<th></th>"
                content += "<th></th>"
                content +="<th><button onClick=\"reminderPWDOf(" + reminder.id + ")\">Remind Now</th>"
                content +="<th><button onClick=\"deleteReminder(" + reminder.id + ")\">Delete</th>"
            content += "</tr>"
            content += name + "</br>"
        })

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
        message_id: name,
        hours: [hour],
    }),
    function(result) {
        console.log(result)
    });
}

var createReminderUI = function(evt) {
    createReminder(event.target[0].value, event.target[1].value)
    event.target[0].value = ""
    event.target[1].value = ""
}

var deleteReminder = function(name) {
    clients.manageReminder.callService(
    new ROSLIB.ServiceRequest({
        cmd : "delete",
        reminderName: name,
    }),
    function(result) {
        console.log(result)
    });
}

var deleteReminderUI = function(evt) {
    deleteReminder(event.target[0].value)
    event.target[0].value = ""
}

var remindPWDOf = function(name) {
    clients.manageReminder.callService(
    new ROSLIB.ServiceRequest({
        cmd : "remind",
        reminderName: name,
    }),
    function(result) {
        console.log(result)
    });
}

var remindPWDOfUI = function(evt) {
    remindUserOf(event.target[0].value)
    event.target[0].value = ""
}