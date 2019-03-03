clients = {}

document.addEventListener("DOMContentLoaded", function(event) {
    clients.reminderList.subscribe(function(message) {
        content = "Current Reminders: </br>"
        message.reminders.forEach(function(name) {
            content += name + "</br>"
        })

        document.getElementById("reminderList").innerHTML = content
    })
})

var createReminder = function(name, message) {
    clients.manageReminder.callService(
    new ROSLIB.ServiceRequest({
        cmd : "create",
        reminderName: name,
        reminderMessage: message,
    }),
    function(result) {
        console.log(result)
    });
}

var createReminderUI = function(evt) {
    createReminder(event.target[0].value, event.target[1].value, event.target[2].value = "")
    event.target[0].value = ""
    event.target[1].value = ""
    event.target[12].value = ""

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

var remindUserOf = function(name) {
    clients.manageReminder.callService(
    new ROSLIB.ServiceRequest({
        cmd : "remind",
        reminderName: name,
    }),
    function(result) {
        console.log(result)
    });
}

var remindUserOfUI = function(evt) {
    remindUserOf(event.target[0].value)
    event.target[0].value = ""
}