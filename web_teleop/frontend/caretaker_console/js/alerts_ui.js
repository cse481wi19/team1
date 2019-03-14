clients = {}

document.addEventListener("DOMContentLoaded", function(event) {
    clients.alert = new ROSLIB.Topic({
        ros : ros,
        name : '/patient_monitor/alerts/current_alert',
        messageType : 'patient_monitor/Alert'
    });
 
    clients.alertList = new ROSLIB.Topic({
        ros : ros,
        name : '/patient_monitor/alerts/alert_list',
        messageType : 'patient_monitor/Alerts'
    });

    clients.addAlert = new ROSLIB.Service({
        ros : ros,
        name : '/patient_monitor/alerts/add_alert',
        serviceType : '/patient_monitor/alerts/add_alert'
    });

    clients.removeAlert = new ROSLIB.Service({
       ros : ros,
       name : '/patient_monitor/alerts/remove_alert',
       serviceType : '/patient_monitor/alerts/remove_alert'
    });

    document.addEventListener("DOMContentLoaded", function(event) {
        clients.alert.subscribe(function(message) {
            alert("ALERT: " + get_alert_by_id(message.id))
        })
    })

    clients.alertList.subscribe(function(message) {
        console.log(message)
        content = "<table><tr><th>Patient</th><th>Status Type</th><th>Status</th><th>Time(s) Alerted</th><th></th></tr>"
        if (message.alerts.length == 0) {
            content += "<tr><th>Nothing to see here!</th><th></th><th></th><th></th><th></th></tr>"
        } else {
            message.alerts.forEach(function(alert) {
                content += "<tr>"
                    content += "<th>Patient 2085</th>"
                    content += "<th>" + get_alert_by_id(alert.id) + "</th>"
                    content += "<th>" + alert.update + "</th>"
                    content += "<th>" + alert.time + "</th>"
                    content +="<th><button onClick=\"deleteAlert(" + alert.id + ")\">Clear</th>"
                content += "</tr>"
            })
        }
        content += "</table>"
        document.getElementById("alertList").innerHTML = content
    })
})

var get_alert_by_id = function(id){
    if (id == 0) {
        return "Patient will not take medicine!"
    } else if (id == 1) {
        return "Patient needs help!"
    } else if (id == 2) {
        return "Test 2"
    } else if (id == 3) {
        return "Test 3"
    } else if (id == 4) {
        return "Test 4"
    }
    return "Unknown Alert :("
}

var deleteAlert = function(id) {
    clients.removeAlert.callService(
    new ROSLIB.ServiceRequest({
        id: id,
    }),
    function(result) {
        console.log(result)
    });
}

var deleteAlertUI = function(evt) {
    deleteAlert(event.target[0].value)
    event.target[0].value = ""
}