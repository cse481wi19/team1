clients = {}

document.addEventListener("DOMContentLoaded", function(event) {
    clients.alert = new ROSLIB.Topic({
        ros : ros,
        name : '/patient_monitor/alerts/current_alert',
        messageType : 'patient_monitor/Alert'
    });

    clients.alert.subscribe(function(message) {
        alert("ALERT: " + get_alert_by_id(message.id))
    })
})

var get_alert_by_id = function(id){
    if (id == 0) {
        return "Patient will not take medicine!"
    } else if (id == 1) {
        return "Pateint needs help!"
    } else if (id == 2) {
        return "Test 2"
    } else if (id == 3) {
        return "Test 3"
    } else if (id == 4) {
        return "Test 4"
    }
    return "Unknown Alert :("
}