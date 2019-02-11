clients = {}

document.addEventListener("DOMContentLoaded", function(event) {
    clients.markerList = new ROSLIB.Topic({
        ros : ros,
        name : '/map_annotator/marker_list',
        messageType : 'map_annotator/Markers'
    });
    
    clients.manageMarker = new ROSLIB.Service({
        ros : ros,
        name : '/map_annotator/manage_marker',
        serviceType : '/map_annotator/manage_marker'
    });

    clients.markerList.subscribe(function(message) {
        content = "Current Markers: </br>"
        message.markers.forEach(function(name) {
            content += name + "</br>"
        })

        document.getElementById("markerList").innerHTML = content
    })
})

var createMarker = function(name) {
    clients.manageMarker.callService(
    new ROSLIB.ServiceRequest({
        cmd : "create",
        markerName: name,
        newMarkerName: ""
    }),
    function(result) {
        console.log(result)
    });
}

var createMarkerUI = function(evt) {
    createMarker(event.target[0].value)
    event.target[0].value = ""
}

var deleteMarker = function(name) {
    clients.manageMarker.callService(
    new ROSLIB.ServiceRequest({
        cmd : "delete",
        markerName: name,
        newMarkerName: ""
    }),
    function(result) {
        console.log(result)
    });
}

var deleteMarkerUI = function(evt) {
    deleteMarker(event.target[0].value)
    event.target[0].value = ""
}

var gotoMarker = function(name) {
    clients.manageMarker.callService(
    new ROSLIB.ServiceRequest({
        cmd : "goto",
        markerName: name,
        newMarkerName: ""
    }),
    function(result) {
        console.log(result)
    });
}

var gotoMarkerUI = function(evt) {
    gotoMarker(event.target[0].value)
    event.target[0].value = ""
}

var renameMarker = function(name, newName) {
    clients.manageMarker.callService(
    new ROSLIB.ServiceRequest({
        cmd : "rename",
        markerName: name,
        newMarkerName: newName
    }),
    function(result) {
        console.log(result)
    });
}

var renameMarkerUI = function(evt) {
    renameMarker(event.target[0].value, event.target[1].value)
    event.target[0].value = ""
    event.target[1].value = ""
}

var lookatMarker = function(name) {
    clients.manageMarker.callService(
    new ROSLIB.ServiceRequest({
        cmd : "look_at",
        markerName: name,
        newMarkerName: ""
    }),
    function(result) {
        console.log(result)
    });
}

var lookatMarkerUI = function(evt) {
    lookatMarker(event.target[0].value)
    event.target[0].value = ""
}