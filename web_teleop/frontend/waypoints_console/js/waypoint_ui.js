clients = {}

document.addEventListener("DOMContentLoaded", function(event) {
    clients.listMarkers = new ROSLIB.Service({
        ros : ros,
        name : '/map_annotator/list_markers',
        serviceType : '/map_annotator/list_markers'
    });
    
    clients.manageMarker = new ROSLIB.Service({
        ros : ros,
        name : '/map_annotator/manage_marker',
        serviceType : '/map_annotator/manage_marker'
    });
})

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