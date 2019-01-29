document.addEventListener("DOMContentLoaded", function(event) {
    var viewer = new ROS3D.Viewer({
        divID : 'markers',
        width : document.body.clientWidth * .4,
        height : document.body.clientWidth * .4,
        antialias : true
    });
    
    var tfClient = new ROSLIB.TFClient({
        ros : ros,
        angularThres : 0.01,
        transThres : 0.01,
        rate : 10.0,
        fixedFrame : '/rotating_frame'
    });
    
    var imClient = new ROS3D.InteractiveMarkerClient({
        ros : ros,
        tfClient : tfClient,
        topic : '/basic_controls',
        camera : viewer.camera,
        rootObject : viewer.selectableObjects
    });
 });