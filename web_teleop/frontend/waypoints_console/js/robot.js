// Connecting to ROS
// -----------------
var scene = {}
var ros = new ROSLIB.Ros();

var movementCommand = null

// If there is an error on the backend, an 'error' emit will be emitted.
ros.on('error', function(error) {
    document.getElementById('connecting').style.display = 'none';
    document.getElementById('connected').style.display = 'none';
    document.getElementById('closed').style.display = 'none';
    document.getElementById('error').style.display = 'inline';
    console.log(error);
});

// Find out exactly when we made a connection.
ros.on('connection', function() {
    console.log('Connection made!');
});

ros.on('close', function() {
    console.log('Connection closed.');
});

// Create a connection to the rosbridge WebSocket server.
ros.connect('ws://shimada.cs.washington.edu:9090');

document.addEventListener("DOMContentLoaded", function(event) {
    // Scene
    scene.viewer = new ROS3D.Viewer({
        divID : 'markers',
        width : document.body.clientWidth * .4,
        height : document.body.clientWidth * .4,
        antialias : true,
        background : "#f0f0ee"
    });

    // Grid
    scene.viewer.addObject(new ROS3D.Grid());
    
    // TF Server
    scene.tfClient = new ROSLIB.TFClient({
        ros : ros,
        angularThres : 0.01,
        transThres : 0.01,
        rate : 10.0,
        fixedFrame : '/odom'
    });

    // Robot Model
    scene.urdfClient = new ROS3D.UrdfClient({
       ros : ros,
       tfClient : scene.tfClient,
       path : 'static/',
       rootObject : scene.viewer.scene,
     });

     setupWaypoints()
 });

 setupWaypoints = function() {
     
 }