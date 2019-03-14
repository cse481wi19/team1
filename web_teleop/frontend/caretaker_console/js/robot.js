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
//ros.connect('ws://panang.hcrlab.cs.washington.edu:11311');

document.addEventListener("DOMContentLoaded", function(event) {
    // Setup here
});