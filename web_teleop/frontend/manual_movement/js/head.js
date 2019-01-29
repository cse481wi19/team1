headClient = new ROSLIB.Service({
    ros : ros,
    name : '/web_teleop/set_head',
    serviceType : 'web_teleop/setHead'
});

currentPan = 0
currentTilt = 0

var publishHead = function(pan, tilt) {
    headClient.callService(
        new ROSLIB.ServiceRequest({
        pan : pan,
        tilt : tilt
        }),
        function(result) {
        console.log(result)
        }
    );
}

var updateHeadPan = function(evt) {
    currentPan = (1 - event.target.value / 100) * 1.56 - .78
    publishHead(currentPan, currentTilt)
}

var updateHeadTilt = function(evt) {
    currentTilt = ((1 - event.target.value / 100) * 1.21)  - 0.92
    publishHead(currentPan, currentTilt)
}