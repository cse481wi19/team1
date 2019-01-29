eyelidsClient = new ROSLIB.Service({
  ros : ros,
  name : '/web_teleop/set_eyelids',
  serviceType : 'web_teleop/setEyelids'
});

var publishEyelids = function(radians) {
  eyelidsClient.callService(
    new ROSLIB.ServiceRequest({
      radians : radians,
    }),
    function(result) {
      console.log(result)
    }
  );
}

var updateEyelids = function(evt) {
  publishEyelids((event.target.value / 100) * .57 - .16)
}