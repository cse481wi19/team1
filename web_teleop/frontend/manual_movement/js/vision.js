//Subscribing to a Topic
//----------------------
// Like when publishing a topic, we first create a Topic object with details of the topic's name
// and message type. Note that we can call publish or subscribe on the same topic object.

var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/vision/results',
    messageType : 'vision_msgs/FrameResults'
});

listener.subscribe(function(message) {
    console.log('Received message on ' + listener.name);
    console.log(message)
    // If desired, we can unsubscribe from the topic as well.
    //listener.unsubscribe();
});
// Calling a service
// -----------------
// First, we create a Service client with details of the service's name and service type.
/*var addTwoIntsClient = new ROSLIB.Service({
ros : ros,
name : '/add_two_ints',
serviceType : 'rospy_tutorials/AddTwoInts'
});
// Then we create a Service Request. The object we pass in to ROSLIB.ServiceRequest matches the
// fields defined in the rospy_tutorials AddTwoInts.srv file.
var request = new ROSLIB.ServiceRequest({
a : 1,
b : 2
});
// Finally, we call the /add_two_ints service and get back the results in the callback. The result
// is a ROSLIB.ServiceResponse object.
addTwoIntsClient.callService(request, function(result) {
console.log('Result for service call on ' + addTwoIntsClient.name + ': ' + result.sum);
});
// Advertising a Service
// ---------------------
// The Service object does double duty for both calling and advertising services
var setBoolServer = new ROSLIB.Service({
ros : ros,
name : '/set_bool',
serviceType : 'std_srvs/SetBool'
});
// Use the advertise() method to indicate that we want to provide this service
setBoolServer.advertise(function(request, response) {
console.log('Received service request on ' + setBoolServer.name + ': ' + request.data);
response['success'] = true;
response['message'] = 'Set successfully';
return true;
});
// Setting a param value
// ---------------------
ros.getParams(function(params) {
console.log(params);
});
// First, we create a Param object with the name of the param.
var maxVelX = new ROSLIB.Param({
ros : ros,
name : 'max_vel_y'
});
//Then we set the value of the param, which is sent to the ROS Parameter Server.
maxVelX.set(0.8);
maxVelX.get(function(value) {
console.log('MAX VAL: ' + value);
});
// Getting a param value
// ---------------------
var favoriteColor = new ROSLIB.Param({
ros : ros,
name : 'favorite_color'
});
favoriteColor.set('red');
favoriteColor.get(function(value) {
console.log('My robot\'s favorite color is ' + value);
});*/