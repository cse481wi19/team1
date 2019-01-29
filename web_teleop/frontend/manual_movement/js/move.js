var cmdVel = new ROSLIB.Topic({
    ros : ros,
    name : '/mobile_base/commands/velocity',
    messageType : 'geometry_msgs/Twist'
});

var movementCommand = null
var pubishMove = function(linear, angular) {
    if (movementCommand) clearInterval(movementCommand)
    movementCommand = setInterval(function() {
        cmdVel.publish(
            new ROSLIB.Message({
                linear : linear,
                angular : angular
            })
        )
        console.log('move published')
    });
}

var endMove = function(evt) {
    if (movementCommand) clearInterval(movementCommand) 
}

var startForward = function(evt) {
    pubishMove(
          {
            x: 0.2,
            y: 0,
            z: 0
          },
          {
            x: 0,
            y: 0,
            z: 0
          }
      )
      console.log("Forward Button Pressed")
}

var startLeft = function(evt) {
    pubishMove(
        {
          x: 0,
          y: 0,
          z: 0
        },
        {
          x: 0,
          y: 0,
          z: 0.5
        }
    )
    console.log("Left Button Pressed")
}

var startRight = function(evt) {
    pubishMove(
        {
          x: 0,
          y: 0,
          z: 0
        },
        {
          x: 0,
          y: 0,
          z: -0.5
        }
    )
    console.log("Left Button Pressed")
}

var startBackward = function(evt) {
    pubishMove(
          {
            x: -0.2,
            y: 0,
            z: 0
          },
          {
            x: 0,
            y: 0,
            z: 0
          }
      )
      console.log("Forward Button Pressed")
}