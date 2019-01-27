var cmdChest = new ROSLIB.Topic({
    ros : ros,
    name : '/mobile_base/commands/chest_leds',
    messageType : 'mobile_base_driver/ChestLeds'
});

var pubishChest = function(r, g, b) {
    //if (movementCommand) clearInterval(movementCommand)
    //movementCommand = setInterval(function() {
        cmdChest.publish(
            new ROSLIB.Message({
                leds:
                    [
                        createLED(r, g, b),
                        createLED(r, g, b),
                        createLED(r, g, b),
                        createLED(r, g, b),
                        createLED(r, g, b),
                        createLED(r, g, b),
                        createLED(r, g, b),
                        createLED(r, g, b),
                        createLED(r, g, b),
                        createLED(r, g, b),
                        createLED(r, g, b),
                        createLED(r, g, b),
                        createLED(r, g, b),
                        createLED(r, g, b),
                        createLED(r, g, b)
                    ]
                }
            )
        )
        console.log('move published')
    //});
}


var createLED = function(r, g, b) {
    return new ROSLIB.Message(
        {red: r, green: g, blue: b}
    )
}

var setChestColor = function(evt) {
    rgb = hexToRgb(event.target.value)
    pubishChest(rgb.r, rgb.g, rgb.b)
}

function hexToRgb(hex) {
    var result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
    return result ? {
        r: parseInt(result[1], 16),
        g: parseInt(result[2], 16),
        b: parseInt(result[3], 16)
    } : null;
}
