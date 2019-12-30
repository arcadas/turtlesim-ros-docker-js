// Init HTML Canvas
var c = document.getElementById("turtle_canvas");
var ctx = c.getContext("2d");
var defaultX = 30;
var defaultY = 30;
var x = defaultX;
var y = defaultY;

// Draw function
function drawLine(x2, y2) {
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(x2, y2);
    ctx.stroke();
    x = x2;
    y = y2;
}
// Clear Canvas
function drawClear() {
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, c.width, c.height);
    ctx.restore();
    x = defaultX;
    y = defaultY;
}

// Init ROS client and subscriber
var ros = new ROSLIB.Ros({
    url: 'ws://arcadas.com:9094'
});
var listener = new ROSLIB.Topic({
    ros: ros,
    name: '/turtlesim_commands',
    messageType: 'std_msgs/String'
});

// Subscribe and listen
listener.subscribe(function(message) {
    console.log('Received command on ' + listener.name + ': ' + message.data);
    switch (message.data) {
        case 'w':
            drawLine(x, y - 50);
            break;
        case 's':
            drawLine(x, y + 50);
            break;
        case 'a':
            drawLine(x - 50, y);
            break;
        case 'd':
            drawLine(x + 50, y);
            break;
        case 'r':
            drawClear();
    }
    // Close socket
    // listener.unsubscribe();
});
