const socket = new WebSocket('ws://localhost:8080'); // WebSocket server address

// Variables to keep track of key press state
let wKeyPressed = false;
let sKeyPressed = false;

// Function to send motor commands based on key presses
function sendMotorCommands() {
    // Calculate motor speed based on key presses
    let motorSpeed = 0;
    if (wKeyPressed) {
        motorSpeed += 1; // Increase motor speed when "W" is pressed
    }
    if (sKeyPressed) {
        motorSpeed -= 1; // Decrease motor speed when "S" is pressed
    }

    // Send the motor speed command to the Python WebSocket server
    socket.send(`MOTOR ${motorSpeed}`);
}

// Event listeners for key press and release (similar to the previous code)
document.addEventListener('keydown', (event) => {
    if (event.key === 'w' || event.key === 'W') {
        wKeyPressed = true;
        sendMotorCommands();
    } else if (event.key === 's' || event.key === 'S') {
        sKeyPressed = true;
        sendMotorCommands();
    }
});

document.addEventListener('keyup', (event) => {
    if (event.key === 'w' || event.key === 'W') {
        wKeyPressed = false;
        sendMotorCommands();
    } else if (event.key === 's' || event.key === 'S') {
        sKeyPressed = false;
        sendMotorCommands();
    }
});
