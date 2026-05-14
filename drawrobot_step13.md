# DrawRobot Step 13: Web Dashboard

> **Interactive HTML/CSS/JavaScript frontend for robot control and visualization**

---

## 📋 Objectives

- ✅ Real-time robot position visualization
- ✅ Virtual joystick for motion control
- ✅ Shape drawing buttons (circle, triangle, etc.)
- ✅ Text input and rendering
- ✅ Live telemetry display (battery, sensors)
- ✅ Path history and ghost path visualization

---

## 📝 File Structure

```
data/
├── index.html      # Main dashboard page
├── style.css       # Styling and layout
└── script.js       # Interactivity and WebSocket
```

### index.html Structure

```html
<!DOCTYPE html>
<html>
<head>
    <title>DrawRobot Dashboard</title>
    <link rel="stylesheet" href="style.css">
</head>
<body>
    <div id="container">
        <!-- Canvas for map visualization -->
        <canvas id="mapCanvas" width="600" height="600"></canvas>

        <!-- Control panel -->
        <div id="controls">
            <!-- Virtual joystick -->
            <div id="joystick"></div>

            <!-- Sequence 1: Stairs & Squares -->
            <fieldset>
                <legend>Sequence 1: Stairs</legend>
                <button onclick="sendCmd('stairs')">Draw Stairs</button>
                <input id="sqSize" type="number" placeholder="Start Size">
                <input id="sqCount" type="number" placeholder="Count">
                <button onclick="drawSquares()">Draw Squares (FA1)</button>
            </fieldset>

            <!-- Sequence 2 & 3 -->
            <fieldset>
                <legend>Sequence 2 & 3</legend>
                <button onclick="drawCircle()">Circle (ET2)</button>
                <button onclick="sendCmd('compass', {full: true})">Compass Rose (ET3)</button>
            </fieldset>

            <!-- Text input -->
            <input id="textInput" type="text" placeholder="Text to write">
            <button onclick="drawText()">Write Text</button>

            <!-- Telemetry display -->
            <div id="telemetry">
                <p>Position: <span id="pos">(0, 0)</span></p>
                <p>Heading: <span id="heading">0°</span></p>
                <p>Battery: <span id="battery">100%</span></p>
            </div>
        </div>
    </div>

    <script src="script.js"></script>
</body>
</html>
```

### script.js Outline

```javascript
// WebSocket connection
let ws = new WebSocket('ws://' + window.location.host);

// Robot state
let robotPose = {x: 0, y: 0, theta: 0};
let pathHistory = [];
let ghostPath = [];

ws.onmessage = (event) => {
    let data = JSON.parse(event.data);
    
    // Update robot position
    robotPose = data.pose;
    
    // Add to path history
    pathHistory.push(data.pose);
    
    // Update telemetry
    document.getElementById('pos').textContent = 
        `(${data.pose.x}, ${data.pose.y})`;
    document.getElementById('battery').textContent = 
        `${data.battery.percentage}%`;
    
    // Redraw canvas
    drawMap();
};

// Canvas drawing
function drawMap() {
    let canvas = document.getElementById('mapCanvas');
    let ctx = canvas.getContext('2d');
    
    // Clear canvas
    ctx.fillStyle = 'white';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    
    // Draw path history (light blue)
    ctx.strokeStyle = '#aaaaff';
    ctx.beginPath();
    if (pathHistory.length > 0) {
        ctx.moveTo(pathHistory[0].x, pathHistory[0].y);
        for (let p of pathHistory) {
            ctx.lineTo(p.x, p.y);
        }
        ctx.stroke();
    }
    
    // Draw robot position (red circle)
    ctx.fillStyle = 'red';
    ctx.beginPath();
    ctx.arc(robotPose.x, robotPose.y, 10, 0, 2*Math.PI);
    ctx.fill();
    
    // Draw heading direction
    let headingX = robotPose.x + 20 * Math.cos(robotPose.theta);
    let headingY = robotPose.y + 20 * Math.sin(robotPose.theta);
    ctx.strokeStyle = 'red';
    ctx.beginPath();
    ctx.moveTo(robotPose.x, robotPose.y);
    ctx.lineTo(headingX, headingY);
    ctx.stroke();
}

// Drawing commands
function drawCircle() {
    ws.send(JSON.stringify({
        cmd: 'draw',
        shape: 'circle',
        radius: 50
    }));
}

function drawTriangle() {
    ws.send(JSON.stringify({
        cmd: 'draw',
        shape: 'triangle'
    }));
}

function drawText() {
    let text = document.getElementById('textInput').value;
    ws.send(JSON.stringify({
        cmd: 'draw',
        shape: 'text',
        text: text
    }));
}
```

---

## ✅ Verification

- [ ] Dashboard loads at http://192.168.4.1
- [ ] Canvas shows robot position in real-time
- [ ] Buttons send commands
- [ ] Path draws as robot moves
- [ ] Telemetry updates (battery, heading)

---

## 🔄 Next Steps

1. **Step 14**: Full integration testing
2. Optimization: Reduce latency, smooth animations

---

## 💡 Dashboard Features

- **Map Visualization**: Shows real-time robot location and path
- **Virtual Joystick**: Touch/mouse control for motion
- **Shape Buttons**: Quick access to drawing commands
- **Telemetry**: Live battery, position, heading display
- **Responsive**: Works on desktop and mobile browsers

---

## 📊 Performance Notes

- WebSocket updates at 20 Hz (50ms)
- Canvas redraws every update
- Mobile: Consider reducing path history size if lag occurs
