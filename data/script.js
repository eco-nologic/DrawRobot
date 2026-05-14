// DEFENSE: "Quel protocole est utilisé ici ?"
// ANSWER: WebSocket (ws://). C'est asynchrone et beaucoup plus rapide que le HTTP classique.
const ws = new WebSocket(`ws://${window.location.hostname}/ws`);
const canvas = document.getElementById('map');
const ctx = canvas.getContext('2d');

// Robot state for drawing
let robotX = 0, robotY = 0;
let pathHistory = [];

// Canvas setup
const MAP_SCALE = 0.5; // 1mm = 0.5 pixels
const CANVAS_CENTER_X = canvas.width / 2;
const CANVAS_CENTER_Y = canvas.height / 2;
/**
 * DEFENSE: "Comment avez-vous généré les graphiques du rapport (p.17) ?"
 * ANSWER: Le dashboard stocke les coordonnées reçues. Nous pouvons les exporter 
 * en format CSV pour comparer les distances théoriques et mesurées sous Excel/Python.
 */
/**
 * @brief Exporte l'historique des positions au format CSV.
 * DEFENSE: "Comment avez-vous généré les graphiques du rapport (p.17) ?"
 * ANSWER: Le dashboard stocke les coordonnées reçues. Nous les exportons en CSV pour comparer les distances théoriques et mesurées.
 */
function exportPath() {
    let csv = "x_mm,y_mm\n";
    pathHistory.forEach(p => csv += `${p.x},${p.y}\n`);
    const blob = new Blob([csv], { type: 'text/csv' });
    const url = window.URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'robot_path.csv';
    a.click();
}

/**
 * DEFENSE: "Comment gérez-vous le passage des coordonnées réelles au Canvas ?"
 * ANSWER: On utilise ctx.translate pour placer l'origine (0,0) au centre du canvas 
 * et ctx.scale pour convertir les millimètres en pixels selon MAP_SCALE.
 */
ctx.translate(CANVAS_CENTER_X, CANVAS_CENTER_Y); // Set origin to center
ctx.scale(MAP_SCALE, MAP_SCALE); // Scale for mm to pixels

ws.onmessage = (event) => {
    // DEFENSE: "Sous quel format les données sont-elles reçues ?"
    // ANSWER: Format JSON, ce qui permet un parsing natif et rapide en JavaScript.
    const data = JSON.parse(event.data);
    
    // Update robot position for drawing
    robotX = data.x;
    robotY = data.y;
    pathHistory.push({x: robotX, y: robotY});

    // Update telemetry display
    document.getElementById('pose').innerText = `${robotX.toFixed(1)}, ${robotY.toFixed(1)}, ${data.h.toFixed(0)}°`;
    document.getElementById('batt').innerText = `${data.b.toFixed(1)}V`;
    document.getElementById('calibStatus').innerText = data.calib ? 'Calibré' : 'Non Calibré';

    drawMap();
};

function drawMap() {
    // Nettoyage de la zone transformée
    ctx.clearRect(-CANVAS_CENTER_X / MAP_SCALE, -CANVAS_CENTER_Y / MAP_SCALE, canvas.width / MAP_SCALE, canvas.height / MAP_SCALE); 
    
    /**
     * DEFENSE: "Comment visualisez-vous la précision du tracé (ET2.3) ?"
     * ANSWER: On stocke un historique des positions reçues (pathHistory) que l'on 
     * redessine à chaque frame. Cela permet de voir visuellement l'erreur de fermeture.
     */
    ctx.strokeStyle = '#3498db'; // Blue path
    ctx.lineWidth = 2 / MAP_SCALE; // Keep line width consistent regardless of scale
    ctx.beginPath();
    if (pathHistory.length > 0) {
        ctx.moveTo(pathHistory[0].x, pathHistory[0].y);
        for (const p of pathHistory) {
            ctx.lineTo(p.x, p.y);
        }
        ctx.stroke();
    }

    // Draw robot (simple circle for now)
    ctx.fillStyle = '#e74c3c'; // Red robot
    ctx.beginPath();
    ctx.arc(robotX, robotY, 10 / MAP_SCALE, 0, 2 * Math.PI); // Robot radius 10mm
    ctx.fill();
}

/**
 * DEFENSE: "Comment les paramètres sont-ils transmis ?"
 * ANSWER: Ils sont encapsulés dans l'objet JSON envoyé au serveur.
 */
function sendCmd(cmd, params = {}) {
    const msg = { cmd: cmd, ...params };
    ws.send(JSON.stringify(msg));
    console.log('Sent:', msg);
    
    if (cmd === 'stop') {
        ctx.clearRect(-CANVAS_CENTER_X / MAP_SCALE, -CANVAS_CENTER_Y / MAP_SCALE, canvas.width / MAP_SCALE, canvas.height / MAP_SCALE);
        pathHistory = []; // Reset path history on stop
    }
}

function sendSquares() {
    const size = parseFloat(document.getElementById('squareSize').value);
    const count = parseInt(document.getElementById('squareCount').value);
    sendCmd('squares', { size: size, count: count });
}

function sendCircle() {
    const radius = parseFloat(document.getElementById('circleRadius').value);
    sendCmd('circle', { radius: radius });
}

function sendConfig() {
    const lkp = parseFloat(document.getElementById('lkp').value);
    const lki = parseFloat(document.getElementById('lki').value);
    const akp = parseFloat(document.getElementById('akp').value);
    
    /**
     * DEFENSE: "Pourquoi permettre le réglage des PID depuis le navigateur ?"
     * ANSWER: Pour calibrer le robot en temps réel sur différentes surfaces sans 
     * avoir à reflasher le firmware, accélérant ainsi la phase de test.
     */
    ws.send(JSON.stringify({
        cmd: 'config',
        lkp: lkp, lki: lki, lkd: 0.01,
        akp: akp, aki: 0.2, akd: 0.1
    }));
}

ws.onopen = () => console.log("Connected to DrawRobot");
ws.onclose = () => {
    document.body.style.opacity = 0.5;
    alert("Connection lost. Check RobotWifi connection.");
};