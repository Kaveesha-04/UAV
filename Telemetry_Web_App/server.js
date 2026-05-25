const express = require('express');
const http = require('http');
const net = require('net');
const { Server } = require('socket.io');

const app = express();
const server = http.createServer(app);
const io = new Server(server);

// Serve static files (the web interface)
app.use(express.static('public'));

// TCP Server to receive data from ESP32
const TCP_PORT = 5000;
let activeEspSocket = null;

const tcpServer = net.createServer((socket) => {
    console.log('ESP32 Connected to TCP Server!');
    activeEspSocket = socket;
    
    let buffer = '';

    socket.on('data', (data) => {
        buffer += data.toString();
        let boundary = buffer.indexOf('\n');
        
        while (boundary !== -1) {
            let jsonString = buffer.substring(0, boundary).trim();
            buffer = buffer.substring(boundary + 1);
            boundary = buffer.indexOf('\n');
            
            if (jsonString.length > 0) {
                try {
                    const telemetry = JSON.parse(jsonString);
                    console.log("Broadcasting telemetry:", telemetry); // ADDED FOR DEBUGGING
                    // Broadcast to all connected web clients
                    io.emit('telemetry', telemetry);
                } catch (err) {
                    console.log("Invalid JSON payload received:", jsonString);
                    console.error("JSON Parse Error:", err.message);
                }
            }
        }
    });

    socket.on('end', () => {
        console.log('ESP32 Disconnected');
        if (activeEspSocket === socket) activeEspSocket = null;
    });
    socket.on('error', (err) => {
        console.log('TCP Error:', err.message);
        if (activeEspSocket === socket) activeEspSocket = null;
    });
});

tcpServer.listen(TCP_PORT, '0.0.0.0', () => {
    console.log(`TCP Server listening for ESP32 on port ${TCP_PORT}`);
});

// WebSocket Handling
io.on('connection', (wsSocket) => {
    console.log('Web Dashboard Client Connected');
    
    wsSocket.on('tune_pid', (data) => {
        if (activeEspSocket) {
            // Format to CSV: P,roll,1.20,0.05,0.01\n
            const cmd = `P,${data.axis},${data.p},${data.i},${data.d}\n`;
            console.log('Sending to ESP32:', cmd.trim());
            activeEspSocket.write(cmd);
        } else {
            console.log('Cannot send PID - ESP32 not connected!');
        }
    });

    wsSocket.on('send_waypoint', (data) => {
        if (activeEspSocket) {
            const cmd = `W,${data.lat},${data.lon}\n`;
            console.log('Sending Waypoint to ESP32:', cmd.trim());
            activeEspSocket.write(cmd);
        }
    });

    wsSocket.on('set_mode', (data) => {
        if (activeEspSocket) {
            const cmd = `M,${data.mode}\n`;
            console.log('Sending Flight Mode to ESP32:', cmd.trim());
            activeEspSocket.write(cmd);
        }
    });

    wsSocket.on('save_pid', () => {
        if (activeEspSocket) {
            const cmd = `B\n`;
            console.log('Sending Burn to Flash Command to ESP32:', cmd.trim());
            activeEspSocket.write(cmd);
        }
    });

    wsSocket.on('calibrate_mag', (data) => {
        if (activeEspSocket) {
            const cmd = `C,${data.x},${data.y},${data.z}\n`;
            console.log('Sending Mag Offsets to ESP32:', cmd.trim());
            activeEspSocket.write(cmd);
        }
    });
});

// Start Web Server
const WEB_PORT = 3000;
server.listen(WEB_PORT, () => {
    console.log(`Web Dashboard running at http://localhost:${WEB_PORT}`);
});
