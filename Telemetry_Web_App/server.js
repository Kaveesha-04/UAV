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

const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');

tcpServer.listen(TCP_PORT, '0.0.0.0', () => {
    console.log(`[WIFI] TCP Server listening for ESP32 on port ${TCP_PORT}`);
});

// ==========================================
// USB SERIAL FALLBACK (Plug and Play)
// ==========================================
let activeSerialPort = null;

async function connectToSTM32Serial() {
    try {
        const ports = await SerialPort.list();
        // Look for STMicroelectronics Virtual COM Port (Vendor ID 0483)
        const stmPortInfo = ports.find(p => p.vendorId && p.vendorId.toUpperCase() === '0483');
        
        if (stmPortInfo) {
            console.log(`[USB] Found STM32 on ${stmPortInfo.path}. Connecting...`);
            activeSerialPort = new SerialPort({ path: stmPortInfo.path, baudRate: 115200 });
            const parser = activeSerialPort.pipe(new ReadlineParser({ delimiter: '\n' }));

            parser.on('data', (jsonString) => {
                if (jsonString.trim().length > 0) {
                    try {
                        const telemetry = JSON.parse(jsonString.trim());
                        // Only broadcast USB data if ESP32 is NOT currently connected
                        if (!activeEspSocket) {
                            io.emit('telemetry', telemetry);
                        }
                    } catch (err) {
                        // Ignore parse errors (partial frames)
                        console.log("[USB] JSON Parse Error:", err.message);
                        console.log("[USB] Raw String:", jsonString.trim());
                    }
                }
            });

            activeSerialPort.on('close', () => {
                console.log(`[USB] STM32 disconnected.`);
                activeSerialPort = null;
                setTimeout(connectToSTM32Serial, 3000); // Try to reconnect
            });

            activeSerialPort.on('error', (err) => {
                console.log(`[USB] Error: ${err.message}`);
                activeSerialPort = null;
                setTimeout(connectToSTM32Serial, 3000);
            });
        } else {
            // Check again in 3 seconds
            setTimeout(connectToSTM32Serial, 3000);
        }
    } catch (e) {
        setTimeout(connectToSTM32Serial, 3000);
    }
}
connectToSTM32Serial();

// WebSocket Handling
io.on('connection', (wsSocket) => {
    console.log('Web Dashboard Client Connected');
    
    function sendCommandToDrone(cmd) {
        if (activeEspSocket) {
            console.log('[WIFI] Sending to ESP32:', cmd.trim());
            activeEspSocket.write(cmd);
        } else if (activeSerialPort) {
            console.log('[USB] Sending to STM32:', cmd.trim());
            activeSerialPort.write(cmd);
        } else {
            console.log('[ERROR] Cannot send command - No drone connected (WiFi or USB)!');
        }
    }

    wsSocket.on('tune_pid', (data) => {
        // Convert floats to integers (*100) to avoid STM32 sscanf %f float parsing issues!
        let p_int = Math.round(data.p * 100);
        let i_int = Math.round(data.i * 100);
        let d_int = Math.round(data.d * 100);
        let f_int = Math.round(data.f * 100);
        sendCommandToDrone(`P,${data.axis},${p_int},${i_int},${d_int},${f_int}\n`);
    });

    wsSocket.on('send_waypoint', (data) => {
        sendCommandToDrone(`W,${data.lat},${data.lon}\n`);
    });

    wsSocket.on('set_mode', (data) => {
        sendCommandToDrone(`M,${data.mode}\n`);
    });

    wsSocket.on('save_pid', () => {
        sendCommandToDrone(`B\n`);
    });

    wsSocket.on('toggle_arm', (data) => {
        sendCommandToDrone(`A,${data.arm}\n`);
    });

    wsSocket.on('calibrate_mag', (data) => {
        sendCommandToDrone(`C,${data.x},${data.y},${data.z}\n`);
    });

    wsSocket.on('heartbeat', () => {
        sendCommandToDrone(`H\n`);
    });

    // --- AUTO SURVEY MISSION COMMANDS ---
    wsSocket.on('survey_reset', () => {
        sendCommandToDrone(`S,RESET\n`);
    });

    wsSocket.on('survey_waypoint', (data) => {
        sendCommandToDrone(`S,WP,${data.lat},${data.lon}\n`);
    });

    wsSocket.on('survey_start', () => {
        sendCommandToDrone(`S,START\n`);
    });

    wsSocket.on('survey_pause', () => {
        sendCommandToDrone(`S,PAUSE\n`);
    });

    wsSocket.on('survey_resume', () => {
        sendCommandToDrone(`S,RESUME\n`);
    });

    wsSocket.on('survey_abort', () => {
        sendCommandToDrone(`S,ABORT\n`);
    });

});

// Start Web Server
const WEB_PORT = 3000;
server.listen(WEB_PORT, () => {
    console.log(`Web Dashboard running at http://localhost:${WEB_PORT}`);
});
