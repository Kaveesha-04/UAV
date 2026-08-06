const express = require('express');
const http = require('http');
const net = require('net');
const { Server } = require('socket.io');

const app = express();
const server = http.createServer(app);
const io = new Server(server);

// Auth storage
const crypto = require('crypto');
const ADMIN_PASSWORD = 'admin123'; // Global Fleet Password
const fleetTokens = new Set();
const droneTokens = new Map(); // token -> droneId
const dronePins = new Map(); // droneId -> PIN

app.use(express.json());

// --- REST API AUTHENTICATION ---

app.post('/api/login/fleet', (req, res) => {
    const { password } = req.body;
    if (password === ADMIN_PASSWORD) {
        const token = crypto.randomUUID();
        fleetTokens.add(token);
        res.json({ success: true, token });
    } else {
        res.status(401).json({ success: false, message: 'Invalid Fleet Password' });
    }
});

app.post('/api/login/drone', (req, res) => {
    const { droneId, pin } = req.body;
    if (dronePins.get(droneId) === pin) {
        const token = crypto.randomUUID();
        droneTokens.set(token, droneId);
        res.json({ success: true, token });
    } else {
        res.status(401).json({ success: false, message: 'Invalid Drone PIN' });
    }
});

// Serve static files (the web interface)
// Disable auto index.html so root '/' hits our redirect to fleet.html
app.use(express.static('public', { index: false }));

// Redirect root to login page
app.get('/', (req, res) => {
    res.redirect('/login.html');
});

// ==========================================
// MULTI-DRONE REGISTRY
// ==========================================

// Named drones: Map<droneId, { socket, lastTelemetry, assignedAt, connectionType }>
const droneRegistry = new Map();

// Unnamed TCP connections waiting to be assigned
const pendingConnections = [];

// Auto-incrementing index for pending connections
let pendingCounter = 0;

// ==========================================
// TCP SERVER — MULTI-DRONE
// ==========================================
const TCP_PORT = 5000;

const tcpServer = net.createServer((socket) => {
    pendingCounter++;
    const pendingId = pendingCounter;
    const pendingEntry = {
        id: pendingId,
        socket: socket,
        connectedAt: new Date().toISOString(),
        remoteAddress: socket.remoteAddress,
        buffer: '',
        previewData: null  // Store first telemetry for preview
    };

    console.log(`[TCP] New ESP32 connection #${pendingId} from ${socket.remoteAddress}`);
    pendingConnections.push(pendingEntry);
    broadcastFleetStatus();

    socket.on('data', (data) => {
        pendingEntry.buffer += data.toString();
        let boundary = pendingEntry.buffer.indexOf('\n');

        while (boundary !== -1) {
            let jsonString = pendingEntry.buffer.substring(0, boundary).trim();
            pendingEntry.buffer = pendingEntry.buffer.substring(boundary + 1);
            boundary = pendingEntry.buffer.indexOf('\n');

            if (jsonString.length > 0) {
                try {
                    const telemetry = JSON.parse(jsonString);

                    // Check if this socket has been assigned to a drone
                    let assignedDroneId = null;
                    for (const [droneId, entry] of droneRegistry) {
                        if (entry.socket === socket) {
                            assignedDroneId = droneId;
                            break;
                        }
                    }

                    if (assignedDroneId) {
                        // Drone is named — broadcast tagged telemetry
                        const entry = droneRegistry.get(assignedDroneId);
                        entry.lastTelemetry = telemetry;
                        entry.lastSeen = Date.now();

                        // Emit to all clients with droneId tag
                        io.emit('drone:telemetry', { droneId: assignedDroneId, data: telemetry });

                        // Also emit legacy 'telemetry' for backward compatibility
                        // (single-drone dashboards that don't know about fleet)
                        io.emit('telemetry', telemetry);
                    } else {
                        // Still in pending pool — store preview data
                        pendingEntry.previewData = telemetry;
                        broadcastFleetStatus();
                    }
                } catch (err) {
                    // Ignore parse errors
                }
            }
        }
    });

    socket.on('end', () => {
        console.log(`[TCP] ESP32 connection #${pendingId} disconnected`);
        removeDroneBySocket(socket);
        removePendingBySocket(socket);
        broadcastFleetStatus();
    });

    socket.on('error', (err) => {
        console.log(`[TCP] Error on connection #${pendingId}:`, err.message);
        removeDroneBySocket(socket);
        removePendingBySocket(socket);
        broadcastFleetStatus();
    });

    socket.on('close', () => {
        console.log(`[TCP] Connection #${pendingId} closed`);
        removeDroneBySocket(socket);
        removePendingBySocket(socket);
        broadcastFleetStatus();
    });
});

tcpServer.listen(TCP_PORT, '0.0.0.0', () => {
    console.log(`[WIFI] TCP Server listening for ESP32 drones on port ${TCP_PORT}`);
});

// ==========================================
// USB SERIAL — AUTO-ASSIGNED AS DRONE
// ==========================================
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');

let activeSerialPort = null;

async function connectToSTM32Serial() {
    try {
        const ports = await SerialPort.list();
        const stmPortInfo = ports.find(p => p.vendorId && p.vendorId.toUpperCase() === '0483');

        if (stmPortInfo) {
            console.log(`[USB] Found STM32 on ${stmPortInfo.path}. Connecting...`);
            activeSerialPort = new SerialPort({ path: stmPortInfo.path, baudRate: 115200 });
            const parser = activeSerialPort.pipe(new ReadlineParser({ delimiter: '\n' }));

            // Register as a USB drone
            const usbDroneId = 'USB-Direct';
            dronePins.set(usbDroneId, '0000'); // Default PIN for USB connections
            droneRegistry.set(usbDroneId, {
                socket: null,
                serialPort: activeSerialPort,
                lastTelemetry: null,
                assignedAt: new Date().toISOString(),
                connectionType: 'USB',
                lastSeen: Date.now()
            });
            broadcastFleetStatus();

            parser.on('data', (jsonString) => {
                if (jsonString.trim().length > 0) {
                    try {
                        const telemetry = JSON.parse(jsonString.trim());
                        const entry = droneRegistry.get(usbDroneId);
                        if (entry) {
                            entry.lastTelemetry = telemetry;
                            entry.lastSeen = Date.now();
                        }
                        io.emit('drone:telemetry', { droneId: usbDroneId, data: telemetry });
                        io.emit('telemetry', telemetry);
                    } catch (err) {
                        // Ignore parse errors
                    }
                }
            });

            activeSerialPort.on('close', () => {
                console.log(`[USB] STM32 disconnected.`);
                droneRegistry.delete(usbDroneId);
                activeSerialPort = null;
                broadcastFleetStatus();
                setTimeout(connectToSTM32Serial, 3000);
            });

            activeSerialPort.on('error', (err) => {
                console.log(`[USB] Error: ${err.message}`);
                droneRegistry.delete(usbDroneId);
                activeSerialPort = null;
                broadcastFleetStatus();
                setTimeout(connectToSTM32Serial, 3000);
            });
        } else {
            setTimeout(connectToSTM32Serial, 3000);
        }
    } catch (e) {
        setTimeout(connectToSTM32Serial, 3000);
    }
}
connectToSTM32Serial();

// ==========================================
// HELPER FUNCTIONS
// ==========================================

function removeDroneBySocket(socket) {
    for (const [droneId, entry] of droneRegistry) {
        if (entry.socket === socket) {
            console.log(`[FLEET] Drone "${droneId}" disconnected`);
            droneRegistry.delete(droneId);
            return;
        }
    }
}

function removePendingBySocket(socket) {
    const idx = pendingConnections.findIndex(p => p.socket === socket);
    if (idx !== -1) {
        pendingConnections.splice(idx, 1);
    }
}

function sendCommandToDrone(droneId, cmd) {
    const entry = droneRegistry.get(droneId);
    if (!entry) {
        // Don't spam logs for heartbeats to non-existent drones
        return false;
    }

    try {
        if (entry.connectionType === 'USB' && entry.serialPort) {
            console.log(`[USB] Sending to "${droneId}":`, cmd.trim());
            entry.serialPort.write(cmd);
            return true;
        } else if (entry.socket) {
            console.log(`[WIFI] Sending to "${droneId}":`, cmd.trim());
            entry.socket.write(cmd);
            return true;
        }
    } catch (err) {
        console.log(`[ERROR] Failed to write to drone "${droneId}":`, err.message);
        return false;
    }

    console.log(`[ERROR] No active connection for drone "${droneId}"`);
    return false;
}

function getFleetStatus() {
    const drones = [];
    for (const [droneId, entry] of droneRegistry) {
        drones.push({
            droneId: droneId,
            connectionType: entry.connectionType || 'WiFi',
            assignedAt: entry.assignedAt,
            lastSeen: entry.lastSeen,
            online: entry.lastSeen ? (Date.now() - entry.lastSeen < 5000) : false,
            telemetry: entry.lastTelemetry || null
        });
    }

    const pending = pendingConnections.map(p => ({
        id: p.id,
        connectedAt: p.connectedAt,
        remoteAddress: p.remoteAddress,
        hasData: p.previewData !== null,
        previewVoltage: p.previewData ? p.previewData.v : null,
        previewGpsFix: p.previewData ? p.previewData.gf : null
    }));

    return { drones, pending };
}

function broadcastFleetStatus() {
    io.emit('fleet:status', getFleetStatus());
}

// Broadcast fleet status every 2 seconds
setInterval(broadcastFleetStatus, 2000);

// ==========================================
// REST API
// ==========================================

app.get('/api/fleet', (req, res) => {
    res.json(getFleetStatus());
});

// ==========================================
// WEBSOCKET — MULTI-DRONE COMMANDS
// ==========================================

io.on('connection', (wsSocket) => {
    console.log('[WS] Dashboard client connected');

    // Validate token on connection
    const token = wsSocket.handshake.auth?.token;
    if (token && !fleetTokens.has(token)) {
        wsSocket.emit('fleet:error', { message: 'Unauthorized session expired. Please login again.' });
    }

    // Send initial fleet status
    wsSocket.emit('fleet:status', getFleetStatus());

    // --- FLEET MANAGEMENT ---

    wsSocket.on('fleet:assign', (data) => {
        const { connectionIndex, droneId, pin, fleetToken } = data;
        
        // Ensure user is authenticated as Fleet Admin
        if (!fleetToken || !fleetTokens.has(fleetToken)) {
            wsSocket.emit('fleet:error', { message: 'Unauthorized. Please login to Fleet Command.' });
            return;
        }

        if (!droneId || droneId.trim().length === 0) {
            wsSocket.emit('fleet:error', { message: 'Drone ID cannot be empty' });
            return;
        }

        if (droneRegistry.has(droneId)) {
            wsSocket.emit('fleet:error', { message: `Drone "${droneId}" already exists` });
            return;
        }

        // Find pending connection by id
        const pendingIdx = pendingConnections.findIndex(p => p.id === connectionIndex);
        if (pendingIdx === -1) {
            wsSocket.emit('fleet:error', { message: 'Pending connection not found' });
            return;
        }

        const pending = pendingConnections.splice(pendingIdx, 1)[0];

        // Store the drone's access PIN
        dronePins.set(droneId, pin || '0000');

        droneRegistry.set(droneId, {
            socket: pending.socket,
            lastTelemetry: pending.previewData,
            assignedAt: new Date().toISOString(),
            connectionType: 'WiFi',
            lastSeen: pending.previewData ? Date.now() : null
        });

        console.log(`[FLEET] Assigned connection #${pending.id} as drone "${droneId}"`);
        broadcastFleetStatus();
    });

    wsSocket.on('fleet:remove', (data) => {
        const { droneId, fleetToken } = data;
        
        if (!fleetToken || !fleetTokens.has(fleetToken)) {
            wsSocket.emit('fleet:error', { message: 'Unauthorized. Please login to Fleet Command.' });
            return;
        }

        const entry = droneRegistry.get(droneId);
        if (entry) {
            if (entry.socket) {
                try { entry.socket.destroy(); } catch(e) {}
            }
            droneRegistry.delete(droneId);
            console.log(`[FLEET] Removed drone "${droneId}"`);
            broadcastFleetStatus();
        }
    });

    wsSocket.on('fleet:auto_assign', (data) => {
        // Ensure user is authenticated as Fleet Admin
        if (!data.fleetToken || !fleetTokens.has(data.fleetToken)) {
            wsSocket.emit('fleet:error', { message: 'Unauthorized. Please login to Fleet Command.' });
            return;
        }

        // Auto-assign all pending connections with sequential names
        const prefix = data.prefix || 'Drone';
        let counter = droneRegistry.size + 1;

        while (pendingConnections.length > 0) {
            let name = `${prefix}-${counter}`;
            while (droneRegistry.has(name)) {
                counter++;
                name = `${prefix}-${counter}`;
            }

            const pending = pendingConnections.shift();
            dronePins.set(name, data.pin || '0000');
            droneRegistry.set(name, {
                socket: pending.socket,
                lastTelemetry: pending.previewData,
                assignedAt: new Date().toISOString(),
                connectionType: 'WiFi',
                lastSeen: pending.previewData ? Date.now() : null
            });

            console.log(`[FLEET] Auto-assigned connection #${pending.id} as "${name}"`);
            counter++;
        }
        broadcastFleetStatus();
    });

    // --- DRONE-SPECIFIC COMMANDS ---

    wsSocket.on('drone:command', (data = {}) => {
        const { droneId, type, payload = {}, droneToken } = data;
        if (!droneId) return;

        // Skip auth for heartbeat to keep connection alive silently
        if (type !== 'heartbeat') {
            const hasDroneAccess = droneToken && droneTokens.get(droneToken) === droneId;
            const hasFleetAccess = data.fleetToken && fleetTokens.has(data.fleetToken);
            if (!hasDroneAccess && !hasFleetAccess) {
                // Not authenticated for this drone
                console.log(`[AUTH] Unauthorized command attempt to ${droneId}`);
                wsSocket.emit('drone:auth_error', { droneId, message: 'Unauthorized. Please login to this drone.' });
                return;
            }
        }

        switch (type) {
            case 'tune_pid': {
                let p_int = Math.round(payload.p * 100);
                let i_int = Math.round(payload.i * 100);
                let d_int = Math.round(payload.d * 100);
                let f_int = Math.round(payload.f * 100);
                sendCommandToDrone(droneId, `P,${payload.axis},${p_int},${i_int},${d_int},${f_int}\n`);
                break;
            }
            case 'send_waypoint':
                sendCommandToDrone(droneId, `W,${payload.lat},${payload.lon}\n`);
                break;
            case 'set_mode':
                sendCommandToDrone(droneId, `M,${payload.mode}\n`);
                break;
            case 'save_pid':
                sendCommandToDrone(droneId, `B\n`);
                break;
            case 'toggle_arm':
                sendCommandToDrone(droneId, `A,${payload.arm}\n`);
                break;
            case 'calibrate_mag':
                sendCommandToDrone(droneId, `C,${payload.x},${payload.y},${payload.z}\n`);
                break;
            case 'set_declination':
                sendCommandToDrone(droneId, `C,DECL,${payload.declination}\n`);
                break;
            case 'heartbeat':
                sendCommandToDrone(droneId, `H\n`);
                break;
            case 'survey_reset':
                sendCommandToDrone(droneId, `S,RESET\n`);
                break;
            case 'survey_waypoint':
                sendCommandToDrone(droneId, `S,WP,${payload.lat},${payload.lon}\n`);
                break;
            case 'survey_start':
                sendCommandToDrone(droneId, `S,START\n`);
                break;
            case 'survey_pause':
                sendCommandToDrone(droneId, `S,PAUSE\n`);
                break;
            case 'survey_resume':
                sendCommandToDrone(droneId, `S,RESUME\n`);
                break;
            case 'survey_abort':
                sendCommandToDrone(droneId, `S,ABORT\n`);
                break;
        }
    });

    // --- LEGACY SINGLE-DRONE COMMANDS (backward compatibility) ---
    // These find the first available drone or a specific one

    function getLegacyDrone() {
        // Return first drone in registry
        if (droneRegistry.size > 0) {
            return droneRegistry.keys().next().value;
        }
        return null;
    }

    wsSocket.on('tune_pid', (data = {}) => {
        const droneId = getLegacyDrone();
        if (!droneId) return;
        let p_int = Math.round((data.p || 0) * 100);
        let i_int = Math.round((data.i || 0) * 100);
        let d_int = Math.round((data.d || 0) * 100);
        let f_int = Math.round((data.f || 0) * 100);
        sendCommandToDrone(droneId, `P,${data.axis || 'roll'},${p_int},${i_int},${d_int},${f_int}\n`);
    });

    wsSocket.on('send_waypoint', (data = {}) => {
        const droneId = getLegacyDrone();
        if (droneId) sendCommandToDrone(droneId, `W,${data.lat},${data.lon}\n`);
    });

    wsSocket.on('set_mode', (data = {}) => {
        const droneId = getLegacyDrone();
        if (droneId) sendCommandToDrone(droneId, `M,${data.mode}\n`);
    });

    wsSocket.on('save_pid', () => {
        const droneId = getLegacyDrone();
        if (droneId) sendCommandToDrone(droneId, `B\n`);
    });

    wsSocket.on('toggle_arm', (data = {}) => {
        const droneId = getLegacyDrone();
        if (droneId) sendCommandToDrone(droneId, `A,${data.arm}\n`);
    });

    wsSocket.on('calibrate_mag', (data = {}) => {
        const droneId = getLegacyDrone();
        if (droneId) sendCommandToDrone(droneId, `C,${data.x},${data.y},${data.z}\n`);
    });

    wsSocket.on('heartbeat', () => {
        const droneId = getLegacyDrone();
        if (droneId) sendCommandToDrone(droneId, `H\n`);
    });

    wsSocket.on('survey_reset', () => {
        const droneId = getLegacyDrone();
        if (droneId) sendCommandToDrone(droneId, `S,RESET\n`);
    });

    wsSocket.on('survey_waypoint', (data = {}) => {
        const droneId = getLegacyDrone();
        if (droneId) sendCommandToDrone(droneId, `S,WP,${data.lat},${data.lon}\n`);
    });

    wsSocket.on('survey_start', () => {
        const droneId = getLegacyDrone();
        if (droneId) sendCommandToDrone(droneId, `S,START\n`);
    });

    wsSocket.on('survey_pause', () => {
        const droneId = getLegacyDrone();
        if (droneId) sendCommandToDrone(droneId, `S,PAUSE\n`);
    });

    wsSocket.on('survey_resume', () => {
        const droneId = getLegacyDrone();
        if (droneId) sendCommandToDrone(droneId, `S,RESUME\n`);
    });

    wsSocket.on('survey_abort', () => {
        const droneId = getLegacyDrone();
        if (droneId) sendCommandToDrone(droneId, `S,ABORT\n`);
    });
});

// Start Web Server
const WEB_PORT = 3000;
server.listen(WEB_PORT, () => {
    console.log(`\n╔══════════════════════════════════════════════════╗`);
    console.log(`║   UAV FLEET COMMAND CENTER                       ║`);
    console.log(`║   Dashboard: http://localhost:${WEB_PORT}               ║`);
    console.log(`║   TCP Port:  ${TCP_PORT} (ESP32 drones)                 ║`);
    console.log(`╚══════════════════════════════════════════════════╝\n`);
});
