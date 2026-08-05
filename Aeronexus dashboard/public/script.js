if (window.location.protocol === 'file:') {
    alert("CRITICAL ERROR: Please start the server by running 'npm start' in the 'Aeronexus dashboard' folder.");
}

// =============================================
// MULTI-DRONE AWARE — Read drone ID from URL
// =============================================

const urlParams = new URLSearchParams(window.location.search);
const DRONE_ID = urlParams.get('drone') || null;
const IS_FLEET_MODE = DRONE_ID !== null;

// Enforce Drone Authentication
let droneToken = null;
if (IS_FLEET_MODE) {
    droneToken = localStorage.getItem(`droneToken_${DRONE_ID}`);
    if (!droneToken) {
        window.location.href = `/drone_login.html?drone=${encodeURIComponent(DRONE_ID)}&target=/index.html`;
    }
}

function droneLogout() {
    if (IS_FLEET_MODE) {
        localStorage.removeItem(`droneToken_${DRONE_ID}`);
        window.location.href = '/fleet.html';
    }
}

const socket = io({
    auth: IS_FLEET_MODE ? { token: droneToken } : {}
});

// Update survey link with drone param
const surveyLink = document.getElementById('survey-link');
if (surveyLink) {
    surveyLink.href = IS_FLEET_MODE
        ? `/survey.html?drone=${encodeURIComponent(DRONE_ID)}`
        : '/survey.html';
}

// Show drone name badge if in fleet mode
if (IS_FLEET_MODE) {
    const badge = document.getElementById('drone-name-badge');
    const nameSpan = document.getElementById('active-drone-name');
    if (badge) badge.style.display = 'flex';
    if (nameSpan) nameSpan.textContent = DRONE_ID;
    document.title = `${DRONE_ID} — UAV Command`;
}

// =============================================
// COMMAND WRAPPER — Routes commands to specific drone
// =============================================

function emitCommand(type, payload) {
    if (IS_FLEET_MODE) {
        socket.emit('drone:command', { droneId: DRONE_ID, type, payload: payload || {}, droneToken: droneToken });
    } else {
        // Legacy single-drone mode
        socket.emit(type, payload || {});
    }
}

// Handle Authentication Errors
socket.on('drone:auth_error', (data) => {
    if (data.droneId === DRONE_ID) {
        alert(data.message);
        localStorage.removeItem(`droneToken_${DRONE_ID}`);
        window.location.href = `/drone_login.html?drone=${encodeURIComponent(DRONE_ID)}&target=/index.html`;
    }
});

// DOM Elements
const connIndicator = document.getElementById('conn-indicator');
const connText = document.getElementById('conn-text');
const artHorizon = document.getElementById('art-horizon');

// =============================================
// THREE.JS 3D ATTITUDE INDICATOR
// =============================================

let attitudeScene = null;

function initAttitude3D() {
    const container = document.getElementById('attitude-3d');
    if (!container || typeof THREE === 'undefined') return;

    const width = container.clientWidth || 260;
    const height = 260;
    container.style.height = height + 'px';

    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(35, width / height, 0.1, 100);
    camera.position.set(3.5, 2.5, 3.5);
    camera.lookAt(0, 0, 0);

    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(width, height);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setClearColor(0x000000, 0);
    container.appendChild(renderer.domElement);

    // Lighting
    scene.add(new THREE.AmbientLight(0x4488cc, 0.6));
    const dir = new THREE.DirectionalLight(0xffffff, 0.8);
    dir.position.set(3, 5, 3);
    scene.add(dir);
    const point = new THREE.PointLight(0x0ea5e9, 0.5, 15);
    point.position.set(0, 3, 0);
    scene.add(point);

    // Grid
    const grid = new THREE.GridHelper(8, 16, 0x1e293b, 0x0f172a);
    grid.position.y = -0.6;
    scene.add(grid);

    // Build drone model
    const droneGroup = new THREE.Group();

    // Body
    const bodyGeo = new THREE.BoxGeometry(0.9, 0.2, 0.9);
    const bodyMat = new THREE.MeshPhongMaterial({ color: 0x1e293b, emissive: 0x0a0f1a, specular: 0x334155, shininess: 80 });
    droneGroup.add(new THREE.Mesh(bodyGeo, bodyMat));

    // Dome
    const domeGeo = new THREE.SphereGeometry(0.32, 16, 8, 0, Math.PI * 2, 0, Math.PI / 2);
    const domeMat = new THREE.MeshPhongMaterial({ color: 0x1e293b, emissive: 0x0f172a, specular: 0x475569, shininess: 100, transparent: true, opacity: 0.8 });
    const dome = new THREE.Mesh(domeGeo, domeMat);
    dome.position.y = 0.1;
    droneGroup.add(dome);

    // LED strip
    const ledGeo = new THREE.BoxGeometry(0.92, 0.04, 0.92);
    const ledMat = new THREE.MeshPhongMaterial({ color: 0x0ea5e9, emissive: 0x0ea5e9, emissiveIntensity: 0.5, transparent: true, opacity: 0.8 });
    const ledStrip = new THREE.Mesh(ledGeo, ledMat);
    ledStrip.position.y = 0.02;
    droneGroup.add(ledStrip);

    // Arms, motors, propellers
    const armPos = [
        { x: 0.8, z: 0.8 },
        { x: -0.8, z: 0.8 },
        { x: -0.8, z: -0.8 },
        { x: 0.8, z: -0.8 }
    ];

    const propellers = [];
    armPos.forEach((pos, i) => {
        // Arm
        const armGeo = new THREE.BoxGeometry(1.1, 0.07, 0.09);
        const armMat = new THREE.MeshPhongMaterial({ color: 0x334155, emissive: 0x0f172a, specular: 0x475569, shininess: 60 });
        const arm = new THREE.Mesh(armGeo, armMat);
        arm.position.set(pos.x * 0.5, 0, pos.z * 0.5);
        arm.rotation.y = -Math.atan2(pos.z, pos.x);
        droneGroup.add(arm);

        // Motor
        const motorGeo = new THREE.CylinderGeometry(0.11, 0.13, 0.16, 12);
        const motorMat = new THREE.MeshPhongMaterial({ color: 0x475569, specular: 0x94a3b8, shininess: 100 });
        const motor = new THREE.Mesh(motorGeo, motorMat);
        motor.position.set(pos.x, 0.1, pos.z);
        droneGroup.add(motor);

        // Prop disc
        const propGeo = new THREE.CylinderGeometry(0.38, 0.38, 0.02, 32);
        const propMat = new THREE.MeshPhongMaterial({ color: 0x0ea5e9, emissive: 0x0ea5e9, emissiveIntensity: 0.3, transparent: true, opacity: 0.25, side: THREE.DoubleSide });
        const prop = new THREE.Mesh(propGeo, propMat);
        prop.position.set(pos.x, 0.22, pos.z);
        droneGroup.add(prop);
        propellers.push(prop);

        // Blades
        const bladeGeo = new THREE.BoxGeometry(0.65, 0.012, 0.045);
        const bladeMat = new THREE.MeshPhongMaterial({ color: 0x64748b, specular: 0x94a3b8, shininess: 80 });
        const b1 = new THREE.Mesh(bladeGeo, bladeMat);
        b1.position.set(pos.x, 0.22, pos.z);
        droneGroup.add(b1);
        propellers.push(b1);
        const b2 = b1.clone();
        b2.rotation.y = Math.PI / 2;
        b2.position.set(pos.x, 0.22, pos.z);
        droneGroup.add(b2);
        propellers.push(b2);
    });

    // Landing gear
    [{ x: 0.35, z: 0.35 }, { x: -0.35, z: 0.35 }, { x: -0.35, z: -0.35 }, { x: 0.35, z: -0.35 }].forEach(pos => {
        const legGeo = new THREE.CylinderGeometry(0.025, 0.025, 0.28, 6);
        const legMat = new THREE.MeshPhongMaterial({ color: 0x475569 });
        const leg = new THREE.Mesh(legGeo, legMat);
        leg.position.set(pos.x, -0.24, pos.z);
        droneGroup.add(leg);
        const footGeo = new THREE.SphereGeometry(0.04, 8, 4);
        const foot = new THREE.Mesh(footGeo, legMat);
        foot.position.set(pos.x, -0.38, pos.z);
        droneGroup.add(foot);
    });

    // Front LED
    const frontLedGeo = new THREE.SphereGeometry(0.045, 8, 4);
    const frontLedMat = new THREE.MeshPhongMaterial({ color: 0xff0000, emissive: 0xff0000, emissiveIntensity: 0.8 });
    const frontLed = new THREE.Mesh(frontLedGeo, frontLedMat);
    frontLed.position.set(0, 0.06, 0.47);
    droneGroup.add(frontLed);

    scene.add(droneGroup);

    attitudeScene = { scene, camera, renderer, droneGroup, propellers, ledStrip };

    function animateAttitude() {
        if (!attitudeScene) return;
        propellers.forEach((p, i) => {
            p.rotation.y += (i % 2 === 0 ? 0.12 : -0.12);
        });
        renderer.render(scene, camera);
        requestAnimationFrame(animateAttitude);
    }
    animateAttitude();
}

// Update 3D attitude from telemetry
function updateAttitude3D(roll, pitch, yaw) {
    if (!attitudeScene) return;
    const { droneGroup } = attitudeScene;
    droneGroup.rotation.order = 'YXZ';
    droneGroup.rotation.x = -(pitch || 0) * Math.PI / 180;
    droneGroup.rotation.y = -((yaw || 0) + 180) * Math.PI / 180;
    droneGroup.rotation.z = (roll || 0) * Math.PI / 180;
}

// Init 3D on load
setTimeout(initAttitude3D, 200);

// =============================================
// Initialize Chart.js
// =============================================

const ctx = document.getElementById('telemetryChart').getContext('2d');
const telemetryChart = new Chart(ctx, {
    type: 'line',
    data: {
        labels: [],
        datasets: [
            { label: 'Roll', borderColor: '#f43f5e', backgroundColor: 'rgba(244, 63, 94, 0.1)', data: [], fill: true, tension: 0.4 },
            { label: 'Pitch', borderColor: '#10b981', backgroundColor: 'rgba(16, 185, 129, 0.1)', data: [], fill: true, tension: 0.4 },
            { label: 'Yaw', borderColor: '#0ea5e9', data: [], fill: false, tension: 0.4 },
            { label: 'Altitude', borderColor: '#fb923c', data: [], fill: false, tension: 0.4 }
        ]
    },
    options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        scales: {
            x: { display: false },
            y: { grid: { color: 'rgba(255, 255, 255, 0.05)' }, ticks: { color: '#94a3b8' } }
        },
        plugins: { legend: { labels: { color: '#f8fafc' } } }
    }
});

// Survey map functionality has been moved to survey.js

function changeFlightMode() {
    const mode = document.getElementById('flight-mode').value;
    emitCommand('set_mode', { mode: mode });
}

function burnToFlash() {
    emitCommand('save_pid');
    alert("Flash Command Sent! PIDs are now permanently saved.");
}

function toggleArm(state) {
    if (state === 1) {
        const droneLabel = IS_FLEET_MODE ? ` drone "${DRONE_ID}"` : '';
        if (confirm(`WARNING: Are you sure you want to ARM${droneLabel}? Propellers may spin!`)) {
            emitCommand('toggle_arm', { arm: 1 });
        }
    } else {
        emitCommand('toggle_arm', { arm: 0 });
    }
}

// Magnetometer 3D Calibration
let magX_data = []; let magY_data = []; let magZ_data = [];
let magPlotInitialized = false;

function initMagPlot() {
    Plotly.newPlot('mag-plot', [{
        x: magX_data, y: magY_data, z: magZ_data,
        mode: 'markers',
        marker: { size: 3, color: '#0ea5e9', opacity: 0.8 },
        type: 'scatter3d'
    }], {
        margin: { l:0, r:0, b:0, t:0 },
        paper_bgcolor: 'rgba(0,0,0,0)', plot_bgcolor: 'rgba(0,0,0,0)',
        scene: {
            xaxis: { showgrid: true, gridcolor: 'rgba(255,255,255,0.1)', color: '#fff' },
            yaxis: { showgrid: true, gridcolor: 'rgba(255,255,255,0.1)', color: '#fff' },
            zaxis: { showgrid: true, gridcolor: 'rgba(255,255,255,0.1)', color: '#fff' }
        }
    });
    magPlotInitialized = true;
}

setInterval(() => {
    if(magPlotInitialized && magX_data.length > 0) {
        Plotly.update('mag-plot', {x: [magX_data], y: [magY_data], z: [magZ_data]});
    }
}, 2000); // Throttled to 2 seconds to save CPU

function calibrateMag() {
    if(magX_data.length < 50) {
        alert("Not enough data! Spin the drone around in a 3D circle first.");
        return;
    }
    const offX = (Math.max(...magX_data) + Math.min(...magX_data)) / 2;
    const offY = (Math.max(...magY_data) + Math.min(...magY_data)) / 2;
    const offZ = (Math.max(...magZ_data) + Math.min(...magZ_data)) / 2;
    
    if(confirm(`Calculated Offsets:\nX: ${offX.toFixed(2)}\nY: ${offY.toFixed(2)}\nZ: ${offZ.toFixed(2)}\n\nBurn to STM32 Flash?`)) {
        emitCommand('calibrate_mag', {x: offX, y: offY, z: offZ});
        magX_data = []; magY_data = []; magZ_data = []; // Clear plot
    }
}

// Slider Number Binding
function bindSlider(sliderId, textId) {
    const slider = document.getElementById(sliderId);
    const text = document.getElementById(textId);
    slider.addEventListener('input', () => {
        text.innerText = parseFloat(slider.value).toFixed(2);
    });
}

bindSlider('pid_r_p', 'val_r_p'); bindSlider('pid_r_i', 'val_r_i'); bindSlider('pid_r_d', 'val_r_d'); bindSlider('pid_r_f', 'val_r_f');
bindSlider('pid_p_p', 'val_p_p'); bindSlider('pid_p_i', 'val_p_i'); bindSlider('pid_p_d', 'val_p_d'); bindSlider('pid_p_f', 'val_p_f');
bindSlider('pid_y_p', 'val_y_p'); bindSlider('pid_y_i', 'val_y_i'); bindSlider('pid_y_d', 'val_y_d'); bindSlider('pid_y_f', 'val_y_f');

// Socket Events
socket.on('connect', () => {
    connIndicator.className = 'indicator green';
    connText.innerText = 'SYSTEM ONLINE';
});

socket.on('disconnect', () => {
    connIndicator.className = 'indicator red';
    connText.innerText = 'SYSTEM OFFLINE';
});

let pidInitialized = false;

// =============================================
// TELEMETRY HANDLER — Works in both fleet & legacy mode
// =============================================

window.lastChartRenderTime = 0;

function handleTelemetry(data) {
    // Update 3D Attitude
    if (data.r !== undefined || data.p !== undefined || data.y !== undefined) {
        updateAttitude3D(data.r, data.p, data.y);
    }

    // Update Attitude text
    if (data.r !== undefined) {
        document.getElementById('val-roll').innerText = data.r.toFixed(1) + '°';
        // Also update 2D horizon as fallback
        const pitchVal = data.p || 0;
        const rollVal = data.r || 0;
        if (artHorizon) {
            artHorizon.style.transform = `rotate(${rollVal}deg) translateY(${pitchVal * 2}%)`;
        }
    }
    if (data.p !== undefined) document.getElementById('val-pitch').innerText = data.p.toFixed(1) + '°';
    if (data.y !== undefined) document.getElementById('val-yaw').innerText = data.y.toFixed(1) + '°';
    
    // Update Navigation
    if (data.a !== undefined) document.getElementById('val-alt').innerText = data.a.toFixed(1) + 'm';
    if (data.d !== undefined) document.getElementById('val-heading').innerText = data.d.toFixed(1) + '°';
    
    // GPS Update
    if (data.glat !== undefined && data.glon !== undefined) {
        document.getElementById('val-lat').innerText = data.glat.toFixed(5);
        document.getElementById('val-lon').innerText = data.glon.toFixed(5);
    }
    
    if (data.gf !== undefined) {
        const fixSpan = document.getElementById('val-fix');
        if (data.gf > 0) {
            fixSpan.innerText = '3D GPS FIX';
            fixSpan.style.color = '#10b981'; // emerald
        } else {
            fixSpan.innerText = 'NO GPS FIX';
            fixSpan.style.color = '#f43f5e'; // rose
        }
    }
    
    // Battery Update
    if (data.v !== undefined) {
        const battSpan = document.getElementById('val-batt');
        battSpan.innerText = 'BAT: ' + data.v.toFixed(1) + 'V';
        if (data.v < 10.2) {
            battSpan.style.color = '#f43f5e'; // Red for low battery
        } else {
            battSpan.style.color = '#10b981'; // Green for healthy battery
        }
    }
    
    // NRF Signal Strength Update
    if (data.sig !== undefined) {
        const sigSpan = document.getElementById('val-sig');
        sigSpan.innerText = 'SIG: ' + data.sig + '%';
        if (data.sig > 80) {
            sigSpan.style.color = '#10b981'; // Green
        } else if (data.sig > 40) {
            sigSpan.style.color = '#fb923c'; // Orange
        } else {
            sigSpan.style.color = '#f43f5e'; // Red
        }
    }
    
    // Motor Outputs & Throttle
    if (data.t !== undefined) {
        document.getElementById('val-throttle').innerText = data.t;
    }
    
    // Individual Motor Speed Bars
    function updateMotorBox(motorId, boxId, pwm) {
        const valEl = document.getElementById('val-' + motorId);
        const barEl = document.getElementById('bar-' + motorId);
        const boxEl = document.getElementById(boxId);
        if (!valEl || !barEl || !boxEl) return;
        
        valEl.innerText = pwm;
        let pct = ((pwm - 1100) / (2000 - 1100)) * 100;
        if (pct < 0) pct = 0;
        if (pct > 100) pct = 100;
        barEl.style.width = pct + '%';
        
        // Color coding based on power level
        barEl.className = 'motor-bar-fill';
        boxEl.className = 'motor-box';
        if (pct > 80) {
            barEl.classList.add('danger');
            boxEl.classList.add('danger');
        } else if (pct > 50) {
            barEl.classList.add('warn');
            boxEl.classList.add('warn');
        } else if (pct > 2) {
            boxEl.classList.add('active');
        }
    }
    
    if (data.m1 !== undefined) updateMotorBox('m1', 'motor-box-bl', data.m1);
    if (data.m2 !== undefined) updateMotorBox('m2', 'motor-box-br', data.m2);
    if (data.m3 !== undefined) updateMotorBox('m3', 'motor-box-fl', data.m3);
    if (data.m4 !== undefined) updateMotorBox('m4', 'motor-box-fr', data.m4);
    
    // Flight Mode Sync
    if (data.md !== undefined) {
        const modeSelect = document.getElementById('flight-mode');
        const modeMap = {0: 'stabilize', 1: 'althold', 2: 'loiter', 3: 'rtl', 4: 'auto'};
        if (modeMap[data.md] && modeSelect.value !== modeMap[data.md]) {
            modeSelect.value = modeMap[data.md];
        }
        if (data.md === 3) modeSelect.style.background = "rgba(244, 63, 94, 0.2)"; // Red tint if RTL (geofence)
        else modeSelect.style.background = ""; 
    }
    
    // Arm State Sync — update 3D LED color
    if (data.arm !== undefined) {
        const armBtn = document.querySelector('button[onclick="toggleArm(1)"]');
        const disarmBtn = document.querySelector('button[onclick="toggleArm(0)"]');
        if (data.arm === 1) {
            armBtn.style.background = "var(--emerald)";
            armBtn.style.color = "#fff";
            disarmBtn.style.background = "rgba(244, 63, 94, 0.2)";
            disarmBtn.style.color = "var(--rose)";

            // Change 3D LED color to red when armed
            if (attitudeScene && attitudeScene.ledStrip) {
                attitudeScene.ledStrip.material.color.setHex(0xf43f5e);
                attitudeScene.ledStrip.material.emissive.setHex(0xf43f5e);
            }
        } else {
            armBtn.style.background = "rgba(16, 185, 129, 0.2)";
            armBtn.style.color = "var(--emerald)";
            disarmBtn.style.background = "var(--rose)";
            disarmBtn.style.color = "#fff";

            // Reset 3D LED color to cyan when disarmed
            if (attitudeScene && attitudeScene.ledStrip) {
                attitudeScene.ledStrip.material.color.setHex(0x0ea5e9);
                attitudeScene.ledStrip.material.emissive.setHex(0x0ea5e9);
            }
        }
    }
    
    // Magnetometer Data
    if (data.mx !== undefined && data.my !== undefined && data.mz !== undefined) {
        magX_data.push(data.mx); magY_data.push(data.my); magZ_data.push(data.mz);
        if(magX_data.length > 300) { magX_data.shift(); magY_data.shift(); magZ_data.shift(); }
    }
    
    // RC Transmitter Progress Bars (Width percentage) and Raw Values
    if (data.ry !== undefined) { 
        let pct = (data.ry + 500) / 10; 
        document.getElementById('bar-yaw').style.width = Math.max(0, Math.min(100, pct)) + "%"; 
        document.getElementById('val-rc-yaw').innerText = data.ry;
    }
    if (data.rp !== undefined) { 
        let pct = (data.rp + 500) / 10; 
        document.getElementById('bar-pit').style.width = Math.max(0, Math.min(100, pct)) + "%"; 
        document.getElementById('val-rc-pit').innerText = data.rp;
    }
    if (data.rr !== undefined) { 
        let pct = (data.rr + 500) / 10; 
        document.getElementById('bar-rol').style.width = Math.max(0, Math.min(100, pct)) + "%"; 
        document.getElementById('val-rc-rol').innerText = data.rr;
    }
    if (data.t !== undefined) { 
        let pct = ((data.t - 1000) / 10); 
        document.getElementById('bar-thr').style.width = Math.max(0, Math.min(100, pct)) + "%"; 
        document.getElementById('val-rc-thr').innerText = data.t;
    }
    
    // Auto-populate PID Sliders on first load
    if (!pidInitialized && data.pid_r && data.pid_p && data.pid_y) {
        const setPID = (axis, p, i, d, f) => {
            document.getElementById(`pid_${axis}_p`).value = p; document.getElementById(`val_${axis}_p`).innerText = p.toFixed(2);
            document.getElementById(`pid_${axis}_i`).value = i; document.getElementById(`val_${axis}_i`).innerText = i.toFixed(2);
            document.getElementById(`pid_${axis}_d`).value = d; document.getElementById(`val_${axis}_d`).innerText = d.toFixed(2);
            document.getElementById(`pid_${axis}_f`).value = f; document.getElementById(`val_${axis}_f`).innerText = f.toFixed(2);
        };
        setPID('r', data.pid_r[0], data.pid_r[1], data.pid_r[2], data.pid_r[3]);
        setPID('p', data.pid_p[0], data.pid_p[1], data.pid_p[2], data.pid_p[3]);
        setPID('y', data.pid_y[0], data.pid_y[1], data.pid_y[2], data.pid_y[3]);
        
        pidInitialized = true;
    }

    // Always update "Current" readouts
    if (data.pid_r) {
        document.getElementById('drone_r_p').innerText = data.pid_r[0].toFixed(2);
        document.getElementById('drone_r_i').innerText = data.pid_r[1].toFixed(2);
        document.getElementById('drone_r_d').innerText = data.pid_r[2].toFixed(2);
        document.getElementById('drone_r_f').innerText = data.pid_r[3].toFixed(2);
    }
    if (data.pid_p) {
        document.getElementById('drone_p_p').innerText = data.pid_p[0].toFixed(2);
        document.getElementById('drone_p_i').innerText = data.pid_p[1].toFixed(2);
        document.getElementById('drone_p_d').innerText = data.pid_p[2].toFixed(2);
        document.getElementById('drone_p_f').innerText = data.pid_p[3].toFixed(2);
    }
    if (data.pid_y) {
        document.getElementById('drone_y_p').innerText = data.pid_y[0].toFixed(2);
        document.getElementById('drone_y_i').innerText = data.pid_y[1].toFixed(2);
        document.getElementById('drone_y_d').innerText = data.pid_y[2].toFixed(2);
        document.getElementById('drone_y_f').innerText = data.pid_y[3].toFixed(2);
    }
    
    // Update Chart data arrays (5Hz)
    const nowStr = new Date().toLocaleTimeString();
    telemetryChart.data.labels.push(nowStr);
    telemetryChart.data.datasets[0].data.push(data.r || 0);
    telemetryChart.data.datasets[1].data.push(data.p || 0);
    telemetryChart.data.datasets[2].data.push(data.y || 0);
    telemetryChart.data.datasets[3].data.push(data.a || 0);
    
    if (telemetryChart.data.labels.length > 50) {
        telemetryChart.data.labels.shift();
        telemetryChart.data.datasets.forEach(dataset => dataset.data.shift());
    }
    
    // Throttle heavy chart rendering to 1Hz (once per second) to prevent browser UI freezing
    if (Date.now() - window.lastChartRenderTime > 1000) {
        telemetryChart.update();
        window.lastChartRenderTime = Date.now();
    }
}

// =============================================
// SUBSCRIBE TO TELEMETRY
// =============================================

if (IS_FLEET_MODE) {
    // Fleet mode: listen to drone:telemetry and filter by our drone ID
    socket.on('drone:telemetry', (msg) => {
        if (msg.droneId === DRONE_ID) {
            handleTelemetry(msg.data);
        }
    });
} else {
    // Legacy mode: listen to global telemetry
    socket.on('telemetry', handleTelemetry);
}



// PID Saving Function
function savePID(axis, btn) {
    let p, i, d, f;
    if (axis === 'roll') {
        p = parseFloat(document.getElementById('pid_r_p').value);
        i = parseFloat(document.getElementById('pid_r_i').value);
        d = parseFloat(document.getElementById('pid_r_d').value);
        f = parseFloat(document.getElementById('pid_r_f').value);
    } else if (axis === 'pitch') {
        p = parseFloat(document.getElementById('pid_p_p').value);
        i = parseFloat(document.getElementById('pid_p_i').value);
        d = parseFloat(document.getElementById('pid_p_d').value);
        f = parseFloat(document.getElementById('pid_p_f').value);
    } else if (axis === 'yaw') {
        p = parseFloat(document.getElementById('pid_y_p').value);
        i = parseFloat(document.getElementById('pid_y_i').value);
        d = parseFloat(document.getElementById('pid_y_d').value);
        f = parseFloat(document.getElementById('pid_y_f').value);
    }
    
    emitCommand('tune_pid', { axis: axis, p: p, i: i, d: d, f: f });
    
    // Flash button green
    const oldText = btn.innerText;
    btn.innerText = "SAVED!";
    btn.style.background = "#10b981";
    btn.style.color = "#fff";
    setTimeout(() => {
        btn.innerText = oldText;
        btn.style.background = "";
        btn.style.color = "";
    }, 1000);
}

// Initialize Magnetometer 3D Plot immediately so the box isn't empty
initMagPlot();

// Heartbeat to let the drone know the dashboard is still connected
setInterval(() => {
    emitCommand('heartbeat');
}, 1000);


// =============================================
// WAYPOINT MAP (Leaflet)
// =============================================

// Initialize the map centered on Sri Lanka (default, will auto-center on GPS fix)
const wpMap = L.map('waypoint-map', {
    center: [7.8731, 80.7718],
    zoom: 15,
    zoomControl: true,
    attributionControl: false
});

// Dark satellite-style tiles
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19
}).addTo(wpMap);

// Custom Leaflet divIcon markers
const droneIcon = L.divIcon({
    className: 'drone-marker-icon',
    iconSize: [16, 16],
    iconAnchor: [8, 8]
});

const homeIcon = L.divIcon({
    className: 'home-marker-icon',
    iconSize: [12, 12],
    iconAnchor: [6, 6]
});

const waypointIcon = L.divIcon({
    className: 'waypoint-marker-icon',
    iconSize: [14, 14],
    iconAnchor: [7, 7]
});

// Map markers (initially null)
let droneMarker = null;
let homeMarker = null;
let waypointMarker = null;
let flightPathLine = null;

// State
let lastDroneLat = 0;
let lastDroneLon = 0;
let hasAutoCenter = false;

// --- CLICK TO PLACE WAYPOINT ---
wpMap.on('click', function(e) {
    const lat = e.latlng.lat;
    const lon = e.latlng.lng;
    placeWaypointOnMap(lat, lon);
});

function placeWaypointOnMap(lat, lon) {
    // Update input fields
    document.getElementById('wp-lat-input').value = lat.toFixed(6);
    document.getElementById('wp-lon-input').value = lon.toFixed(6);

    // Place or move waypoint marker
    if (waypointMarker) {
        waypointMarker.setLatLng([lat, lon]);
    } else {
        waypointMarker = L.marker([lat, lon], { icon: waypointIcon }).addTo(wpMap);
        waypointMarker.bindTooltip('WAYPOINT', {
            permanent: false,
            direction: 'top',
            className: 'wp-tooltip'
        });
    }

    // Draw flight path line from drone to waypoint
    updateFlightPathLine(lat, lon);

    // Update distance display
    if (lastDroneLat !== 0 && lastDroneLon !== 0) {
        const dist = haversineDistance(lastDroneLat, lastDroneLon, lat, lon);
        document.getElementById('wp-distance').innerText = dist < 1000
            ? dist.toFixed(0) + 'm'
            : (dist / 1000).toFixed(2) + 'km';
    }
}

function updateFlightPathLine(wpLat, wpLon) {
    if (lastDroneLat === 0 && lastDroneLon === 0) return;

    const coords = [[lastDroneLat, lastDroneLon], [wpLat, wpLon]];

    if (flightPathLine) {
        flightPathLine.setLatLngs(coords);
    } else {
        flightPathLine = L.polyline(coords, {
            color: '#f43f5e',
            weight: 2,
            opacity: 0.7,
            dashArray: '8, 8'
        }).addTo(wpMap);
    }
}

// --- MANUAL INPUT → MAP SYNC ---
document.getElementById('wp-lat-input').addEventListener('change', syncInputsToMap);
document.getElementById('wp-lon-input').addEventListener('change', syncInputsToMap);

function syncInputsToMap() {
    const lat = parseFloat(document.getElementById('wp-lat-input').value);
    const lon = parseFloat(document.getElementById('wp-lon-input').value);
    if (!isNaN(lat) && !isNaN(lon) && lat !== 0 && lon !== 0) {
        placeWaypointOnMap(lat, lon);
        wpMap.panTo([lat, lon]);
    }
}

// --- SEND WAYPOINT TO DRONE ---
function sendWaypoint() {
    const lat = parseFloat(document.getElementById('wp-lat-input').value);
    const lon = parseFloat(document.getElementById('wp-lon-input').value);

    if (isNaN(lat) || isNaN(lon) || lat === 0 || lon === 0) {
        alert('⚠️ No waypoint selected! Click on the map or enter coordinates first.');
        return;
    }

    // Safety confirmation
    const dist = (lastDroneLat !== 0 && lastDroneLon !== 0)
        ? haversineDistance(lastDroneLat, lastDroneLon, lat, lon)
        : null;

    const distStr = dist !== null
        ? (dist < 1000 ? dist.toFixed(0) + 'm' : (dist / 1000).toFixed(2) + 'km')
        : 'unknown distance';

    const droneLabel = IS_FLEET_MODE ? ` (${DRONE_ID})` : '';

    if (!confirm(
        `🎯 FLY TO WAYPOINT?${droneLabel}\n\n` +
        `Lat: ${lat.toFixed(6)}\n` +
        `Lon: ${lon.toFixed(6)}\n` +
        `Distance: ${distStr}\n\n` +
        `The drone will switch to AUTO mode and fly to this location.`
    )) {
        return;
    }

    // Send to drone via socket
    emitCommand('send_waypoint', { lat: lat, lon: lon });

    // Visual feedback on button
    const btn = document.getElementById('wp-send-btn');
    btn.classList.add('sent');
    btn.innerHTML = '<span class="wp-send-icon">✅</span><span>WAYPOINT SENT!</span>';

    setTimeout(() => {
        btn.classList.remove('sent');
        btn.innerHTML = '<span class="wp-send-icon">🎯</span><span>FLY TO WAYPOINT</span>';
    }, 2500);
}

// --- HAVERSINE DISTANCE ---
function haversineDistance(lat1, lon1, lat2, lon2) {
    const R = 6371000; // Earth radius in meters
    const dLat = (lat2 - lat1) * Math.PI / 180;
    const dLon = (lon2 - lon1) * Math.PI / 180;
    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
              Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
              Math.sin(dLon / 2) * Math.sin(dLon / 2);
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
}

// --- UPDATE DRONE POSITION FROM TELEMETRY ---
// Use the appropriate telemetry event based on mode
function handleMapTelemetry(data) {
    // Update drone marker on map
    if (data.glat !== undefined && data.glon !== undefined && data.glat !== 0 && data.glon !== 0) {
        lastDroneLat = data.glat;
        lastDroneLon = data.glon;

        // Update overlay HUD
        document.getElementById('wp-drone-pos').innerText =
            data.glat.toFixed(5) + ', ' + data.glon.toFixed(5);

        // Place or move drone marker
        if (droneMarker) {
            droneMarker.setLatLng([data.glat, data.glon]);
        } else {
            droneMarker = L.marker([data.glat, data.glon], { icon: droneIcon, zIndexOffset: 1000 }).addTo(wpMap);
            droneMarker.bindTooltip(IS_FLEET_MODE ? DRONE_ID : 'DRONE', {
                permanent: false,
                direction: 'top',
                className: 'wp-tooltip'
            });
        }

        // Auto-center map on first GPS fix
        if (!hasAutoCenter) {
            wpMap.setView([data.glat, data.glon], 17);
            hasAutoCenter = true;
        }

        // Update home marker (home is set on ARM, approximate using first fix)
        if (data.arm === 1 && !homeMarker) {
            homeMarker = L.marker([data.glat, data.glon], { icon: homeIcon }).addTo(wpMap);
            homeMarker.bindTooltip('HOME', {
                permanent: false,
                direction: 'top',
                className: 'wp-tooltip'
            });
            document.getElementById('wp-home-pos').innerText =
                data.glat.toFixed(5) + ', ' + data.glon.toFixed(5);
        }

        // Update flight path line if waypoint exists
        if (waypointMarker) {
            const wpLatLng = waypointMarker.getLatLng();
            updateFlightPathLine(wpLatLng.lat, wpLatLng.lng);

            // Update distance
            const dist = haversineDistance(data.glat, data.glon, wpLatLng.lat, wpLatLng.lng);
            document.getElementById('wp-distance').innerText = dist < 1000
                ? dist.toFixed(0) + 'm'
                : (dist / 1000).toFixed(2) + 'km';
        }
    }

    // Clear home marker on disarm
    if (data.arm === 0 && homeMarker) {
        wpMap.removeLayer(homeMarker);
        homeMarker = null;
        document.getElementById('wp-home-pos').innerText = '—';
    }
}

// Register map telemetry handler on the right event
if (IS_FLEET_MODE) {
    socket.on('drone:telemetry', (msg) => {
        if (msg.droneId === DRONE_ID) {
            handleMapTelemetry(msg.data);
        }
    });
} else {
    socket.on('telemetry', handleMapTelemetry);
}

// Fix Leaflet rendering in initially hidden/resized panels
setTimeout(() => { wpMap.invalidateSize(); }, 500);
