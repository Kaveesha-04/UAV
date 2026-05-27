if (window.location.protocol === 'file:') {
    alert("CRITICAL ERROR: Please start the server by running 'npm start' in the Telemetry_Web_App folder.");
}

const socket = io();

// DOM Elements
const connIndicator = document.getElementById('conn-indicator');
const connText = document.getElementById('conn-text');
const artHorizon = document.getElementById('art-horizon');

// Initialize Chart.js
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

// Initialize Leaflet Map
const map = L.map('map').setView([0, 0], 2);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    attribution: '© OpenStreetMap contributors',
    className: 'map-tiles'
}).addTo(map);

let droneMarker = L.marker([0, 0]).addTo(map);
let mapInitialized = false;

map.on('click', function(e) {
    const lat = e.latlng.lat;
    const lon = e.latlng.lng;
    if(confirm(`Send Waypoint to Drone:\nLat: ${lat.toFixed(5)}\nLon: ${lon.toFixed(5)} ?`)) {
        socket.emit('send_waypoint', { lat: lat, lon: lon });
    }
});

function changeFlightMode() {
    const mode = document.getElementById('flight-mode').value;
    socket.emit('set_mode', { mode: mode });
}

function burnToFlash() {
    socket.emit('save_pid');
    alert("Flash Command Sent! PIDs are now permanently saved.");
}

function toggleArm(state) {
    if (state === 1) {
        if (confirm("WARNING: Are you sure you want to ARM the drone? Propellers may spin!")) {
            socket.emit('toggle_arm', { arm: 1 });
        }
    } else {
        socket.emit('toggle_arm', { arm: 0 });
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
}, 500);

function calibrateMag() {
    if(magX_data.length < 50) {
        alert("Not enough data! Spin the drone around in a 3D circle first.");
        return;
    }
    const offX = (Math.max(...magX_data) + Math.min(...magX_data)) / 2;
    const offY = (Math.max(...magY_data) + Math.min(...magY_data)) / 2;
    const offZ = (Math.max(...magZ_data) + Math.min(...magZ_data)) / 2;
    
    if(confirm(`Calculated Offsets:\nX: ${offX.toFixed(2)}\nY: ${offY.toFixed(2)}\nZ: ${offZ.toFixed(2)}\n\nBurn to STM32 Flash?`)) {
        socket.emit('calibrate_mag', {x: offX, y: offY, z: offZ});
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

bindSlider('pid_r_p', 'val_r_p'); bindSlider('pid_r_i', 'val_r_i'); bindSlider('pid_r_d', 'val_r_d');
bindSlider('pid_p_p', 'val_p_p'); bindSlider('pid_p_i', 'val_p_i'); bindSlider('pid_p_d', 'val_p_d');
bindSlider('pid_y_p', 'val_y_p'); bindSlider('pid_y_i', 'val_y_i'); bindSlider('pid_y_d', 'val_y_d');

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

socket.on('telemetry', (data) => {
    // Update Attitude
    if (data.r !== undefined) {
        document.getElementById('val-roll').innerText = data.r.toFixed(1) + '°';
        // Artificial Horizon Roll (Rotate)
        // Transform the horizon: rotation is roll, translation is pitch
        // Note: For a realistic horizon, positive pitch pushes the ground UP
        const pitchVal = data.p || 0;
        const rollVal = data.r || 0;
        artHorizon.style.transform = `rotate(${rollVal}deg) translateY(${pitchVal * 2}%)`;
    }
    if (data.p !== undefined) document.getElementById('val-pitch').innerText = data.p.toFixed(1) + '°';
    if (data.y !== undefined) document.getElementById('val-yaw').innerText = data.y.toFixed(1) + '°';
    
    // Update Navigation
    if (data.a !== undefined) document.getElementById('val-alt').innerText = data.a.toFixed(1) + 'm';
    if (data.d !== undefined) document.getElementById('val-heading').innerText = data.d.toFixed(1) + '°';
    
    // GPS & Map Update
    if (data.glat !== undefined && data.glon !== undefined) {
        document.getElementById('val-lat').innerText = data.glat.toFixed(5);
        document.getElementById('val-lon').innerText = data.glon.toFixed(5);
        
        if (data.glat !== 0 && data.glon !== 0) {
            const newLatLng = new L.LatLng(data.glat, data.glon);
            droneMarker.setLatLng(newLatLng);
            if (!mapInitialized) {
                map.setView(newLatLng, 17); // Zoom in when first fix acquired
                setTimeout(() => map.invalidateSize(), 500);
                mapInitialized = true;
            }
        }
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
    
    // Engine Power
    if (data.t !== undefined) {
        document.getElementById('val-throttle').innerText = data.t;
        let t_pct = ((data.t - 1000) / 1000) * 100;
        if(t_pct < 0) t_pct = 0;
        if(t_pct > 100) t_pct = 100;
        document.getElementById('throttle-bar').style.width = t_pct + '%';
    }
    
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
        const setPID = (axis, p, i, d) => {
            document.getElementById(`pid_${axis}_p`).value = p; document.getElementById(`val_${axis}_p`).innerText = p.toFixed(2);
            document.getElementById(`pid_${axis}_i`).value = i; document.getElementById(`val_${axis}_i`).innerText = i.toFixed(2);
            document.getElementById(`pid_${axis}_d`).value = d; document.getElementById(`val_${axis}_d`).innerText = d.toFixed(2);
        };
        setPID('r', data.pid_r[0], data.pid_r[1], data.pid_r[2]);
        setPID('p', data.pid_p[0], data.pid_p[1], data.pid_p[2]);
        setPID('y', data.pid_y[0], data.pid_y[1], data.pid_y[2]);
        
        pidInitialized = true;
    }
    
    // Update Chart
    const now = new Date().toLocaleTimeString();
    telemetryChart.data.labels.push(now);
    telemetryChart.data.datasets[0].data.push(data.r || 0);
    telemetryChart.data.datasets[1].data.push(data.p || 0);
    telemetryChart.data.datasets[2].data.push(data.y || 0);
    telemetryChart.data.datasets[3].data.push(data.a || 0);
    
    if (telemetryChart.data.labels.length > 50) {
        telemetryChart.data.labels.shift();
        telemetryChart.data.datasets.forEach(dataset => dataset.data.shift());
    }
    telemetryChart.update();
});

// PID Saving Function
function savePID(axis) {
    let p, i, d;
    if (axis === 'roll') {
        p = parseFloat(document.getElementById('pid_r_p').value);
        i = parseFloat(document.getElementById('pid_r_i').value);
        d = parseFloat(document.getElementById('pid_r_d').value);
    } else if (axis === 'pitch') {
        p = parseFloat(document.getElementById('pid_p_p').value);
        i = parseFloat(document.getElementById('pid_p_i').value);
        d = parseFloat(document.getElementById('pid_p_d').value);
    } else if (axis === 'yaw') {
        p = parseFloat(document.getElementById('pid_y_p').value);
        i = parseFloat(document.getElementById('pid_y_i').value);
        d = parseFloat(document.getElementById('pid_y_d').value);
    }
    
    socket.emit('tune_pid', { axis: axis, p: p, i: i, d: d });
    
    // Flash button green
    const btn = event.target;
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

