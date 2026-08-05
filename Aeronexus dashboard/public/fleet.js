// =============================================
// FLEET COMMAND CENTER — Main Logic
// Three.js 3D Drone Models + Fleet Management
// =============================================

// =============================================
// AUTHENTICATION
// =============================================
const fleetToken = localStorage.getItem('fleetToken');
if (!fleetToken) {
    window.location.href = '/login.html';
}

function fleetLogout() {
    localStorage.removeItem('fleetToken');
    window.location.href = '/login.html';
}

const socket = io({
    auth: { token: fleetToken }
});

// =============================================
// STATE
// =============================================

let fleetData = { drones: [], pending: [] };
let droneCardElements = {};  // droneId -> card DOM element
let drone3DScenes = {};      // droneId -> { scene, camera, renderer, drone }
let droneMapMarkers = {};    // droneId -> Leaflet marker
let currentView = 'grid';
const startTime = Date.now();

// =============================================
// BACKGROUND PARTICLES
// =============================================

function initParticles() {
    const canvas = document.getElementById('bg-particles');
    if (!canvas) return;
    const ctx = canvas.getContext('2d');

    function resize() {
        canvas.width = window.innerWidth;
        canvas.height = window.innerHeight;
    }
    resize();
    window.addEventListener('resize', resize);

    const particles = [];
    const count = 60;

    for (let i = 0; i < count; i++) {
        particles.push({
            x: Math.random() * canvas.width,
            y: Math.random() * canvas.height,
            vx: (Math.random() - 0.5) * 0.3,
            vy: (Math.random() - 0.5) * 0.3,
            size: Math.random() * 1.5 + 0.5,
            opacity: Math.random() * 0.3 + 0.05
        });
    }

    function animate() {
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        particles.forEach(p => {
            p.x += p.vx;
            p.y += p.vy;
            if (p.x < 0) p.x = canvas.width;
            if (p.x > canvas.width) p.x = 0;
            if (p.y < 0) p.y = canvas.height;
            if (p.y > canvas.height) p.y = 0;

            ctx.beginPath();
            ctx.arc(p.x, p.y, p.size, 0, Math.PI * 2);
            ctx.fillStyle = `rgba(14, 165, 233, ${p.opacity})`;
            ctx.fill();
        });

        // Draw connections between nearby particles
        for (let i = 0; i < particles.length; i++) {
            for (let j = i + 1; j < particles.length; j++) {
                const dx = particles[i].x - particles[j].x;
                const dy = particles[i].y - particles[j].y;
                const dist = Math.sqrt(dx * dx + dy * dy);
                if (dist < 150) {
                    ctx.beginPath();
                    ctx.moveTo(particles[i].x, particles[i].y);
                    ctx.lineTo(particles[j].x, particles[j].y);
                    ctx.strokeStyle = `rgba(14, 165, 233, ${0.03 * (1 - dist / 150)})`;
                    ctx.lineWidth = 0.5;
                    ctx.stroke();
                }
            }
        }

        requestAnimationFrame(animate);
    }
    animate();
}

// =============================================
// THREE.JS 3D DRONE MODEL BUILDER
// =============================================

function createDroneModel(color = 0x0ea5e9) {
    const group = new THREE.Group();

    // Center body - rounded box shape
    const bodyGeo = new THREE.BoxGeometry(0.8, 0.2, 0.8);
    const bodyMat = new THREE.MeshPhongMaterial({
        color: 0x1e293b,
        emissive: 0x0a0f1a,
        specular: 0x334155,
        shininess: 80
    });
    const body = new THREE.Mesh(bodyGeo, bodyMat);
    group.add(body);

    // Top dome
    const domeGeo = new THREE.SphereGeometry(0.3, 16, 8, 0, Math.PI * 2, 0, Math.PI / 2);
    const domeMat = new THREE.MeshPhongMaterial({
        color: 0x1e293b,
        emissive: 0x0f172a,
        specular: 0x475569,
        shininess: 100,
        transparent: true,
        opacity: 0.8
    });
    const dome = new THREE.Mesh(domeGeo, domeMat);
    dome.position.y = 0.1;
    group.add(dome);

    // LED strip on body
    const ledGeo = new THREE.BoxGeometry(0.82, 0.04, 0.82);
    const ledMat = new THREE.MeshPhongMaterial({
        color: color,
        emissive: color,
        emissiveIntensity: 0.5,
        transparent: true,
        opacity: 0.8
    });
    const led = new THREE.Mesh(ledGeo, ledMat);
    led.position.y = 0.02;
    group.add(led);

    // Arms + Motors + Propellers
    const armPositions = [
        { x: 0.7, z: 0.7 },
        { x: -0.7, z: 0.7 },
        { x: -0.7, z: -0.7 },
        { x: 0.7, z: -0.7 }
    ];

    const propellers = [];

    armPositions.forEach((pos, i) => {
        // Arm
        const armGeo = new THREE.BoxGeometry(1.0, 0.06, 0.08);
        const armMat = new THREE.MeshPhongMaterial({
            color: 0x334155,
            emissive: 0x0f172a,
            specular: 0x475569,
            shininess: 60
        });
        const arm = new THREE.Mesh(armGeo, armMat);
        arm.position.set(pos.x * 0.5, 0, pos.z * 0.5);
        arm.rotation.y = -Math.atan2(pos.z, pos.x);
        group.add(arm);

        // Motor housing
        const motorGeo = new THREE.CylinderGeometry(0.1, 0.12, 0.15, 12);
        const motorMat = new THREE.MeshPhongMaterial({
            color: 0x475569,
            specular: 0x94a3b8,
            shininess: 100
        });
        const motor = new THREE.Mesh(motorGeo, motorMat);
        motor.position.set(pos.x, 0.1, pos.z);
        group.add(motor);

        // Propeller disc (transparent)
        const propGeo = new THREE.CylinderGeometry(0.35, 0.35, 0.02, 32);
        const propMat = new THREE.MeshPhongMaterial({
            color: color,
            emissive: color,
            emissiveIntensity: 0.3,
            transparent: true,
            opacity: 0.3,
            side: THREE.DoubleSide
        });
        const prop = new THREE.Mesh(propGeo, propMat);
        prop.position.set(pos.x, 0.2, pos.z);
        group.add(prop);
        propellers.push(prop);

        // Prop blade crosses
        const bladeGeo = new THREE.BoxGeometry(0.6, 0.01, 0.04);
        const bladeMat = new THREE.MeshPhongMaterial({
            color: 0x64748b,
            specular: 0x94a3b8,
            shininess: 80
        });
        const blade1 = new THREE.Mesh(bladeGeo, bladeMat);
        blade1.position.set(pos.x, 0.2, pos.z);
        group.add(blade1);
        propellers.push(blade1);

        const blade2 = blade1.clone();
        blade2.rotation.y = Math.PI / 2;
        blade2.position.set(pos.x, 0.2, pos.z);
        group.add(blade2);
        propellers.push(blade2);
    });

    // Landing gear
    const legPositions = [
        { x: 0.3, z: 0.3 }, { x: -0.3, z: 0.3 },
        { x: -0.3, z: -0.3 }, { x: 0.3, z: -0.3 }
    ];

    legPositions.forEach(pos => {
        const legGeo = new THREE.CylinderGeometry(0.02, 0.02, 0.25, 6);
        const legMat = new THREE.MeshPhongMaterial({ color: 0x475569 });
        const leg = new THREE.Mesh(legGeo, legMat);
        leg.position.set(pos.x, -0.22, pos.z);
        group.add(leg);

        const footGeo = new THREE.SphereGeometry(0.035, 8, 4);
        const foot = new THREE.Mesh(footGeo, legMat);
        foot.position.set(pos.x, -0.35, pos.z);
        group.add(foot);
    });

    // Front indicator LED
    const frontLedGeo = new THREE.SphereGeometry(0.04, 8, 4);
    const frontLedMat = new THREE.MeshPhongMaterial({
        color: 0xff0000,
        emissive: 0xff0000,
        emissiveIntensity: 0.8
    });
    const frontLed = new THREE.Mesh(frontLedGeo, frontLedMat);
    frontLed.position.set(0, 0.05, 0.42);
    group.add(frontLed);

    return { group, propellers, led, frontLed };
}

function create3DScene(container, options = {}) {
    const width = container.clientWidth || 120;
    const height = container.clientHeight || 100;

    const scene = new THREE.Scene();

    const camera = new THREE.PerspectiveCamera(35, width / height, 0.1, 100);
    camera.position.set(2.5, 1.8, 2.5);
    camera.lookAt(0, 0, 0);

    const renderer = new THREE.WebGLRenderer({
        antialias: true,
        alpha: true
    });
    renderer.setSize(width, height);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setClearColor(0x000000, 0);
    container.appendChild(renderer.domElement);

    // Lighting
    const ambient = new THREE.AmbientLight(0x4488cc, 0.5);
    scene.add(ambient);

    const directional = new THREE.DirectionalLight(0xffffff, 0.8);
    directional.position.set(3, 5, 3);
    scene.add(directional);

    const pointLight = new THREE.PointLight(options.color || 0x0ea5e9, 0.6, 10);
    pointLight.position.set(0, 2, 0);
    scene.add(pointLight);

    // Subtle ground grid
    const gridHelper = new THREE.GridHelper(6, 12, 0x1e293b, 0x0f172a);
    gridHelper.position.y = -0.5;
    scene.add(gridHelper);

    const droneModel = createDroneModel(options.color || 0x0ea5e9);
    scene.add(droneModel.group);

    return {
        scene, camera, renderer, droneModel,
        width, height, pointLight
    };
}

function animate3DScene(sceneData, roll, pitch, yaw) {
    if (!sceneData || !sceneData.renderer) return;

    const { scene, camera, renderer, droneModel } = sceneData;

    // Apply attitude
    if (droneModel && droneModel.group) {
        const rollRad = (roll || 0) * Math.PI / 180;
        const pitchRad = (pitch || 0) * Math.PI / 180;

        droneModel.group.rotation.order = 'YXZ';
        droneModel.group.rotation.x = -pitchRad;
        droneModel.group.rotation.y = -((yaw || 0) + 180) * Math.PI / 180;
        droneModel.group.rotation.z = rollRad;

        // Spin propellers
        droneModel.propellers.forEach((prop, i) => {
            prop.rotation.y += (i % 2 === 0 ? 0.15 : -0.15);
        });
    }

    renderer.render(scene, camera);
}

// =============================================
// HEADER 3D DRONE (spinning logo)
// =============================================

let headerScene = null;

function initHeaderDrone() {
    const container = document.getElementById('header-drone-3d');
    if (!container) return;

    headerScene = create3DScene(container, { color: 0x0ea5e9 });

    function animateHeader() {
        if (headerScene) {
            headerScene.droneModel.group.rotation.y += 0.01;
            animate3DScene(headerScene, 0, 0, 0);
        }
        requestAnimationFrame(animateHeader);
    }
    animateHeader();
}

// =============================================
// EMPTY STATE 3D DRONE (floating animation)
// =============================================

let emptyScene = null;

function initEmptyDrone() {
    const container = document.getElementById('empty-3d');
    if (!container || container.children.length > 0) return;

    emptyScene = create3DScene(container, { color: 0x0ea5e9 });
    emptyScene.camera.position.set(3, 2.2, 3);
    emptyScene.camera.lookAt(0, 0, 0);

    let time = 0;
    function animateEmpty() {
        if (emptyScene) {
            time += 0.02;
            emptyScene.droneModel.group.rotation.y += 0.008;
            emptyScene.droneModel.group.position.y = Math.sin(time) * 0.15;
            animate3DScene(emptyScene, Math.sin(time * 0.7) * 3, Math.cos(time * 0.5) * 2, 0);
        }
        requestAnimationFrame(animateEmpty);
    }
    animateEmpty();
}

// =============================================
// FLEET MAP (Leaflet)
// =============================================

let fleetMap = null;
let darkTileLayer = null;

function initFleetMap() {
    fleetMap = L.map('fleet-map', {
        center: [7.8731, 80.7718],
        zoom: 5,
        zoomControl: true,
        attributionControl: false
    });

    darkTileLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        maxZoom: 19
    }).addTo(fleetMap);
}

function updateDroneMarker(droneId, lat, lon, armed) {
    if (!fleetMap) return;
    if (!lat || !lon || lat === 0 || lon === 0) return;

    const markerClass = armed ? 'fleet-map-marker armed' : 'fleet-map-marker';
    const labelClass = armed ? 'fleet-drone-label armed' : 'fleet-drone-label';

    const icon = L.divIcon({
        className: markerClass,
        iconSize: [18, 18],
        iconAnchor: [9, 9]
    });

    if (droneMapMarkers[droneId]) {
        droneMapMarkers[droneId].marker.setLatLng([lat, lon]);
        droneMapMarkers[droneId].marker.setIcon(icon);
    } else {
        const marker = L.marker([lat, lon], { icon: icon, zIndexOffset: 1000 }).addTo(fleetMap);
        marker.bindTooltip(droneId, {
            permanent: true,
            direction: 'top',
            offset: [0, -12],
            className: labelClass
        });
        droneMapMarkers[droneId] = { marker };
    }
}

function removeMapMarker(droneId) {
    if (droneMapMarkers[droneId]) {
        fleetMap.removeLayer(droneMapMarkers[droneId].marker);
        delete droneMapMarkers[droneId];
    }
}

function fitAllDrones() {
    const bounds = [];
    for (const key in droneMapMarkers) {
        bounds.push(droneMapMarkers[key].marker.getLatLng());
    }
    if (bounds.length > 0) {
        fleetMap.fitBounds(L.latLngBounds(bounds).pad(0.3));
    }
}

function toggleMapStyle() {
    // Toggle filter on/off for map tiles
    const pane = document.querySelector('#fleet-map .leaflet-tile-pane');
    if (pane) {
        if (pane.style.filter === 'none') {
            pane.style.filter = '';
        } else {
            pane.style.filter = 'none';
        }
    }
}

// =============================================
// DRONE CARD RENDERING
// =============================================

function renderDroneCard(drone) {
    const { droneId, connectionType, telemetry, online } = drone;
    const t = telemetry || {};

    const isArmed = t.arm === 1;
    const modeNames = { 0: 'STABILIZE', 1: 'ALT HOLD', 2: 'LOITER', 3: 'RTL', 4: 'AUTO' };
    const modeName = modeNames[t.md] || 'UNKNOWN';
    const battV = t.v !== undefined ? t.v.toFixed(1) : '--.-';
    const battClass = t.v !== undefined ? (t.v < 10.2 ? 'danger' : t.v < 10.8 ? 'warning' : 'good') : '';
    const altVal = t.a !== undefined ? t.a.toFixed(1) + 'm' : '--';
    const sigVal = t.sig !== undefined ? t.sig + '%' : '--';
    const sigClass = t.sig !== undefined ? (t.sig > 80 ? 'good' : t.sig > 40 ? 'warning' : 'danger') : '';
    const gpsFix = t.gf !== undefined ? (t.gf > 0 ? '3D FIX' : 'NO FIX') : '--';
    const gpsClass = t.gf > 0 ? 'good' : 'danger';
    const sats = t.gsat !== undefined ? t.gsat : '--';
    const heading = t.d !== undefined ? t.d.toFixed(0) + '°' : '--';

    // Signal bar levels
    const sigLevel = t.sig !== undefined ? Math.ceil(t.sig / 25) : 0;

    const cardClass = `drone-card${isArmed ? ' armed' : ''}${!online ? ' offline' : ''}`;
    const statusDotClass = `card-status-dot${isArmed ? ' armed' : ''}${!online ? ' offline' : ''}`;

    const html = `
        <div class="card-header">
            <div class="card-identity">
                <div class="${statusDotClass}"></div>
                <div>
                    <div class="card-drone-name">${droneId}</div>
                    <div class="card-conn-type">${connectionType} • ${online ? 'ONLINE' : 'OFFLINE'}</div>
                </div>
            </div>
            <div class="card-badges">
                <span class="card-badge mode-badge">${modeName}</span>
                <span class="card-badge ${isArmed ? 'armed-badge' : 'disarmed-badge'}">${isArmed ? 'ARMED' : 'SAFE'}</span>
            </div>
        </div>

        <div class="card-3d-section">
            <div class="card-3d-container" id="drone-3d-${CSS.escape(droneId)}"></div>
            <div class="card-telemetry-strip">
                <div class="telem-item">
                    <span class="telem-label">BATTERY</span>
                    <span class="telem-value ${battClass}">${battV}V</span>
                </div>
                <div class="telem-item">
                    <span class="telem-label">ALTITUDE</span>
                    <span class="telem-value">${altVal}</span>
                </div>
                <div class="telem-item">
                    <span class="telem-label">HEADING</span>
                    <span class="telem-value">${heading}</span>
                </div>
                <div class="telem-item">
                    <span class="telem-label">GPS</span>
                    <span class="telem-value ${gpsClass}">${gpsFix}</span>
                </div>
            </div>
        </div>

        <div class="card-status-bar">
            <div class="status-mini">
                <span class="status-mini-icon">📡</span>
                <span class="status-mini-text">${sigVal}</span>
                <div class="signal-bars">
                    <div class="signal-bar${sigLevel >= 1 ? ' active' : ''}"></div>
                    <div class="signal-bar${sigLevel >= 2 ? ' active' : ''}"></div>
                    <div class="signal-bar${sigLevel >= 3 ? ' active' : ''}"></div>
                    <div class="signal-bar${sigLevel >= 4 ? ' active' : ''}"></div>
                </div>
            </div>
            <div class="status-mini">
                <span class="status-mini-icon">🛰️</span>
                <span class="status-mini-text">${sats} SATS</span>
            </div>
            <div class="status-mini">
                <span class="status-mini-icon">⚡</span>
                <span class="status-mini-text">${t.t || '--'} THR</span>
            </div>
        </div>

        <div class="card-actions">
            <a href="/index.html?drone=${encodeURIComponent(droneId)}" target="_blank" class="card-btn btn-dashboard">
                🖥️ COMMAND
            </a>
            <a href="/survey.html?drone=${encodeURIComponent(droneId)}" target="_blank" class="card-btn btn-survey">
                📡 SURVEY
            </a>
            ${isArmed
                ? `<button class="card-btn btn-disarm" onclick="quickDisarm('${droneId}')">DISARM</button>`
                : `<button class="card-btn btn-arm" onclick="quickArm('${droneId}')">ARM</button>`
            }
            <button class="card-btn btn-remove" onclick="removeDrone('${droneId}')" title="Disconnect">✕</button>
        </div>
    `;

    return { html, cardClass };
}

function updateDroneGrid() {
    const grid = document.getElementById('drone-grid');
    const emptyState = document.getElementById('empty-state');

    if (fleetData.drones.length === 0) {
        grid.style.display = 'none';
        emptyState.style.display = 'flex';
        initEmptyDrone();
        return;
    }

    grid.style.display = '';
    emptyState.style.display = 'none';

    // Track which drone IDs we've seen
    const currentIds = new Set(fleetData.drones.map(d => d.droneId));

    // Remove cards for disconnected drones
    Object.keys(droneCardElements).forEach(id => {
        if (!currentIds.has(id)) {
            if (droneCardElements[id]) {
                droneCardElements[id].remove();
                delete droneCardElements[id];
            }
            if (drone3DScenes[id]) {
                drone3DScenes[id].renderer.dispose();
                delete drone3DScenes[id];
            }
            removeMapMarker(id);
        }
    });

    // Update or create cards
    fleetData.drones.forEach(drone => {
        const { html, cardClass } = renderDroneCard(drone);

        if (droneCardElements[drone.droneId]) {
            // Update existing card
            const card = droneCardElements[drone.droneId];
            card.className = cardClass;
            card.innerHTML = html;
            
            // Restore 3D canvas to the newly recreated container
            const containerId = `drone-3d-${CSS.escape(drone.droneId)}`;
            const container = card.querySelector(`#${containerId}`);
            if (container && drone3DScenes[drone.droneId]) {
                container.appendChild(drone3DScenes[drone.droneId].renderer.domElement);
            }
        } else {
            // Create new card
            const card = document.createElement('div');
            card.className = cardClass;
            card.innerHTML = html;
            card.dataset.droneId = drone.droneId;
            grid.appendChild(card);
            droneCardElements[drone.droneId] = card;
        }

        // Initialize 3D scene if container exists and scene doesn't
        const containerId = `drone-3d-${CSS.escape(drone.droneId)}`;
        const container = document.getElementById(containerId);
        if (container && !drone3DScenes[drone.droneId]) {
            // Small delay to let DOM settle
            setTimeout(() => {
                const c = document.getElementById(containerId);
                if (c && c.clientWidth > 0) {
                    const isArmed = drone.telemetry && drone.telemetry.arm === 1;
                    const color = isArmed ? 0xf43f5e : 0x0ea5e9;
                    drone3DScenes[drone.droneId] = create3DScene(c, { color });
                }
            }, 100);
        }

        // Update 3D model attitude
        if (drone3DScenes[drone.droneId] && drone.telemetry) {
            animate3DScene(
                drone3DScenes[drone.droneId],
                drone.telemetry.r || 0,
                drone.telemetry.p || 0,
                drone.telemetry.y || 0
            );
        }

        // Update map marker
        if (drone.telemetry) {
            updateDroneMarker(
                drone.droneId,
                drone.telemetry.glat,
                drone.telemetry.glon,
                drone.telemetry.arm === 1
            );
        }
    });
}

// =============================================
// PENDING CONNECTIONS
// =============================================

function updatePendingPanel() {
    const section = document.getElementById('pending-section');
    const grid = document.getElementById('pending-grid');

    if (fleetData.pending.length === 0) {
        section.style.display = 'none';
        grid.innerHTML = '';
        return;
    }

    section.style.display = '';

    const newIds = new Set(fleetData.pending.map(p => p.id));

    // Remove cards that no longer exist
    for (const el of Array.from(grid.children)) {
        if (!newIds.has(parseInt(el.dataset.id))) {
            grid.removeChild(el);
        }
    }

    // Add or update cards
    for (const p of fleetData.pending) {
        let card = grid.querySelector(`.pending-card[data-id="${p.id}"]`);
        
        const metaText = `${p.hasData ? '✅ Receiving telemetry' : '⏳ Waiting for data...'} ${p.previewVoltage ? ' • ' + p.previewVoltage.toFixed(1) + 'V' : ''}`;
        const ipText = `Connection #${p.id} ${p.remoteAddress ? '— ' + p.remoteAddress : ''}`;

        if (!card) {
            // Create new card
            card = document.createElement('div');
            card.className = 'pending-card';
            card.dataset.id = p.id;
            card.innerHTML = `
                <div class="pending-pulse"></div>
                <div class="pending-info">
                    <div class="pending-ip">${ipText}</div>
                    <div class="pending-meta">${metaText}</div>
                </div>
                <div class="pending-assign">
                    <input type="text" class="pending-input" id="pending-name-${p.id}" placeholder="Drone Name..." style="width: 120px;">
                    <input type="password" class="pending-input" id="pending-pin-${p.id}" placeholder="Access PIN" maxlength="8" style="width: 90px;" 
                           onkeypress="if(event.key==='Enter')assignDrone(${p.id})">
                    <button class="btn-assign" onclick="assignDrone(${p.id})">ASSIGN</button>
                </div>
            `;
            grid.appendChild(card);
        } else {
            // Update existing card's meta info without touching the inputs
            card.querySelector('.pending-meta').textContent = metaText;
            card.querySelector('.pending-ip').textContent = ipText;
        }
    }
}

// =============================================
// HEADER STATS
// =============================================

function updateHeaderStats() {
    const totalDrones = fleetData.drones.length;
    const armedCount = fleetData.drones.filter(d => d.telemetry && d.telemetry.arm === 1).length;
    const pendingCount = fleetData.pending.length;
    const alertCount = fleetData.drones.filter(d => d.telemetry && d.telemetry.v < 10.2).length;

    document.getElementById('total-drones').textContent = totalDrones;
    document.getElementById('armed-count').textContent = armedCount;
    document.getElementById('pending-count').textContent = pendingCount;
    document.getElementById('alert-count').textContent = alertCount;
    document.getElementById('map-drone-count').textContent = totalDrones;

    // GPS fixes count
    const gpsCount = fleetData.drones.filter(d => d.telemetry && d.telemetry.gf > 0).length;
    document.getElementById('map-gps-count').textContent = gpsCount;

    // Average altitude
    const alts = fleetData.drones
        .filter(d => d.telemetry && d.telemetry.a !== undefined)
        .map(d => d.telemetry.a);
    const avgAlt = alts.length > 0 ? (alts.reduce((a, b) => a + b, 0) / alts.length).toFixed(1) + 'm' : '--';
    document.getElementById('map-avg-alt').textContent = avgAlt;
}

// =============================================
// SOCKET EVENTS
// =============================================

socket.on('connect', () => {
    document.getElementById('system-status').innerHTML = `
        <div class="status-dot online"></div>
        <span>SYSTEM ONLINE</span>
    `;
});

socket.on('disconnect', () => {
    document.getElementById('system-status').innerHTML = `
        <div class="status-dot offline"></div>
        <span style="color: var(--rose);">SYSTEM OFFLINE</span>
    `;
});

socket.on('fleet:status', (data) => {
    fleetData = data;
    updateHeaderStats();
    updatePendingPanel();
    updateDroneGrid();
});

socket.on('drone:telemetry', (data) => {
    const { droneId, data: telemetry } = data;

    // Update fleet data in memory
    const drone = fleetData.drones.find(d => d.droneId === droneId);
    if (drone) {
        drone.telemetry = telemetry;
        drone.online = true;
    }

    // Update 3D model in real-time
    if (drone3DScenes[droneId]) {
        animate3DScene(drone3DScenes[droneId], telemetry.r, telemetry.p, telemetry.y);
    }

    // Update map marker
    if (telemetry.glat && telemetry.glon) {
        updateDroneMarker(droneId, telemetry.glat, telemetry.glon, telemetry.arm === 1);
    }

    // Update card telemetry values (efficient targeted update)
    updateCardTelemetry(droneId, telemetry);
});

socket.on('fleet:error', (data) => {
    alert('Fleet Error: ' + data.message);
});

// =============================================
// EFFICIENT CARD UPDATE (without full re-render)
// =============================================

function updateCardTelemetry(droneId, t) {
    const card = droneCardElements[droneId];
    if (!card) return;

    // Update telemetry values
    const telemValues = card.querySelectorAll('.telem-value');
    if (telemValues.length >= 4) {
        // Battery
        if (t.v !== undefined) {
            telemValues[0].textContent = t.v.toFixed(1) + 'V';
            telemValues[0].className = 'telem-value ' + (t.v < 10.2 ? 'danger' : t.v < 10.8 ? 'warning' : 'good');
        }
        // Altitude
        if (t.a !== undefined) {
            telemValues[1].textContent = t.a.toFixed(1) + 'm';
        }
        // Heading
        if (t.d !== undefined) {
            telemValues[2].textContent = t.d.toFixed(0) + '°';
        }
        // GPS
        if (t.gf !== undefined) {
            telemValues[3].textContent = t.gf > 0 ? '3D FIX' : 'NO FIX';
            telemValues[3].className = 'telem-value ' + (t.gf > 0 ? 'good' : 'danger');
        }
    }

    // Update armed state
    const isArmed = t.arm === 1;
    const statusDot = card.querySelector('.card-status-dot');
    if (statusDot) {
        statusDot.className = `card-status-dot${isArmed ? ' armed' : ''}`;
    }

    // Update card class
    card.className = `drone-card${isArmed ? ' armed' : ''}`;

    // Update badges
    const badges = card.querySelectorAll('.card-badge');
    if (badges.length >= 2) {
        const modeNames = { 0: 'STABILIZE', 1: 'ALT HOLD', 2: 'LOITER', 3: 'RTL', 4: 'AUTO' };
        if (t.md !== undefined) badges[0].textContent = modeNames[t.md] || 'UNKNOWN';
        badges[1].textContent = isArmed ? 'ARMED' : 'SAFE';
        badges[1].className = `card-badge ${isArmed ? 'armed-badge' : 'disarmed-badge'}`;
    }
}

// =============================================
// USER ACTIONS
// =============================================

function assignDrone(connectionId) {
    const nameInput = document.getElementById(`pending-name-${connectionId}`);
    const pinInput = document.getElementById(`pending-pin-${connectionId}`);
    if (!nameInput || !pinInput) return;

    const name = nameInput.value.trim();
    const pin = pinInput.value.trim();

    if (!name) {
        nameInput.style.borderColor = 'var(--rose)';
        nameInput.placeholder = 'Enter a name!';
        setTimeout(() => { nameInput.style.borderColor = ''; nameInput.placeholder = 'Drone name...'; }, 2000);
        return;
    }
    
    if (!pin) {
        pinInput.style.borderColor = 'var(--rose)';
        pinInput.placeholder = 'Required!';
        setTimeout(() => { pinInput.style.borderColor = ''; pinInput.placeholder = 'Access PIN'; }, 2000);
        return;
    }

    socket.emit('fleet:assign', {
        connectionIndex: connectionId,
        droneId: name,
        pin: pin,
        fleetToken: fleetToken
    });
}

function autoAssignAll() {
    const defaultPin = prompt("Enter a default Access PIN for auto-assigned drones:");
    if (defaultPin) {
        socket.emit('fleet:auto_assign', { prefix: 'Drone', pin: defaultPin, fleetToken: fleetToken });
    }
}

function removeDrone(droneId) {
    if (confirm(`Disconnect drone "${droneId}"? The ESP32 TCP connection will be closed.`)) {
        socket.emit('fleet:remove', { droneId, fleetToken });

        // Clean up local state
        if (drone3DScenes[droneId]) {
            drone3DScenes[droneId].renderer.dispose();
            delete drone3DScenes[droneId];
        }
        if (droneCardElements[droneId]) {
            droneCardElements[droneId].remove();
            delete droneCardElements[droneId];
        }
        removeMapMarker(droneId);
    }
}

function quickArm(droneId) {
    if (confirm(`⚠️ ARM drone "${droneId}"? Propellers may spin!`)) {
        socket.emit('drone:command', { droneId, type: 'toggle_arm', payload: { arm: 1 }, fleetToken: fleetToken });
    }
}

function quickDisarm(droneId) {
    socket.emit('drone:command', { droneId, type: 'toggle_arm', payload: { arm: 0 }, fleetToken: fleetToken });
}

function setView(mode) {
    currentView = mode;
    const grid = document.getElementById('drone-grid');
    document.getElementById('view-grid').className = `view-btn${mode === 'grid' ? ' active' : ''}`;
    document.getElementById('view-list').className = `view-btn${mode === 'list' ? ' active' : ''}`;

    if (mode === 'list') {
        grid.classList.add('list-view');
    } else {
        grid.classList.remove('list-view');
    }
}

// =============================================
// CLOCK & UPTIME
// =============================================

setInterval(() => {
    const now = new Date();
    document.getElementById('header-time').textContent =
        now.toLocaleTimeString('en-US', { hour12: false });

    const uptime = Math.floor((Date.now() - startTime) / 1000);
    const h = Math.floor(uptime / 3600);
    const m = Math.floor((uptime % 3600) / 60);
    const s = uptime % 60;
    document.getElementById('uptime').textContent =
        `Uptime: ${h > 0 ? h + 'h ' : ''}${m}m ${s}s`;
}, 1000);

// =============================================
// CONTINUOUS 3D ANIMATION LOOP
// =============================================

function globalAnimationLoop() {
    // Animate all drone 3D scenes
    for (const droneId in drone3DScenes) {
        const sceneData = drone3DScenes[droneId];
        if (sceneData && sceneData.droneModel) {
            // Keep propellers spinning
            sceneData.droneModel.propellers.forEach((prop, i) => {
                prop.rotation.y += (i % 2 === 0 ? 0.08 : -0.08);
            });
            sceneData.renderer.render(sceneData.scene, sceneData.camera);
        }
    }
    requestAnimationFrame(globalAnimationLoop);
}

// =============================================
// INITIALIZATION
// =============================================

document.addEventListener('DOMContentLoaded', () => {
    initParticles();
    initHeaderDrone();
    initFleetMap();
    globalAnimationLoop();

    // Fix Leaflet rendering
    setTimeout(() => {
        if (fleetMap) fleetMap.invalidateSize();
    }, 500);
});
