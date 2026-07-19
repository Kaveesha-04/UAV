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
        window.location.href = `/drone_login.html?drone=${encodeURIComponent(DRONE_ID)}&target=/survey.html`;
    }
}

const socket = io({
    auth: IS_FLEET_MODE ? { token: droneToken } : {}
});

// Setup drone-aware UI
if (IS_FLEET_MODE) {
    // Update command link to include drone param
    const cmdLink = document.getElementById('command-link');
    if (cmdLink) cmdLink.href = `/index.html?drone=${encodeURIComponent(DRONE_ID)}`;
    // Show drone name
    const nameEl = document.getElementById('survey-drone-name');
    if (nameEl) { nameEl.textContent = DRONE_ID; nameEl.style.display = 'inline-block'; }
    document.title = `${DRONE_ID} — Survey`;
}

// Command wrapper for multi-drone
function emitSurveyCommand(type, data) {
    if (IS_FLEET_MODE) {
        socket.emit('drone:command', { droneId: DRONE_ID, type, payload: data || {}, droneToken: droneToken });
    } else {
        socket.emit(type, data || {});
    }
}

// Handle Authentication Errors
socket.on('drone:auth_error', (data) => {
    if (data.droneId === DRONE_ID) {
        alert(data.message);
        localStorage.removeItem(`droneToken_${DRONE_ID}`);
        window.location.href = `/drone_login.html?drone=${encodeURIComponent(DRONE_ID)}&target=/survey.html`;
    }
});

// Map Initialization
const map = L.map('map', {
    zoomControl: false // Disable default zoom control for a cleaner look
}).setView([0, 0], 2);

L.control.zoom({
    position: 'topleft'
}).addTo(map);

// Satellite map tiles
const tileLayer = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
    attribution: 'Tiles &copy; Esri &mdash; Source: Esri, i-cubed, USDA, USGS, AEX, GeoEye, Getmapping, Aerogrid, IGN, IGP, UPR-EGP, and the GIS User Community',
    maxZoom: 24,
    maxNativeZoom: 18
}).addTo(map);

// Survey Layers
const heatLayer = L.heatLayer([], {
    radius: 35,
    blur: 20,
    maxZoom: 24,
    max: 1.0,
    gradient: {0.2: '#0ea5e9', 0.5: '#84cc16', 0.8: '#eab308', 1.0: '#ef4444'}
}).addTo(map);

// Gradient heatmap layer (separate from anomaly heatmap)
const gradientHeatLayer = L.heatLayer([], {
    radius: 30,
    blur: 25,
    maxZoom: 24,
    max: 1.0,
    gradient: {0.0: '#1e293b', 0.3: '#6366f1', 0.6: '#ec4899', 1.0: '#f43f5e'}
});

const surveyPathLayer = L.featureGroup().addTo(map);
const gridOverlayLayer = L.featureGroup(); // Survey grid coverage overlay

// Survey State
let isSurveyMode = false;
let magBaseline = 0;
let lastFitBoundsTime = 0;
let lastPlottedLat = 0;
let lastPlottedLon = 0;
let mapInitialized = false;
let surveyData = []; // Array of {lat, lon, val, valCorrected, gradient, hdop, speed, altitude, heading, timestamp}
let showGradientLayer = false;
let showGridOverlay = false;

// Magnetometer Rolling Average (client-side smoothing on top of firmware averaging)
const MAG_WINDOW_SIZE = 5;
let magRollingWindow = [];

// Auto-baseline State
let autoBaselineActive = false;
let autoBaselineSamples = [];
const AUTO_BASELINE_DURATION = 10000; // 10 seconds
let autoBaselineStartTime = 0;

// GPS Quality Thresholds
const HDOP_EXCELLENT = 1.5;
const HDOP_GOOD = 2.5;
const HDOP_POOR = 4.0;
const MIN_SATELLITES = 5;

// Survey Statistics
let surveyStartTime = 0;

// --- DIURNAL DRIFT CORRECTION ---
// Earth's magnetic field drifts ~30-50 nT/hour due to solar activity
// We track baseline drift over time and compensate
let diurnalHistory = []; // {time, mag} pairs for drift tracking
const DIURNAL_WINDOW = 60000; // Assess drift every 60 seconds
let diurnalCorrectionRate = 0; // nT per millisecond drift rate
let lastDiurnalUpdate = 0;
let diurnalEnabled = true;

// --- FOURTH-DIFFERENCE NOISE ESTIMATOR ---
// Standard geophysics QC metric: measures high-frequency sensor noise
// FD = |x[i-2] - 4*x[i-1] + 6*x[i] - 4*x[i+1] + x[i+2]| / 6.72
let fdNoiseBuffer = []; // Circular buffer of recent TMI values
const FD_BUFFER_SIZE = 20; // Need at least 5 for computation
let currentNoiseLevel = 0; // In µT

// --- SURVEY LINE DETECTION ---
// Detect straight flight lines vs. turns based on heading rate of change
let headingHistory = []; // Recent heading values
const HEADING_HISTORY_SIZE = 5;
const TURN_THRESHOLD = 8.0; // Degrees per update — above this = turning
let isInStraightLine = true;
let turnRejectCount = 0;

// --- ALTITUDE NORMALIZATION ---
// Magnetic field strength decreases with inverse cube of distance
// Normalize all readings to a reference altitude for consistency
let referenceAltitude = 0; // Set when survey starts (first valid reading)
let altNormEnabled = true;

// --- SURVEY GRID COVERAGE ---
// Divide area into cells and track which cells have been surveyed
let gridCells = {}; // key: "row,col" -> {count, avgVal}
const GRID_CELL_SIZE = 5.0; // meters per cell
let gridOriginLat = 0;
let gridOriginLon = 0;
let gridInitialized = false;

// =============================================
//  CONTROLS & UI FUNCTIONS
// =============================================

function toggleHeatmap() {
    isSurveyMode = document.getElementById('heatmap-toggle').checked;
    if (isSurveyMode) {
        // Dark room mode for survey
        map.removeLayer(tileLayer);
        document.getElementById('map').style.background = 'var(--bg-color)'; 
        lastPlottedLat = 0; 
        lastPlottedLon = 0;
        surveyData = [];
        magRollingWindow = [];
        diurnalHistory = [];
        fdNoiseBuffer = [];
        headingHistory = [];
        gridCells = {};
        gridInitialized = false;
        referenceAltitude = 0;
        diurnalCorrectionRate = 0;
        turnRejectCount = 0;
        surveyStartTime = Date.now();
        document.getElementById('survey-legend').style.display = 'flex';
        document.getElementById('survey-stats').style.display = 'flex';
        updateSurveyStats();
    } else {
        map.addLayer(tileLayer);
        document.getElementById('map').style.background = '';
        document.getElementById('survey-legend').style.display = 'none';
        document.getElementById('survey-stats').style.display = 'none';
        map.removeLayer(gradientHeatLayer);
        map.removeLayer(gridOverlayLayer);
        showGradientLayer = false;
        showGridOverlay = false;
    }
}

function setMagneticBaseline() {
    magBaseline = window.lastMagMagnitude || 0;
    if(magBaseline > 0) {
        document.getElementById('survey-baseline-val').innerText = `Baseline: ${magBaseline.toFixed(2)} µT`;
        document.getElementById('survey-baseline-val').style.display = 'inline';
    }
}

function startAutoBaseline() {
    autoBaselineActive = true;
    autoBaselineSamples = [];
    autoBaselineStartTime = Date.now();
    
    const btn = document.getElementById('btn-auto-baseline');
    btn.innerText = 'CALIBRATING...';
    btn.style.background = 'rgba(234, 179, 8, 0.3)';
    btn.style.color = '#eab308';
    btn.style.pointerEvents = 'none';
    
    // Progress indicator
    const progressInterval = setInterval(() => {
        if (!autoBaselineActive) {
            clearInterval(progressInterval);
            return;
        }
        const elapsed = Date.now() - autoBaselineStartTime;
        const pct = Math.min(100, (elapsed / AUTO_BASELINE_DURATION) * 100);
        btn.innerText = `CALIBRATING ${pct.toFixed(0)}%`;
    }, 200);
}

function completeAutoBaseline() {
    autoBaselineActive = false;
    if (autoBaselineSamples.length > 0) {
        // Use median instead of mean for robustness against outliers
        autoBaselineSamples.sort((a, b) => a - b);
        const mid = Math.floor(autoBaselineSamples.length / 2);
        magBaseline = autoBaselineSamples.length % 2 !== 0
            ? autoBaselineSamples[mid]
            : (autoBaselineSamples[mid - 1] + autoBaselineSamples[mid]) / 2;
        
        document.getElementById('survey-baseline-val').innerText = `Baseline: ${magBaseline.toFixed(2)} µT (auto, ${autoBaselineSamples.length} samples)`;
        document.getElementById('survey-baseline-val').style.display = 'inline';
    }
    
    const btn = document.getElementById('btn-auto-baseline');
    btn.innerText = 'AUTO BASELINE';
    btn.style.background = 'rgba(16, 185, 129, 0.15)';
    btn.style.color = 'var(--emerald)';
    btn.style.pointerEvents = 'auto';
}

// Toggle gradient heatmap overlay
function toggleGradientLayer() {
    showGradientLayer = !showGradientLayer;
    const btn = document.getElementById('btn-gradient');
    if (showGradientLayer) {
        map.addLayer(gradientHeatLayer);
        btn.style.background = '#6366f1';
        btn.style.color = '#fff';
    } else {
        map.removeLayer(gradientHeatLayer);
        btn.style.background = 'rgba(99, 102, 241, 0.15)';
        btn.style.color = '#6366f1';
    }
}

// Toggle grid coverage overlay
function toggleGridOverlay() {
    showGridOverlay = !showGridOverlay;
    const btn = document.getElementById('btn-grid');
    if (showGridOverlay) {
        renderGridOverlay();
        map.addLayer(gridOverlayLayer);
        btn.style.background = '#f59e0b';
        btn.style.color = '#fff';
    } else {
        map.removeLayer(gridOverlayLayer);
        btn.style.background = 'rgba(245, 158, 11, 0.15)';
        btn.style.color = '#f59e0b';
    }
}

// =============================================
//  GPS QUALITY
// =============================================

function getGPSQuality(hdop, satellites) {
    if (satellites < MIN_SATELLITES || hdop > HDOP_POOR) return { level: 'poor', color: '#ef4444', label: 'POOR' };
    if (hdop <= HDOP_EXCELLENT) return { level: 'excellent', color: '#10b981', label: 'EXCELLENT' };
    if (hdop <= HDOP_GOOD) return { level: 'good', color: '#84cc16', label: 'GOOD' };
    return { level: 'fair', color: '#eab308', label: 'FAIR' };
}

// =============================================
//  SURVEY LINE DETECTION
// =============================================

function detectSurveyLine(currentHeading) {
    headingHistory.push(currentHeading);
    if (headingHistory.length > HEADING_HISTORY_SIZE) {
        headingHistory.shift();
    }
    
    if (headingHistory.length < 3) {
        isInStraightLine = true; // Not enough data, assume straight
        return true;
    }
    
    // Compute heading rate of change (degrees per update)
    let totalChange = 0;
    for (let i = 1; i < headingHistory.length; i++) {
        let diff = headingHistory[i] - headingHistory[i - 1];
        // Normalize to -180..+180
        if (diff > 180) diff -= 360;
        if (diff < -180) diff += 360;
        totalChange += Math.abs(diff);
    }
    let avgChange = totalChange / (headingHistory.length - 1);
    
    isInStraightLine = avgChange < TURN_THRESHOLD;
    return isInStraightLine;
}

// =============================================
//  ALTITUDE NORMALIZATION (Inverse Cube Law)
// =============================================

function normalizeToReferenceAltitude(rawMag, currentAltitude) {
    if (!altNormEnabled || referenceAltitude <= 0 || currentAltitude <= 0) {
        return rawMag;
    }
    
    // Magnetic dipole field falls off as 1/r³
    // B_normalized = B_measured * (h_current / h_reference)³
    // This corrects for altitude variations during the survey
    let altRatio = currentAltitude / referenceAltitude;
    
    // Clamp to prevent extreme corrections from bad altitude data
    if (altRatio < 0.5) altRatio = 0.5;
    if (altRatio > 2.0) altRatio = 2.0;
    
    return rawMag * (altRatio * altRatio * altRatio);
}

// =============================================
//  DIURNAL DRIFT CORRECTION
// =============================================

function updateDiurnalCorrection(currentMag) {
    let now = Date.now();
    diurnalHistory.push({ time: now, mag: currentMag });
    
    // Keep only last 10 minutes of data for drift estimation
    while (diurnalHistory.length > 0 && (now - diurnalHistory[0].time) > 600000) {
        diurnalHistory.shift();
    }
    
    // Recalculate drift rate every DIURNAL_WINDOW
    if (now - lastDiurnalUpdate > DIURNAL_WINDOW && diurnalHistory.length >= 10) {
        lastDiurnalUpdate = now;
        
        // Linear regression on diurnal history to find drift rate
        let n = diurnalHistory.length;
        let sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
        let t0 = diurnalHistory[0].time;
        
        for (let p of diurnalHistory) {
            let x = (p.time - t0) / 1000.0; // seconds
            let y = p.mag;
            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumX2 += x * x;
        }
        
        let denom = (n * sumX2 - sumX * sumX);
        if (Math.abs(denom) > 0.001) {
            let slope = (n * sumXY - sumX * sumY) / denom; // µT per second
            diurnalCorrectionRate = slope; // Store as µT/second
        }
    }
}

function applyDiurnalCorrection(rawVariance) {
    if (!diurnalEnabled || diurnalCorrectionRate === 0 || surveyData.length === 0) {
        return rawVariance;
    }
    
    // Calculate elapsed time from first data point
    let elapsedSec = (Date.now() - surveyStartTime) / 1000.0;
    
    // Subtract the estimated drift
    let driftCorrection = diurnalCorrectionRate * elapsedSec;
    return rawVariance - driftCorrection;
}

// =============================================
//  FOURTH-DIFFERENCE NOISE ESTIMATOR
// =============================================

function updateFourthDifference(tmiValue) {
    fdNoiseBuffer.push(tmiValue);
    if (fdNoiseBuffer.length > FD_BUFFER_SIZE) {
        fdNoiseBuffer.shift();
    }
    
    if (fdNoiseBuffer.length < 5) return;
    
    // Compute Fourth Difference for all valid windows
    // FD[i] = x[i-2] - 4*x[i-1] + 6*x[i] - 4*x[i+1] + x[i+2]
    // Noise estimate = RMS of FD values / 6.72 (standard normalization factor)
    let fdValues = [];
    for (let i = 2; i < fdNoiseBuffer.length - 2; i++) {
        let fd = fdNoiseBuffer[i-2] 
               - 4 * fdNoiseBuffer[i-1] 
               + 6 * fdNoiseBuffer[i] 
               - 4 * fdNoiseBuffer[i+1] 
               + fdNoiseBuffer[i+2];
        fdValues.push(fd);
    }
    
    if (fdValues.length > 0) {
        let sumSq = fdValues.reduce((s, v) => s + v * v, 0);
        let rms = Math.sqrt(sumSq / fdValues.length);
        currentNoiseLevel = rms / 6.72; // Standard normalization factor
    }
}

function getNoiseQuality(noise) {
    // Noise thresholds based on survey-grade standards (adapted for hobby sensors)
    if (noise < 0.5) return { label: 'EXCELLENT', color: '#10b981' };
    if (noise < 1.5) return { label: 'GOOD', color: '#84cc16' };
    if (noise < 3.0) return { label: 'FAIR', color: '#eab308' };
    return { label: 'NOISY', color: '#ef4444' };
}

// =============================================
//  SPATIAL GRADIENT COMPUTATION
// =============================================

function computeGradient(currentPoint, previousPoint) {
    if (!previousPoint) return 0;
    
    // Calculate spatial distance in meters
    let distMeters = map.distance(
        [previousPoint.lat, previousPoint.lon], 
        [currentPoint.lat, currentPoint.lon]
    );
    
    if (distMeters < 0.5) return 0; // Too close, gradient meaningless
    
    // Gradient = change in anomaly / distance (µT/m)
    let dMag = Math.abs(currentPoint.val - previousPoint.val);
    return dMag / distMeters;
}

// =============================================
//  SURVEY GRID COVERAGE
// =============================================

function updateGridCoverage(lat, lon, val) {
    if (!gridInitialized) {
        gridOriginLat = lat;
        gridOriginLon = lon;
        gridInitialized = true;
    }
    
    // Convert lat/lon offset to meters
    let dLatM = (lat - gridOriginLat) * 111320;
    let dLonM = (lon - gridOriginLon) * 111320 * Math.cos(gridOriginLat * Math.PI / 180);
    
    // Calculate grid cell indices
    let row = Math.floor(dLatM / GRID_CELL_SIZE);
    let col = Math.floor(dLonM / GRID_CELL_SIZE);
    let key = `${row},${col}`;
    
    if (!gridCells[key]) {
        gridCells[key] = { count: 0, sumVal: 0, row: row, col: col };
    }
    gridCells[key].count++;
    gridCells[key].sumVal += val;
}

function getGridCoverage() {
    let cellCount = Object.keys(gridCells).length;
    
    // Calculate bounding box of cells
    if (cellCount === 0) return { cells: 0, coverage: 0 };
    
    let minRow = Infinity, maxRow = -Infinity;
    let minCol = Infinity, maxCol = -Infinity;
    
    for (let key in gridCells) {
        let c = gridCells[key];
        if (c.row < minRow) minRow = c.row;
        if (c.row > maxRow) maxRow = c.row;
        if (c.col < minCol) minCol = c.col;
        if (c.col > maxCol) maxCol = c.col;
    }
    
    let totalPossible = (maxRow - minRow + 1) * (maxCol - minCol + 1);
    let coverage = totalPossible > 0 ? (cellCount / totalPossible) * 100 : 0;
    
    return { cells: cellCount, coverage: Math.min(100, coverage) };
}

function renderGridOverlay() {
    gridOverlayLayer.clearLayers();
    
    if (!gridInitialized) return;
    
    let cosLat = Math.cos(gridOriginLat * Math.PI / 180);
    
    for (let key in gridCells) {
        let cell = gridCells[key];
        let avgVal = cell.sumVal / cell.count;
        
        // Convert cell indices back to lat/lon
        let lat1 = gridOriginLat + (cell.row * GRID_CELL_SIZE) / 111320;
        let lon1 = gridOriginLon + (cell.col * GRID_CELL_SIZE) / (111320 * cosLat);
        let lat2 = lat1 + GRID_CELL_SIZE / 111320;
        let lon2 = lon1 + GRID_CELL_SIZE / (111320 * cosLat);
        
        // Color by measurement density
        let alpha = Math.min(0.6, 0.15 + (cell.count / 10) * 0.45);
        let color = cell.count >= 3 ? '#10b981' : (cell.count >= 1 ? '#eab308' : '#ef4444');
        
        L.rectangle([[lat1, lon1], [lat2, lon2]], {
            color: color,
            fillColor: color,
            fillOpacity: alpha,
            weight: 1,
            opacity: 0.5
        }).addTo(gridOverlayLayer);
    }
}

// =============================================
//  SURVEY AREA & STATISTICS
// =============================================

function calculateSurveyArea(points) {
    if (points.length < 3) return 0;
    
    let pts = points.map(p => ({x: p.lon, y: p.lat}));
    pts.sort((a, b) => a.y - b.y || a.x - b.x);
    let pivot = pts[0];
    
    pts.slice(1).sort((a, b) => {
        let angleA = Math.atan2(a.y - pivot.y, a.x - pivot.x);
        let angleB = Math.atan2(b.y - pivot.y, b.x - pivot.x);
        return angleA - angleB;
    });
    
    let area = 0;
    let n = pts.length;
    for (let i = 0; i < n; i++) {
        let j = (i + 1) % n;
        area += pts[i].x * pts[j].y;
        area -= pts[j].x * pts[i].y;
    }
    area = Math.abs(area) / 2;
    
    let avgLat = points.reduce((s, p) => s + p.lat, 0) / points.length;
    let metersPerDegreeLat = 111320;
    let metersPerDegreeLon = 111320 * Math.cos(avgLat * Math.PI / 180);
    area = area * metersPerDegreeLat * metersPerDegreeLon;
    
    return area;
}

function updateSurveyStats() {
    const pointCount = surveyData.length;
    const area = calculateSurveyArea(surveyData);
    const elapsed = surveyStartTime > 0 ? Math.floor((Date.now() - surveyStartTime) / 1000) : 0;
    const minutes = Math.floor(elapsed / 60);
    const seconds = elapsed % 60;
    
    // Basic stats
    document.getElementById('stat-points').innerText = pointCount;
    document.getElementById('stat-area').innerText = area < 1000 
        ? area.toFixed(0) + ' m²' 
        : (area / 10000).toFixed(2) + ' ha';
    document.getElementById('stat-time').innerText = `${minutes}:${seconds.toString().padStart(2, '0')}`;
    
    // Noise level indicator
    let noiseEl = document.getElementById('stat-noise');
    if (noiseEl) {
        let nq = getNoiseQuality(currentNoiseLevel);
        noiseEl.innerText = currentNoiseLevel.toFixed(2) + ' µT';
        noiseEl.style.color = nq.color;
    }
    let noiseLabelEl = document.getElementById('stat-noise-label');
    if (noiseLabelEl) {
        let nq = getNoiseQuality(currentNoiseLevel);
        noiseLabelEl.innerText = `NOISE (${nq.label})`;
    }
    
    // Diurnal drift rate
    let driftEl = document.getElementById('stat-drift');
    if (driftEl) {
        let driftPerHour = diurnalCorrectionRate * 3600; // µT/hour
        driftEl.innerText = (driftPerHour >= 0 ? '+' : '') + driftPerHour.toFixed(3) + ' µT/hr';
    }
    
    // Survey line indicator
    let lineEl = document.getElementById('stat-line');
    if (lineEl) {
        lineEl.innerText = isInStraightLine ? 'SURVEY LINE' : 'TURNING';
        lineEl.style.color = isInStraightLine ? '#10b981' : '#ef4444';
    }
    
    // Grid coverage
    let gridInfo = getGridCoverage();
    let gridEl = document.getElementById('stat-grid');
    if (gridEl) {
        gridEl.innerText = `${gridInfo.cells} cells · ${gridInfo.coverage.toFixed(0)}%`;
    }
    
    // Turn reject counter
    let rejectEl = document.getElementById('stat-rejects');
    if (rejectEl) {
        rejectEl.innerText = turnRejectCount.toString();
    }
}

// =============================================
//  CSV EXPORT (Enhanced with all computed fields)
// =============================================

function exportSurveyCSV() {
    if (surveyData.length === 0) {
        alert('No survey data collected yet.');
        return;
    }
    
    let csv = 'Latitude,Longitude,Raw_Anomaly_uT,Corrected_Anomaly_uT,Gradient_uT_per_m,Altitude_m,HDOP,Speed_ms,Heading,Flight_Segment,Timestamp\n';
    for (let p of surveyData) {
        csv += `${p.lat.toFixed(7)},${p.lon.toFixed(7)},${p.val.toFixed(3)},${(p.valCorrected || p.val).toFixed(3)},${(p.gradient || 0).toFixed(4)},${(p.altitude || 0).toFixed(1)},${(p.hdop || 0).toFixed(1)},${(p.speed || 0).toFixed(1)},${(p.heading || 0).toFixed(1)},${p.lineSegment || 'unknown'},${p.timestamp || ''}\n`;
    }
    
    // Append survey metadata as comment rows
    csv += `\n# Survey Metadata\n`;
    csv += `# Baseline: ${magBaseline.toFixed(3)} µT\n`;
    csv += `# Reference Altitude: ${referenceAltitude.toFixed(1)} m\n`;
    csv += `# Diurnal Drift Rate: ${(diurnalCorrectionRate * 3600).toFixed(4)} µT/hr\n`;
    csv += `# Final Noise Level: ${currentNoiseLevel.toFixed(4)} µT\n`;
    csv += `# Total Points: ${surveyData.length}\n`;
    csv += `# Grid Cells Covered: ${getGridCoverage().cells}\n`;
    csv += `# Turn-Rejected Points: ${turnRejectCount}\n`;
    
    const blob = new Blob([csv], { type: 'text/csv' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `aeromagnetic_survey_${new Date().toISOString().slice(0,19).replace(/:/g,'-')}.csv`;
    a.click();
    URL.revokeObjectURL(url);
}

// =============================================
//  MAIN TELEMETRY HANDLER
// =============================================

// Subscribe to correct telemetry event
function registerTelemetryHandler(handler) {
    if (IS_FLEET_MODE) {
        socket.on('drone:telemetry', (msg) => {
            if (msg.droneId === DRONE_ID) handler(msg.data);
        });
    } else {
        socket.on('telemetry', handler);
    }
}

registerTelemetryHandler((data) => {
    // Initial centering
    if (!mapInitialized && data.glat !== 0 && data.glon !== 0) {
        map.setView([data.glat, data.glon], 18);
        mapInitialized = true;
    }

    // GPS Quality Indicator (always update, even outside survey mode)
    let currentHdop = data.ghdop !== undefined ? data.ghdop : 99;
    let currentSats = data.gsat !== undefined ? data.gsat : 0;
    let currentSpeed = data.gspd !== undefined ? data.gspd : 0;
    let currentAlt = data.a !== undefined ? data.a : 0;
    let currentHeading = data.d !== undefined ? data.d : 0;
    
    let gpsQuality = getGPSQuality(currentHdop, currentSats);
    let qualityBadge = document.getElementById('gps-quality-badge');
    if (qualityBadge) {
        qualityBadge.innerText = `${gpsQuality.label} · ${currentSats} SAT · HDOP ${currentHdop.toFixed(1)}`;
        qualityBadge.style.color = gpsQuality.color;
        qualityBadge.style.borderColor = gpsQuality.color + '66';
    }

    // Compute Magnetic Magnitude using averaged mag readings when available
    let useMx, useMy, useMz;
    if (data.amx !== undefined && data.amy !== undefined && data.amz !== undefined) {
        useMx = data.amx;
        useMy = data.amy;
        useMz = data.amz;
    } else if (data.mx !== undefined && data.my !== undefined && data.mz !== undefined) {
        useMx = data.mx;
        useMy = data.my;
        useMz = data.mz;
    }

    if (useMx !== undefined) {
        let mx = useMx;
        let my = useMy;
        let mz = useMz;
        
        // Convert raw LSB to true Microteslas (µT) based on sensor hardware
        let lsb_per_gauss = 1090.0;
        if (data.mt !== undefined) {
            if (data.mt === 13) lsb_per_gauss = 3000.0;
            else if (data.mt === 30) lsb_per_gauss = 1090.0;
            else if (data.mt === 44) lsb_per_gauss = 3000.0;
        }

        mx = (mx / lsb_per_gauss) * 100.0;
        my = (my / lsb_per_gauss) * 100.0;
        mz = (mz / lsb_per_gauss) * 100.0;

        let instantMag = Math.sqrt(mx*mx + my*my + mz*mz);
        
        // Client-side rolling average on top of firmware averaging
        magRollingWindow.push(instantMag);
        if (magRollingWindow.length > MAG_WINDOW_SIZE) {
            magRollingWindow.shift();
        }
        
        window.lastMagMagnitude = magRollingWindow.reduce((a, b) => a + b, 0) / magRollingWindow.length;
        
        // Update Fourth-Difference noise estimator
        updateFourthDifference(window.lastMagMagnitude);
    }

    // Update live mag display
    let magDisplay = document.getElementById('stat-mag');
    if (magDisplay && window.lastMagMagnitude) {
        magDisplay.innerText = window.lastMagMagnitude.toFixed(1) + ' µT';
    }

    // Auto-baseline collection
    if (autoBaselineActive && window.lastMagMagnitude) {
        autoBaselineSamples.push(window.lastMagMagnitude);
        if (Date.now() - autoBaselineStartTime >= AUTO_BASELINE_DURATION) {
            completeAutoBaseline();
        }
    }

    // ============================
    //  AEROMAGNETIC SURVEY LOGIC
    // ============================
    if (isSurveyMode && magBaseline > 0 && data.gf > 0) {
        
        // Use EMA-smoothed coordinates for survey (firmware-filtered)
        let surveyLat = (data.slat !== undefined && data.slat !== 0) ? data.slat : data.glat;
        let surveyLon = (data.slon !== undefined && data.slon !== 0) ? data.slon : data.glon;
        
        if (surveyLat === 0 || surveyLon === 0) return;
        
        // --- GPS Quality Gating ---
        if (currentHdop > HDOP_GOOD) return;
        if (currentSats < MIN_SATELLITES) return;
        
        // --- Speed-based filtering ---
        if (currentSpeed < 0.3 && surveyData.length > 0) return;
        if (currentSpeed > 15.0) return;
        
        // --- Survey Line Detection ---
        let straightLine = detectSurveyLine(currentHeading);
        if (!straightLine) {
            turnRejectCount++;
            // Still update stats to show turn indicator
            updateSurveyStats();
            return; // Reject data collected during turns
        }
        
        // --- Set reference altitude on first valid reading ---
        if (referenceAltitude <= 0 && currentAlt > 0.5) {
            referenceAltitude = currentAlt;
        }
        
        // --- Adaptive Distance Threshold ---
        let minDistance = Math.max(1.5, 2.0 * currentHdop);
        
        let distance = 100;
        if (lastPlottedLat !== 0 && lastPlottedLon !== 0) {
            distance = map.distance([lastPlottedLat, lastPlottedLon], [surveyLat, surveyLon]);
        }
        
        if (distance > minDistance) {
            lastPlottedLat = surveyLat;
            lastPlottedLon = surveyLon;

            let currentMag = window.lastMagMagnitude;
            
            // --- Altitude Normalization (Inverse Cube Law) ---
            let normalizedMag = normalizeToReferenceAltitude(currentMag, currentAlt);
            
            // --- Diurnal Drift Tracking ---
            updateDiurnalCorrection(normalizedMag);
            
            // --- Compute Raw and Corrected Anomaly ---
            let rawVariance = Math.abs(normalizedMag - magBaseline);
            let correctedVariance = Math.abs(applyDiurnalCorrection(rawVariance));
            
            // Use corrected variance for mapping
            let variance = correctedVariance;
            
            // --- Compute Spatial Gradient ---
            let previousPoint = surveyData.length > 0 ? surveyData[surveyData.length - 1] : null;
            let gradient = 0;
            
            let newPoint = {
                lat: surveyLat, 
                lon: surveyLon, 
                val: rawVariance,
                valCorrected: correctedVariance
            };
            gradient = computeGradient(newPoint, previousPoint);
            
            // Map variance to intensity (0.0 to 1.0)
            let intensity = Math.min(1.0, variance / 50.0);
            
            if (variance > 5) { 
                heatLayer.addLatLng([surveyLat, surveyLon, intensity]);
            }
            
            // Add gradient data to the gradient heatmap
            if (gradient > 0.1) { // Only plot meaningful gradients
                let gradIntensity = Math.min(1.0, gradient / 5.0); // 5 µT/m = max
                gradientHeatLayer.addLatLng([surveyLat, surveyLon, gradIntensity]);
            }
            
            // Record data with all computed fields
            surveyData.push({
                lat: surveyLat, 
                lon: surveyLon, 
                val: rawVariance,
                valCorrected: correctedVariance,
                gradient: gradient,
                altitude: currentAlt,
                hdop: currentHdop,
                speed: currentSpeed,
                heading: currentHeading,
                lineSegment: 'straight',
                timestamp: new Date().toISOString()
            });
            
            // Update grid coverage
            updateGridCoverage(surveyLat, surveyLon, variance);
            
            // Survey path marker
            L.circleMarker([surveyLat, surveyLon], {
                radius: 2,
                color: '#0ea5e9',
                fillColor: '#0ea5e9',
                fillOpacity: 1,
                stroke: false
            }).addTo(surveyPathLayer);
            
            // Auto-fit bounds
            let now = Date.now();
            if (now - lastFitBoundsTime > 1000) {
                if (surveyPathLayer.getLayers().length > 1) {
                    map.fitBounds(surveyPathLayer.getBounds(), { padding: [50, 50], maxZoom: 24, animate: false });
                }
                lastFitBoundsTime = now;
            }
            
            // Update live grid overlay if visible
            if (showGridOverlay && surveyData.length % 5 === 0) {
                renderGridOverlay();
            }
            
            // Update stats
            updateSurveyStats();
        }
    }

    // ============================
    //  AUTO SURVEY MISSION STATE
    // ============================
    if (data.sst !== undefined) {
        updateMissionProgress(data.swp, data.swt, data.sst);
    }
});

// =============================================
//  2D/3D MAP GENERATION (Quality-Weighted IDW)
// =============================================

function generateMap(mapType) {
    if (surveyData.length < 3) {
        alert("Not enough survey data collected yet. Please collect at least 3 points.");
        return;
    }

    document.getElementById('contour-container').style.display = 'block';
    document.getElementById('btn-generate-2d').style.display = 'none';
    document.getElementById('btn-generate-3d').style.display = 'none';
    document.getElementById('btn-close-map').style.display = 'block';
    document.getElementById('survey-legend').style.display = 'none';

    let minLat = 90, maxLat = -90, minLon = 180, maxLon = -180;
    for (let p of surveyData) {
        if (p.lat < minLat) minLat = p.lat;
        if (p.lat > maxLat) maxLat = p.lat;
        if (p.lon < minLon) minLon = p.lon;
        if (p.lon > maxLon) maxLon = p.lon;
    }

    let latPad = Math.max((maxLat - minLat) * 0.1, 0.0001);
    let lonPad = Math.max((maxLon - minLon) * 0.1, 0.0001);
    minLat -= latPad; maxLat += latPad;
    minLon -= lonPad; maxLon += lonPad;

    let resolution = 50;
    let gridZ = [];
    let gridX = [];
    let gridY = [];

    for(let i=0; i<resolution; i++) {
        gridX.push(minLon + (i/(resolution-1)) * (maxLon - minLon));
        gridY.push(minLat + (i/(resolution-1)) * (maxLat - minLat));
    }

    // Quality-Weighted IDW using corrected anomaly values
    for (let yi = 0; yi < resolution; yi++) {
        let row = [];
        let y = gridY[yi];
        for (let xi = 0; xi < resolution; xi++) {
            let x = gridX[xi];
            
            let numerator = 0;
            let denominator = 0;
            
            for (let p of surveyData) {
                let dx = x - p.lon;
                let dy = y - p.lat;
                let d2 = dx*dx + dy*dy;
                
                if (d2 < 0.000000001) d2 = 0.000000001;
                
                let qualityWeight = 1.0 / Math.max(0.5, p.hdop || 1.0);
                
                let weight = qualityWeight / d2;
                // Use diurnal-corrected + altitude-normalized values
                numerator += weight * (p.valCorrected || p.val);
                denominator += weight;
            }
            row.push(numerator / denominator);
        }
        gridZ.push(row);
    }

    let data = [{
        z: gridZ,
        x: gridX,
        y: gridY,
        type: mapType,
        colorscale: [
            ['0.0', 'rgb(15, 23, 42)'],
            ['0.2', 'rgb(14, 165, 233)'],
            ['0.5', 'rgb(132, 204, 22)'],
            ['0.8', 'rgb(234, 179, 8)'],
            ['1.0', 'rgb(239, 68, 68)']
        ],
        showscale: true,
        zsmooth: mapType === 'heatmap' ? false : undefined
    }];

    let layout = {
        title: { text: 'Aeromagnetic Survey (Altitude-Normalized, Drift-Corrected, Quality-Weighted IDW)', font: {color: '#f8fafc', size: 20} },
        paper_bgcolor: '#0f172a',
        plot_bgcolor: '#0f172a',
        xaxis: { showgrid: false, zeroline: false, showticklabels: false },
        yaxis: { showgrid: false, zeroline: false, showticklabels: false, scaleanchor: 'x', scaleratio: 1 },
        margin: { t: 80, b: 20, l: 20, r: 20 }
    };

    Plotly.newPlot('contour-plot', data, layout);
}

function closeContourMap() {
    document.getElementById('contour-container').style.display = 'none';
    document.getElementById('btn-generate-2d').style.display = 'block';
    document.getElementById('btn-generate-3d').style.display = 'block';
    document.getElementById('btn-close-map').style.display = 'none';
    if (isSurveyMode) {
        document.getElementById('survey-legend').style.display = 'flex';
    }
}

// =============================================
//  LAWNMOWER SURVEY MISSION PLANNER
// =============================================

// Planning state
window._planningActive = false;
let planCorners = []; // Two corner markers
let planRectangle = null; // Leaflet rectangle
let planWaypoints = []; // Generated lawnmower waypoints
let planWaypointMarkers = L.featureGroup(); // Visual markers on map
let planPathLine = null; // Polyline showing flight path
let missionDroneMarker = null; // Live drone position during mission

planWaypointMarkers.addTo(map);

// --- AREA SELECTION ---
function startAreaSelection() {
    if (window._planningActive) {
        // Cancel planning mode
        cancelAreaSelection();
        return;
    }
    
    window._planningActive = true;
    planCorners = [];
    clearPlanVisuals();
    
    const btn = document.getElementById('btn-plan-area');
    btn.innerText = '📐 CLICK CORNER 1';
    btn.style.background = '#fb923c';
    btn.style.color = '#fff';
    
    map.getContainer().style.cursor = 'crosshair';
    map.on('click', onPlanClick);
}

function cancelAreaSelection() {
    window._planningActive = false;
    map.off('click', onPlanClick);
    map.getContainer().style.cursor = '';
    
    const btn = document.getElementById('btn-plan-area');
    btn.innerText = '📐 PLAN AREA';
    btn.style.background = 'rgba(251, 146, 60, 0.15)';
    btn.style.color = '#fb923c';
}

function onPlanClick(e) {
    planCorners.push(e.latlng);
    
    // Add corner marker
    L.circleMarker(e.latlng, {
        radius: 6,
        color: '#fb923c',
        fillColor: '#fb923c',
        fillOpacity: 1,
        weight: 2
    }).addTo(planWaypointMarkers);
    
    if (planCorners.length === 1) {
        const btn = document.getElementById('btn-plan-area');
        btn.innerText = '📐 CLICK CORNER 2';
    }
    
    if (planCorners.length === 2) {
        // Done selecting - generate the plan
        map.off('click', onPlanClick);
        map.getContainer().style.cursor = '';
        window._planningActive = false;
        
        const btn = document.getElementById('btn-plan-area');
        btn.innerText = '📐 PLAN AREA';
        btn.style.background = 'rgba(251, 146, 60, 0.15)';
        btn.style.color = '#fb923c';
        
        generateLawnmowerPattern();
    }
}

// --- LAWNMOWER PATTERN GENERATOR ---
function generateLawnmowerPattern() {
    const spacingMeters = parseFloat(document.getElementById('line-spacing').value) || 5;
    
    const c1 = planCorners[0];
    const c2 = planCorners[1];
    
    // Get bounding box
    const minLat = Math.min(c1.lat, c2.lat);
    const maxLat = Math.max(c1.lat, c2.lat);
    const minLon = Math.min(c1.lng, c2.lng);
    const maxLon = Math.max(c1.lng, c2.lng);
    
    // Draw the survey rectangle
    if (planRectangle) map.removeLayer(planRectangle);
    planRectangle = L.rectangle([[minLat, minLon], [maxLat, maxLon]], {
        color: '#fb923c',
        weight: 2,
        fillColor: '#fb923c',
        fillOpacity: 0.08,
        dashArray: '8, 4'
    }).addTo(map);
    
    // Convert spacing from meters to approximate degrees
    // 1 degree latitude ≈ 111,320 meters
    const latPerMeter = 1.0 / 111320.0;
    const lonPerMeter = 1.0 / (111320.0 * Math.cos((minLat + maxLat) / 2.0 * Math.PI / 180.0));
    
    const latSpacing = spacingMeters * latPerMeter;
    
    // Generate boustrophedon (lawnmower) waypoints
    // Fly east-west lines, stepping north by spacingMeters each line
    planWaypoints = [];
    let lineIndex = 0;
    
    for (let lat = minLat; lat <= maxLat; lat += latSpacing) {
        if (lineIndex % 2 === 0) {
            // Fly west to east
            planWaypoints.push({ lat: lat, lon: minLon });
            planWaypoints.push({ lat: lat, lon: maxLon });
        } else {
            // Fly east to west (reverse)
            planWaypoints.push({ lat: lat, lon: maxLon });
            planWaypoints.push({ lat: lat, lon: minLon });
        }
        lineIndex++;
    }
    
    // Limit to 50 waypoints (STM32 queue size)
    if (planWaypoints.length > 50) {
        alert(`⚠️ Too many waypoints (${planWaypoints.length}). Maximum is 50.\nIncrease line spacing or reduce the area.`);
        planWaypoints = planWaypoints.slice(0, 50);
    }
    
    // Draw waypoints and path on map
    clearPlanVisuals();
    
    const pathCoords = planWaypoints.map(wp => [wp.lat, wp.lon]);
    planPathLine = L.polyline(pathCoords, {
        color: '#10b981',
        weight: 2,
        opacity: 0.7,
        dashArray: '6, 4'
    }).addTo(map);
    
    planWaypoints.forEach((wp, i) => {
        const isCorner = (i === 0 || i === planWaypoints.length - 1);
        L.circleMarker([wp.lat, wp.lon], {
            radius: isCorner ? 5 : 3,
            color: isCorner ? '#10b981' : '#0ea5e9',
            fillColor: isCorner ? '#10b981' : '#0ea5e9',
            fillOpacity: 1,
            weight: 1
        }).bindTooltip(`WP ${i + 1}`, {
            permanent: false,
            direction: 'top',
            className: 'wp-tooltip'
        }).addTo(planWaypointMarkers);
    });
    
    // Fit map to show the plan
    map.fitBounds(planRectangle.getBounds(), { padding: [60, 60] });
    
    // Show mission controls
    document.getElementById('btn-start-mission').style.display = 'inline-block';
    document.getElementById('btn-clear-plan').style.display = 'inline-block';
    document.getElementById('mission-progress').style.display = 'block';
    
    // Update counter
    document.getElementById('mission-wp-counter').innerText = `0 / ${planWaypoints.length}`;
    document.getElementById('mission-state-badge').innerText = 'PLANNED';
    document.getElementById('mission-state-badge').style.background = 'rgba(251, 146, 60, 0.3)';
    document.getElementById('mission-state-badge').style.color = '#fb923c';
}

function clearPlanVisuals() {
    planWaypointMarkers.clearLayers();
    if (planPathLine) { map.removeLayer(planPathLine); planPathLine = null; }
    if (planRectangle) { map.removeLayer(planRectangle); planRectangle = null; }
    if (missionDroneMarker) { map.removeLayer(missionDroneMarker); missionDroneMarker = null; }
}

// --- MISSION CONTROL ---
function startSurveyMission() {
    if (planWaypoints.length === 0) {
        alert('No waypoints planned! Use PLAN AREA to draw a survey rectangle first.');
        return;
    }
    
    if (!confirm(
        `🚁 START SURVEY MISSION?\n\n` +
        `Waypoints: ${planWaypoints.length}\n` +
        `Line Spacing: ${document.getElementById('line-spacing').value}m\n\n` +
        `The drone must be ARMED and airborne.\n` +
        `It will fly the lawnmower pattern at current altitude.\n` +
        `RC remote can override at any time.`
    )) return;
    
    // 1. Reset the drone's waypoint queue
    emitSurveyCommand('survey_reset');
    
    // 2. Send all waypoints (with small delay between each)
    let sent = 0;
    const sendNext = () => {
        if (sent < planWaypoints.length) {
            const wp = planWaypoints[sent];
            emitSurveyCommand('survey_waypoint', { lat: wp.lat.toFixed(6), lon: wp.lon.toFixed(6) });
            sent++;
            document.getElementById('mission-state-badge').innerText = `UPLOADING ${sent}/${planWaypoints.length}`;
            setTimeout(sendNext, 50); // 50ms between commands to prevent USB buffer overflow
        } else {
            // 3. All waypoints sent — start the mission!
            setTimeout(() => {
                emitSurveyCommand('survey_start');
                
                document.getElementById('btn-start-mission').style.display = 'none';
                document.getElementById('btn-clear-plan').style.display = 'none';
                document.getElementById('btn-pause-mission').style.display = 'inline-block';
                document.getElementById('btn-abort-mission').style.display = 'inline-block';
            }, 200);
        }
    };
    sendNext();
}

function pauseSurveyMission() {
    emitSurveyCommand('survey_pause');
    document.getElementById('btn-pause-mission').style.display = 'none';
    document.getElementById('btn-resume-mission').style.display = 'inline-block';
}

function resumeSurveyMission() {
    emitSurveyCommand('survey_resume');
    document.getElementById('btn-resume-mission').style.display = 'none';
    document.getElementById('btn-pause-mission').style.display = 'inline-block';
}

function abortSurveyMission() {
    if (!confirm('⚠️ ABORT the survey mission? The drone will hold position (LOITER).')) return;
    emitSurveyCommand('survey_abort');
    
    // Reset UI
    document.getElementById('btn-pause-mission').style.display = 'none';
    document.getElementById('btn-resume-mission').style.display = 'none';
    document.getElementById('btn-abort-mission').style.display = 'none';
    document.getElementById('btn-start-mission').style.display = 'inline-block';
    document.getElementById('btn-clear-plan').style.display = 'inline-block';
}

function clearSurveyPlan() {
    planWaypoints = [];
    planCorners = [];
    clearPlanVisuals();
    
    document.getElementById('btn-start-mission').style.display = 'none';
    document.getElementById('btn-clear-plan').style.display = 'none';
    document.getElementById('btn-pause-mission').style.display = 'none';
    document.getElementById('btn-resume-mission').style.display = 'none';
    document.getElementById('btn-abort-mission').style.display = 'none';
    document.getElementById('mission-progress').style.display = 'none';
}

// --- MISSION PROGRESS TRACKING (from telemetry) ---
function updateMissionProgress(wpIndex, wpTotal, state) {
    const progressPanel = document.getElementById('mission-progress');
    const stateBadge = document.getElementById('mission-state-badge');
    const wpCounter = document.getElementById('mission-wp-counter');
    const progressBar = document.getElementById('mission-progress-bar');
    
    if (wpTotal === 0 && state === 0) {
        // No mission active and nothing planned locally — hide panel
        if (planWaypoints.length === 0) {
            progressPanel.style.display = 'none';
        }
        return;
    }
    
    progressPanel.style.display = 'block';
    wpCounter.innerText = `${wpIndex + (state === 3 ? 0 : 1)} / ${wpTotal}`;
    
    const pct = wpTotal > 0 ? ((wpIndex) / wpTotal) * 100 : 0;
    progressBar.style.width = (state === 3 ? 100 : pct) + '%';
    
    const stateMap = {
        0: { text: 'IDLE',    bg: 'rgba(100,116,139,0.3)', color: '#94a3b8' },
        1: { text: 'FLYING',  bg: 'rgba(16,185,129,0.3)',  color: '#10b981' },
        2: { text: 'PAUSED',  bg: 'rgba(245,158,11,0.3)',  color: '#f59e0b' },
        3: { text: 'DONE ✓',  bg: 'rgba(14,165,233,0.3)',  color: '#0ea5e9' }
    };
    
    const s = stateMap[state] || stateMap[0];
    stateBadge.innerText = s.text;
    stateBadge.style.background = s.bg;
    stateBadge.style.color = s.color;
    
    // When mission completes, show clear button
    if (state === 3) {
        document.getElementById('btn-pause-mission').style.display = 'none';
        document.getElementById('btn-resume-mission').style.display = 'none';
        document.getElementById('btn-abort-mission').style.display = 'none';
        document.getElementById('btn-clear-plan').style.display = 'inline-block';
        progressBar.style.background = 'linear-gradient(90deg, #0ea5e9, #10b981)';
    }
    
    // Update button visibility based on state
    if (state === 1) {
        document.getElementById('btn-start-mission').style.display = 'none';
        document.getElementById('btn-clear-plan').style.display = 'none';
        document.getElementById('btn-pause-mission').style.display = 'inline-block';
        document.getElementById('btn-resume-mission').style.display = 'none';
        document.getElementById('btn-abort-mission').style.display = 'inline-block';
    } else if (state === 2) {
        document.getElementById('btn-pause-mission').style.display = 'none';
        document.getElementById('btn-resume-mission').style.display = 'inline-block';
    }
}
