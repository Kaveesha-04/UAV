const socket = io();

// Map Initialization
const map = L.map('map', {
    zoomControl: false // Disable default zoom control for a cleaner look
}).setView([0, 0], 2);

L.control.zoom({
    position: 'bottomright'
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

const surveyPathLayer = L.featureGroup().addTo(map);

// Survey State
let isSurveyMode = false;
let magBaseline = 0;
let lastFitBoundsTime = 0;
let lastPlottedLat = 0;
let lastPlottedLon = 0;
let mapInitialized = false;
let surveyData = []; // Array of {lat, lon, val} to store all points for 2D map

// Controls
function toggleHeatmap() {
    isSurveyMode = document.getElementById('heatmap-toggle').checked;
    if (isSurveyMode) {
        // Dark room mode for survey
        map.removeLayer(tileLayer);
        document.getElementById('map').style.background = 'var(--bg-color)'; 
        lastPlottedLat = 0; 
        lastPlottedLon = 0;
        surveyData = []; // Reset survey data on new scan
        document.getElementById('survey-legend').style.display = 'flex';
    } else {
        map.addLayer(tileLayer);
        document.getElementById('map').style.background = '';
        document.getElementById('survey-legend').style.display = 'none';
    }
}

function setMagneticBaseline() {
    magBaseline = window.lastMagMagnitude || 0;
    if(magBaseline > 0) {
        document.getElementById('survey-baseline-val').innerText = `Baseline: ${magBaseline.toFixed(2)} µT`;
        document.getElementById('survey-baseline-val').style.display = 'inline';
    }
}

// Telemetry Logic
socket.on('telemetry', (data) => {
    // Initial centering
    if (!mapInitialized && data.glat !== 0 && data.glon !== 0) {
        map.setView([data.glat, data.glon], 18);
        mapInitialized = true;
    }

    if (data.mx !== undefined && data.my !== undefined && data.mz !== undefined) {
        let mx = data.mx;
        let my = data.my;
        let mz = data.mz;
        
        // Convert raw LSB to true Microteslas (µT) based on sensor hardware
        let lsb_per_gauss = 1090.0; // Default HMC5883L
        if (data.mt !== undefined) {
            if (data.mt === 13) lsb_per_gauss = 3000.0; // QMC5883L (8G config)
            else if (data.mt === 30) lsb_per_gauss = 1090.0; // HMC5883L
            else if (data.mt === 44) lsb_per_gauss = 3000.0; // QMC5883P
        }

        // 1 Gauss = 100 µT
        mx = (mx / lsb_per_gauss) * 100.0;
        my = (my / lsb_per_gauss) * 100.0;
        mz = (mz / lsb_per_gauss) * 100.0;

        window.lastMagMagnitude = Math.sqrt(mx*mx + my*my + mz*mz);
    }

    // Aeromagnetic Survey Logic
    if (isSurveyMode && magBaseline > 0 && data.glat !== 0 && data.glon !== 0 && data.gf > 0) {
        
        // Hover detection
        let distance = 100;
        if (lastPlottedLat !== 0 && lastPlottedLon !== 0) {
            distance = map.distance([lastPlottedLat, lastPlottedLon], [data.glat, data.glon]);
        }
        
        if (distance > 2.0) {
            lastPlottedLat = data.glat;
            lastPlottedLon = data.glon;

            let currentMag = window.lastMagMagnitude;
            let variance = Math.abs(currentMag - magBaseline);
            
            // Map variance to intensity (0.0 to 1.0)
            // 50.0 scales small anomalies so they are clearly visible
            let intensity = Math.min(1.0, variance / 50.0);
            
            if (variance > 5) { 
                heatLayer.addLatLng([data.glat, data.glon, intensity]);
            }
            
            // Record data for the 2D Contour Map
            surveyData.push({lat: data.glat, lon: data.glon, val: variance});
            
            L.circleMarker([data.glat, data.glon], {
                radius: 2,
                color: '#0ea5e9',
                fillColor: '#0ea5e9',
                fillOpacity: 1,
                stroke: false
            }).addTo(surveyPathLayer);
            
            let now = Date.now();
            if (now - lastFitBoundsTime > 1000) {
                if (surveyPathLayer.getLayers().length > 1) {
                    map.fitBounds(surveyPathLayer.getBounds(), { padding: [50, 50], maxZoom: 24, animate: false });
                }
                lastFitBoundsTime = now;
            }
        }
    }
});

// --- 2D Contour Map (IDW Interpolation) ---

function generateMap(mapType) {
    if (surveyData.length < 3) {
        alert("Not enough survey data collected yet. Please collect at least 3 points.");
        return;
    }

    // Show the overlay
    document.getElementById('contour-container').style.display = 'block';
    document.getElementById('btn-generate-2d').style.display = 'none';
    document.getElementById('btn-generate-3d').style.display = 'none';
    document.getElementById('btn-close-map').style.display = 'block';
    document.getElementById('survey-legend').style.display = 'none';

    // Find bounding box
    let minLat = 90, maxLat = -90, minLon = 180, maxLon = -180;
    for (let p of surveyData) {
        if (p.lat < minLat) minLat = p.lat;
        if (p.lat > maxLat) maxLat = p.lat;
        if (p.lon < minLon) minLon = p.lon;
        if (p.lon > maxLon) maxLon = p.lon;
    }

    // Add padding
    let latPad = Math.max((maxLat - minLat) * 0.1, 0.0001);
    let lonPad = Math.max((maxLon - minLon) * 0.1, 0.0001);
    minLat -= latPad; maxLat += latPad;
    minLon -= lonPad; maxLon += lonPad;

    // Create 50x50 Grid
    let resolution = 50;
    let gridZ = [];
    let gridX = [];
    let gridY = [];

    for(let i=0; i<resolution; i++) {
        gridX.push(minLon + (i/(resolution-1)) * (maxLon - minLon));
        gridY.push(minLat + (i/(resolution-1)) * (maxLat - minLat));
    }

    // Inverse Distance Weighting (IDW)
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
                
                if (d2 < 0.000000001) d2 = 0.000000001; // Avoid division by zero
                
                let weight = 1.0 / d2;
                numerator += weight * p.val;
                denominator += weight;
            }
            row.push(numerator / denominator);
        }
        gridZ.push(row);
    }

    // Plotly Data
    let data = [{
        z: gridZ,
        x: gridX,
        y: gridY,
        type: mapType, // 'heatmap' or 'surface'
        colorscale: [
            ['0.0', 'rgb(15, 23, 42)'],    // Slate 900 (Background)
            ['0.2', 'rgb(14, 165, 233)'],  // Cyan
            ['0.5', 'rgb(132, 204, 22)'],  // Lime Green
            ['0.8', 'rgb(234, 179, 8)'],   // Yellow
            ['1.0', 'rgb(239, 68, 68)']    // Red
        ],
        showscale: true,
        zsmooth: mapType === 'heatmap' ? false : undefined // Blocky for 2D, smooth for 3D
    }];

    let layout = {
        title: { text: 'Aeromagnetic Survey (IDW Interpolation)', font: {color: '#f8fafc', size: 24} },
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
