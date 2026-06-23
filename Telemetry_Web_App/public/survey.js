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
    gradient: {0.2: '#84cc16', 0.6: '#eab308', 1.0: '#ef4444'}
}).addTo(map);

const surveyPathLayer = L.featureGroup().addTo(map);

// Survey State
let isSurveyMode = false;
let magBaseline = 0;
let lastFitBoundsTime = 0;
let lastPlottedLat = 0;
let lastPlottedLon = 0;
let mapInitialized = false;

// Controls
function toggleHeatmap() {
    isSurveyMode = document.getElementById('heatmap-toggle').checked;
    if (isSurveyMode) {
        // Dark room mode for survey
        map.removeLayer(tileLayer);
        document.getElementById('map').style.background = 'var(--bg-color)'; 
        lastPlottedLat = 0; 
        lastPlottedLon = 0;
    } else {
        map.addLayer(tileLayer);
        document.getElementById('map').style.background = '';
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
        window.lastMagMagnitude = Math.sqrt(data.mx*data.mx + data.my*data.my + data.mz*data.mz);
    }

    // Aeromagnetic Survey Logic
    if (isSurveyMode && magBaseline > 0 && data.glat !== 0 && data.glon !== 0 && data.gf > 0) {
        
        // Hover detection
        let distance = 100;
        if (lastPlottedLat !== 0 && lastPlottedLon !== 0) {
            distance = map.distance([lastPlottedLat, lastPlottedLon], [data.glat, data.glon]);
        }
        
        if (distance > 0.5) {
            lastPlottedLat = data.glat;
            lastPlottedLon = data.glon;

            let currentMag = window.lastMagMagnitude;
            let variance = Math.abs(currentMag - magBaseline);
            
            let intensity = Math.min(1.0, variance / 300.0);
            
            if (variance > 20) { 
                heatLayer.addLatLng([data.glat, data.glon, intensity]);
            }
            
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
