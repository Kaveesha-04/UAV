# UAV Template — Codebase Documentation

## Project Overview

A custom-built quadcopter flight controller system built on **STM32H743** (Cortex-M7), with an **ESP32** remote controller, **ESP32** WiFi telemetry bridge, and a **Node.js** web dashboard for real-time monitoring, PID tuning, magnetometer calibration, and automated aeromagnetic surveys.

---

## Architecture

```
┌─────────────────┐    NRF24L01 (2.4GHz)    ┌─────────────────────────┐
│  ESP32 Remote   │ ──────────────────────►  │    STM32H743 (FC)       │
│  (Joystick TX)  │   8-byte Data_Package    │  main.c — Flight Loop   │
└─────────────────┘                          │  PID / Motors / GPS     │
                                             │  Magnetometer / Baro    │
┌─────────────────┐    UART2 (115200)        │  Auto Survey Module     │
│ ESP32 Telemetry │ ◄───────────────────────►│                         │
│  (WiFi Bridge)  │  JSON telemetry + cmds   └─────────┬───────────────┘
└────────┬────────┘                                     │ USB CDC
         │ TCP:5000                                     │
         ▼                                              ▼
┌─────────────────────────────────────────────────────────┐
│              Node.js Server (server.js)                 │
│         WebSocket (Socket.IO) ◄──► Web Dashboard        │
│   index.html (Command)  |  survey.html (Survey Map)     │
└─────────────────────────────────────────────────────────┘
```

---

## Directory Structure

```
UAV Template/
├── UAV Template/              # STM32CubeIDE Project (Flight Controller)
│   ├── Core/
│   │   ├── Inc/               # Header files
│   │   │   ├── main.h         # Pin definitions, HAL config
│   │   │   ├── mpu6500.h      # IMU (SPI2) — gyro + accelerometer
│   │   │   ├── bmp280.h       # Barometer (SPI2) — altitude
│   │   │   ├── qmc5883l.h     # Magnetometer (I2C1) — heading
│   │   │   ├── hmc5883l.h     # Magnetometer alt (I2C1) — heading
│   │   │   ├── gps.h          # GPS (UART3) — position + speed
│   │   │   ├── nrf24.h        # NRF24L01 Radio (SPI1) — RC input
│   │   │   ├── pid.h          # PID controller struct + compute
│   │   │   ├── motors.h       # PWM motor output (TIM3 CH1-4)
│   │   │   ├── flash_store.h  # Flash storage for PIDs + mag offsets
│   │   │   └── auto_survey.h  # Waypoint queue for lawnmower survey
│   │   └── Src/               # Source files
│   │       ├── main.c         # Main flight loop (250Hz via TIM4 ISR)
│   │       ├── mpu6500.c      # MPU6500 driver + complementary filter
│   │       ├── bmp280.c       # BMP280 barometer driver
│   │       ├── qmc5883l.c     # QMC5883L magnetometer driver
│   │       ├── hmc5883l.c     # HMC5883L magnetometer driver
│   │       ├── gps.c          # NMEA parser (GGA + RMC) + EMA smoothing
│   │       ├── nrf24.c        # NRF24L01 SPI driver (RX mode)
│   │       ├── pid.c          # PID with D-on-measurement, feedforward, TPA
│   │       ├── motors.c       # PWM output to ESCs
│   │       ├── flash_store.c  # STM32H7 internal flash read/write
│   │       └── auto_survey.c  # Survey waypoint queue (50 WP max)
│   └── Debug/Core/Src/
│       └── subdir.mk          # GNU Make build rules
│
├── ESP32_Remote/              # RC Transmitter (Arduino)
│   └── ESP32_Remote.ino       # Joystick → NRF24L01 TX (50Hz)
│
├── ESP32_Telemetry/           # WiFi Telemetry Bridge (Arduino)
│   └── ESP32_Telemetry.ino    # UART ↔ TCP bridge to Node.js server
│
└── Telemetry_Web_App/         # Ground Station Dashboard
    ├── server.js              # Express + Socket.IO + USB Serial + TCP
    ├── package.json
    └── public/
        ├── index.html         # Command Dashboard (telemetry, PID, waypoints)
        ├── script.js          # Dashboard logic, charts, mag calibration
        ├── style.css          # Shared styles (glassmorphism dark theme)
        ├── survey.html        # Aeromagnetic Survey Dashboard
        └── survey.js          # Survey map, heatmaps, lawnmower planner
```

---

## STM32 Flight Controller (main.c)

### Loop Architecture

- **250Hz PID loop** driven by TIM4 ISR (`pid_loop_flag`)
- **50Hz sensor polling** (barometer + magnetometer, every 5th loop)
- **5Hz telemetry** output via USB CDC + UART2 (every 50th loop)
- **GPS parsing** in main loop (event-driven from UART3 RX interrupt)

### Flight State Machine

```
STATE_BOOT → STATE_DISARMED ↔ STATE_ARMED → STATE_FAILSAFE
```

- **Arming**: Stick command (throttle down + yaw right 1s), dashboard `A,1`, or hardware button
- **Disarming**: Stick command (throttle down + yaw left 1s), dashboard `A,0`, or hardware button

### Flight Modes

| Mode | Enum | Behavior |
|------|------|----------|
| STABILIZE | `MODE_STABILIZE` | Manual throttle + attitude PID only |
| ALT_HOLD | `MODE_ALTHOLD` | Barometer altitude hold + attitude PID |
| LOITER | `MODE_LOITER` | GPS position hold + altitude hold |
| RTL | `MODE_RTL` | Return to home GPS coordinates |
| AUTO | `MODE_AUTO` | Fly to `wp_lat`/`wp_lon` waypoint |

### PID Controllers

| Controller | Purpose | Output Range |
|-----------|---------|-------------|
| `pid_roll` | Roll stabilization | ±200 |
| `pid_pitch` | Pitch stabilization | ±200 |
| `pid_yaw` | Yaw rate control (angular) | ±200 |
| `pid_alt` | Altitude hold → throttle correction | -200 to +300 |
| `pid_gps_pitch` | GPS nav → pitch angle override | ±20° |
| `pid_gps_roll` | GPS nav → roll angle override | ±20° |

### Motor Mixing (Quad-X, Props In)

```
CH1 (m1) = Rear Left  (CCW): throttle + roll - pitch + yaw
CH2 (m2) = Rear Right (CW):  throttle - roll - pitch - yaw
CH3 (m3) = Front Left (CW):  throttle + roll + pitch - yaw
CH4 (m4) = Front Right(CCW): throttle - roll + pitch + yaw
```

### Safety Systems

- **Radio Failsafe**: No NRF packets for 500ms → RTL (if GPS) or auto-land
- **Dashboard Failsafe**: No heartbeat for 3s → abort AUTO to RTL
- **Geofence**: >500m = force RTL, >1000m = hard kill
- **Crash Detection**: >45° tilt = instant disarm
- **Low Battery**: <10.2V (3S) = auto-land
- **AirMode**: Motor minimum clamping preserves rotational authority

---

## Command Protocol (USB / UART2)

Commands sent from dashboard → STM32 as newline-terminated strings:

| Prefix | Format | Action |
|--------|--------|--------|
| `P` | `P,axis,Kp*100,Ki*100,Kd*100,Kf*100` | Update PID gains |
| `M` | `M,mode_name` | Change flight mode |
| `W` | `W,lat,lon` | Single waypoint → AUTO mode |
| `A` | `A,1` or `A,0` | Arm / Disarm |
| `B` | `B` | Burn PIDs + mag offsets to flash |
| `C` | `C,offset_x,offset_y,offset_z` | Set magnetometer offsets |
| `H` | `H` | Dashboard heartbeat |
| `S` | `S,RESET` | Clear survey waypoint queue |
| `S` | `S,WP,lat,lon` | Add waypoint to survey queue |
| `S` | `S,START` | Begin survey mission (requires ARMED) |
| `S` | `S,PAUSE` | Pause survey → LOITER |
| `S` | `S,RESUME` | Resume survey → AUTO |
| `S` | `S,ABORT` | Abort survey → LOITER |

---

## Telemetry JSON (5Hz, STM32 → Dashboard)

```json
{
  "v": 12.60,        // Battery voltage
  "r": -2.3,         // Roll (degrees)
  "p": 1.5,          // Pitch (degrees)
  "y": 180.0,        // Yaw (degrees)
  "a": 5.2,          // Altitude (meters, barometric)
  "d": 270.0,        // Compass heading (degrees)
  "glat": 6.92710,   // GPS latitude
  "glon": 79.86120,  // GPS longitude
  "gf": 1,           // GPS fix type (0=none, 1=GPS, 2=DGPS)
  "t": 1500,         // Throttle value
  "mt": 13,          // Magnetometer type (0x0D=QMC, 0x1E=HMC, 0x2C=QMC-P)
  "pid_r": [0.50, 0.00, 0.01, 0.00],  // Roll PID [P, I, D, F]
  "pid_p": [0.50, 0.00, 0.01, 0.00],  // Pitch PID
  "pid_y": [0.50, 0.00, 0.01, 0.00],  // Yaw PID
  "md": 0,           // Flight mode (0=STAB, 1=ALT, 2=LOIT, 3=RTL, 4=AUTO)
  "mx": 150,         // Raw mag X
  "my": -200,        // Raw mag Y
  "mz": 400,         // Raw mag Z
  "ry": 0, "rp": 0, "rr": 0,  // Raw NRF joystick values
  "arm": 1,          // Armed state (0/1)
  "sig": 85,         // NRF signal strength (0-100%)
  "m1": 1500, "m2": 1500, "m3": 1500, "m4": 1500,  // Motor PWM
  "gsat": 8,         // GPS satellites
  "ghdop": 1.2,      // GPS HDOP (horizontal dilution)
  "gspd": 2.5,       // GPS ground speed (m/s)
  "slat": 6.92711,   // EMA-smoothed latitude (survey only)
  "slon": 79.86121,  // EMA-smoothed longitude (survey only)
  "amx": 148, "amy": -199, "amz": 401,  // Averaged mag readings (survey)
  "swp": 5,          // Survey: current waypoint index
  "swt": 24,         // Survey: total waypoint count
  "sst": 1           // Survey: state (0=idle, 1=running, 2=paused, 3=done)
}
```

---

## Auto Survey Module (auto_survey.c)

Manages an automated lawnmower (boustrophedon) flight pattern for aeromagnetic surveys.

- **Queue**: Up to 50 waypoints stored in a static array (~400 bytes RAM)
- **Waypoint Radius**: 3m threshold to advance (based on GPS CEP accuracy)
- **States**: `SURVEY_IDLE` → `SURVEY_RUNNING` ↔ `SURVEY_PAUSED` → `SURVEY_DONE`
- **Integration**: Updates `wp_lat`/`wp_lon` in main.c each loop; drone uses existing `MODE_AUTO` GPS navigation
- **Safety**: Only starts if armed; all existing failsafes remain active

---

## Web Dashboard

### Server (server.js)

- **Express** static file server on port 3000
- **TCP server** on port 5000 for ESP32 WiFi telemetry bridge
- **USB Serial** auto-detection of STM32 (Vendor ID `0483`) as fallback
- **Socket.IO** WebSocket bridge: browser ↔ drone (bidirectional)
- Priority: WiFi (ESP32 TCP) > USB (direct serial)

### Command Dashboard (index.html + script.js)

- Real-time artificial horizon with roll/pitch/yaw
- Live telemetry chart (Chart.js) — roll, pitch, yaw, altitude
- PID tuning panels with live editing + flash burn
- Magnetometer 3D calibration plot (Plotly scatter3d)
- GPS map with waypoint selection (Leaflet + ESRI satellite tiles)
- Motor output bars, battery, signal strength, GPS status
- ARM/DISARM buttons with safety confirmations

### Survey Dashboard (survey.html + survey.js)

- Full-screen satellite map (Leaflet) for aeromagnetic surveys
- **Scanner mode**: Real-time magnetic anomaly heatmap overlay
- **Auto/Manual baseline** calibration (10s median sampling)
- **Data quality**: GPS gating (HDOP/satellite thresholds), speed filtering, turn rejection
- **Geophysics processing**:
  - Altitude normalization (inverse cube law)
  - Diurnal drift correction (linear regression)
  - Fourth-difference noise estimation
  - Spatial gradient computation
- **Lawnmower Mission Planner**:
  - Click two corners on map to define survey rectangle
  - Configurable line spacing (default 5m)
  - Generates boustrophedon waypoints (max 50)
  - Upload → Start → Pause/Resume → Abort controls
  - Live progress tracking (waypoint counter + progress bar)
- **Visualization**: 2D heatmaps, 3D surface plots (Plotly), grid coverage overlay
- **Export**: CSV with all survey data (corrected anomalies, gradients, quality metrics)

---

## Hardware Peripherals (STM32H743)

| Peripheral | Bus | Connected Device |
|-----------|-----|-----------------|
| SPI1 | PA5/PA6/PA7 + PB0 (CSN) | NRF24L01+ Radio |
| SPI2 | PB13/PB14/PB15 | MPU6500 IMU + BMP280 Barometer |
| I2C1 | PB6 (SCL) / PB7 (SDA) | QMC5883L / HMC5883L Magnetometer |
| UART2 | PA2 (TX) / PA3 (RX) | ESP32 Telemetry Bridge |
| UART3 | — | GPS Module (NMEA) |
| USB FS | — | CDC Virtual COM Port (telemetry + commands) |
| TIM3 | CH1-CH4 | ESC Motor PWM (1100-2000µs) |
| TIM4 | — | 250Hz PID loop ISR |
| ADC1 | — | Battery voltage (4:1 divider, 3S LiPo) |

---

## ESP32 Remote Controller

- **ADS1115** I2C ADC reads 4 analog joystick channels (16-bit precision)
- Maps to calibrated midpoints with ±500 range and deadband
- Transmits **50Hz** via NRF24L01 as 8-byte `Data_Package`
- **SSD1306 OLED** displays live throttle/yaw/pitch/roll values
- Power-safe: throttle must be zero for transmission to start

## ESP32 Telemetry Bridge

- Receives JSON telemetry from STM32 via UART2 (115200 baud)
- Forwards to Node.js server via TCP (port 5000) over WiFi
- Relays dashboard commands back to STM32 via UART2
- Auto-reconnects on TCP disconnect (2s retry)
- 512-byte buffer protection against OOM from dropped newlines

---

## NRF24L01 Data Package

```c
#pragma pack(push, 1)
struct Data_Package {
  uint16_t throttle; // 1000 to 2000
  int16_t  yaw;      // -500 to 500
  int16_t  pitch;    // -500 to 500
  int16_t  roll;     // -500 to 500
};
#pragma pack(pop)
// Total: 8 bytes, transmitted at 50Hz
```
