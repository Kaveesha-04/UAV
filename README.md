<p align="center">
  <h1 align="center">🚁 Aeronexus — Open-Source UAV Flight Controller</h1>
  <p align="center">
    A custom-built quadcopter flight controller system powered by <strong>STM32H743</strong>, with an <strong>ESP32</strong> radio transmitter, <strong>ESP32</strong> WiFi telemetry bridge, and a real-time <strong>Node.js</strong> ground station dashboard.
  </p>
</p>

<p align="center">
  <a href="#features">Features</a> •
  <a href="#architecture">Architecture</a> •
  <a href="#hardware">Hardware</a> •
  <a href="#getting-started">Getting Started</a> •
  <a href="#project-structure">Project Structure</a> •
  <a href="#license">License</a>
</p>

---

## Features

### 🎮 Flight Controller (STM32H743)
- **250 Hz PID control loop** with D-on-measurement, feedforward, and throttle-based PID attenuation (TPA)
- **Sensor fusion** — Mahony AHRS quaternion filter + complementary filter with centrifugal compensation
- **Multi-sensor support**:
  - MPU6500 IMU (gyroscope + accelerometer) via SPI
  - BMP280 barometer (altitude, temperature, pressure) via SPI
  - QMC5883L / HMC5883L / QMC5883P magnetometer (auto-detected) via I2C
  - GPS module (NMEA GGA + RMC parsing with EMA smoothing) via UART
- **NRF24L01+ radio** receiver (250 kbps, 2.4 GHz)
- **Automated aeromagnetic survey** with waypoint queue (50 waypoints max)
- **Flash storage** for persistent PID gains and calibration data
- **USB CDC** virtual COM port for telemetry and commands
- **Battery voltage monitoring** with configurable low-voltage failsafe

### 📡 ESP32 Remote Transmitter
- Dual-axis joystick input via ADS1115 ADC
- NRF24L01+ radio TX at 50 Hz
- OLED display for real-time status
- Configurable trims, rates, and expo

### 🌐 ESP32 WiFi Telemetry Bridge
- UART ↔ TCP bridge between the flight controller and ground station
- Bidirectional JSON telemetry + command forwarding
- Auto-reconnect with WiFi and server

### 📊 Ground Station Dashboard (Node.js + Socket.IO)
- **Command Dashboard** — Real-time telemetry display, PID tuning interface, magnetometer calibration, 3D drone visualization
- **Aeromagnetic Survey Dashboard** — Mission planner with interactive map, magnetic field heatmaps, survey data export
- **Fleet Management Dashboard** — Multi-drone monitoring, drone assignment, connection pool management
- **Authentication** — Fleet admin and per-drone PIN-based login
- Glassmorphism dark theme with responsive design

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
│         fleet.html (Multi-Drone Fleet Management)       │
└─────────────────────────────────────────────────────────┘
```

---

## Hardware

### STM32H743 Flight Controller Peripherals

| Peripheral | Pins | Connected Device | Configuration |
|-----------|------|------------------|---------------|
| SPI1 | PA5/PA6/PA7 + PA4(CSN) + PC4(CE) | NRF24L01+ Radio | 250 kbps, Ch 76 |
| SPI2 | PB10/PC2/PC1 + PB12(CS) + PB11(CS) | MPU6500 + BMP280 | 500°/s, 8G |
| I2C1 | PB6(SCL) / PB7(SDA) | QMC5883L / HMC5883L Magnetometer | Auto-detected |
| UART2 | PA2(TX) / PA3(RX) | ESP32 Telemetry Bridge | 115200 baud |
| UART3 | PD9(TX) / PD2(RX) | GPS Module (NMEA) | 115200 baud |
| USB FS | — | CDC Virtual COM Port | Telemetry + Commands |
| TIM3 | CH1–CH4 (PC6/PC7/PC8/PB1) | ESC Motor PWM | 1100–2000 µs |
| TIM4 | — | 250 Hz PID Loop ISR | Sets `pid_loop_flag` |
| ADC1 | — | Battery Voltage (4:1 divider) | 16-bit, 3.3V ref |

### Motor Layout

```
    Front
  FL ╲  ╱ FR
      ╳
  BL ╱  ╲ BR
    Back
```

| Motor | Position | Timer Channel | Pin |
|-------|----------|---------------|-----|
| M3 | Front Left | TIM3 CH3 | PC8 |
| M4 | Front Right | TIM3 CH4 | PB1 |
| M1 | Back Left | TIM3 CH1 | PC6 |
| M2 | Back Right | TIM3 CH2 | PC7 |

---

## Getting Started

### Prerequisites

- **STM32CubeIDE** (v1.13+) — for building and flashing the flight controller firmware
- **Arduino IDE** (v2.x) or **PlatformIO** — for ESP32 sketches
- **Node.js** (v18+) and **npm** — for the ground station dashboard
- **Hardware**: STM32H743VIT6 board, ESP32 (×2), NRF24L01+ (×2), MPU6500, BMP280, QMC5883L/HMC5883L, GPS module, ESCs, brushless motors

### 1. Flash the Flight Controller

1. Open `Aeronexus firmware/` as an STM32CubeIDE project
2. Build and flash via ST-Link or USB DFU
3. Verify USB CDC appears as a COM port

### 2. Flash the ESP32 Remote Transmitter

1. Open `AMD24 mini transmitter/ESP32_Remote.ino` in Arduino IDE
2. Install required libraries: `RF24`, `Adafruit_ADS1X15`, `Adafruit_SSD1306`, `Adafruit_GFX`
3. Flash to the remote's ESP32

### 3. Flash the ESP32 Telemetry Bridge

1. Open `Aeronexus telemetry/ESP32_Telemetry.ino` in Arduino IDE
2. **Update WiFi credentials** (SSID, password) and server IP address
3. Flash to the telemetry ESP32

### 4. Run the Ground Station Dashboard

```bash
cd "Aeronexus dashboard"
npm install
npm start
```

The dashboard will be available at `http://localhost:3000`

---

## Project Structure

```
Aeronexus/
├── Aeronexus firmware/            # STM32H743 Flight Controller (STM32CubeIDE)
│   ├── Core/
│   │   ├── Inc/                   # Header files
│   │   │   ├── main.h             # Pin definitions, HAL config, global externs
│   │   │   ├── mpu6500.h          # IMU driver (SPI2)
│   │   │   ├── mahony.h           # Mahony AHRS quaternion filter
│   │   │   ├── bmp280.h           # Barometer driver (SPI2)
│   │   │   ├── qmc5883l.h         # QMC5883L/P magnetometer (I2C1)
│   │   │   ├── hmc5883l.h         # HMC5883L magnetometer (I2C1)
│   │   │   ├── gps.h              # GPS NMEA parser (UART3)
│   │   │   ├── nrf24.h            # NRF24L01 radio driver (SPI1)
│   │   │   ├── pid.h              # PID controller
│   │   │   ├── motors.h           # PWM motor output (TIM3)
│   │   │   ├── flash_store.h      # Internal flash storage
│   │   │   └── auto_survey.h      # Waypoint queue + survey state machine
│   │   └── Src/                   # Source files (matching headers above)
│   ├── Drivers/                   # STM32 HAL + CMSIS (vendor provided)
│   ├── Middlewares/               # STM32 USB Device Library (vendor provided)
│   └── USB_DEVICE/                # USB CDC configuration
│
├── AMD24 mini transmitter/        # ESP32 RC Transmitter (Arduino)
│   └── ESP32_Remote.ino           # Joystick → NRF24L01 TX (50 Hz)
│
├── Aeronexus telemetry/           # ESP32 WiFi Telemetry Bridge (Arduino)
│   └── ESP32_Telemetry.ino        # UART ↔ TCP bridge to ground station
│
├── Aeronexus dashboard/           # Ground Station Dashboard (Node.js)
│   ├── server.js                  # Express + Socket.IO + USB Serial + TCP
│   ├── package.json               # Dependencies
│   └── public/
│       ├── index.html             # Command Dashboard
│       ├── script.js              # Dashboard logic, charts, 3D visualization
│       ├── style.css              # Glassmorphism dark theme
│       ├── survey.html            # Aeromagnetic Survey Dashboard
│       ├── survey.js              # Survey map, heatmaps, mission planner
│       ├── fleet.html             # Fleet Management Dashboard
│       ├── fleet.js               # Fleet monitoring + drone assignment
│       ├── fleet.css              # Fleet dashboard styles
│       ├── login.html             # Fleet admin authentication
│       └── drone_login.html       # Per-drone PIN authentication
│
├── CODEBASE.md                    # Detailed codebase documentation
├── pid_tuning_guide.md            # PID tuning reference guide
├── LICENSE                        # MIT License (this project)
└── README.md                      # This file
```

---

## Configuration

### WiFi & Network Settings

The ESP32 telemetry bridge requires WiFi credentials and the ground station server IP. Update these in `Aeronexus telemetry/ESP32_Telemetry.ino`:

```cpp
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* server_ip = "YOUR_PC_LOCAL_IP";
```

### PID Tuning

PID gains can be tuned in real-time through the web dashboard or by modifying the defaults in `main.c`. See [pid_tuning_guide.md](pid_tuning_guide.md) for a comprehensive tuning reference.

### Magnetometer Calibration

Hard-iron and soft-iron calibration is supported through the web dashboard's magnetometer calibration interface. Calibration data is persisted to internal flash.

---

## ⚠️ Safety Warning

**This is a flight controller for unmanned aerial vehicles.** Improper configuration or software bugs can cause dangerous behavior including uncontrolled flight, crashes, and property damage or injury.

- **Always remove propellers** when testing on the bench
- **Verify all PID gains** are reasonable before first flight
- **Test failsafe behavior** (radio loss, low battery) before flying
- **Fly in open areas** away from people and property
- **Comply with local drone regulations** in your jurisdiction

---

## Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/your-feature`)
3. Commit your changes (`git commit -m "Add your feature"`)
4. Push to the branch (`git push origin feature/your-feature`)
5. Open a Pull Request

---

## License

This project is licensed under the **MIT License** — see the [LICENSE](LICENSE) file for details.

### Third-Party Licenses

This project includes vendor-provided libraries with their own licenses:

| Component | License | Location |
|-----------|---------|----------|
| STM32H7xx HAL Driver | BSD-3-Clause | `Aeronexus firmware/Drivers/STM32H7xx_HAL_Driver/LICENSE.txt` |
| ARM CMSIS | Apache License 2.0 | `Aeronexus firmware/Drivers/CMSIS/LICENSE.txt` |
| STM32 USB Device Library | SLA0044 (Ultimate Liberty) | `Aeronexus firmware/Middlewares/ST/STM32_USB_Device_Library/LICENSE.txt` |

> **Note:** The STM32 USB Device Library (SLA0044) restricts use to STMicroelectronics microcontrollers only. This restriction applies solely to the USB middleware component, not to the rest of this project.

---

## Acknowledgments

- **STMicroelectronics** — STM32 HAL drivers, USB middleware, and STM32CubeIDE
- **ARM** — CMSIS core headers
- **Nordic Semiconductor** — NRF24L01+ radio protocol reference
- **Arduino Community** — ESP32 libraries and toolchain
