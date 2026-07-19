# UAV Template — Full Codebase Documentation

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
│         fleet.html (Multi-Drone Fleet Management)       │
└─────────────────────────────────────────────────────────┘
```

---

## Directory Structure

```
UAV Template/
├── UAV Template/              # STM32CubeIDE Project (Flight Controller)
│   ├── Core/
│   │   ├── Inc/               # Header files
│   │   │   ├── main.h         # Pin definitions, HAL config, global externs
│   │   │   ├── mpu6500.h      # IMU (SPI2) — gyro + accelerometer + IMU offsets
│   │   │   ├── mahony.h       # Mahony AHRS quaternion filter parameters
│   │   │   ├── bmp280.h       # Barometer (SPI2) — altitude, temp, pressure
│   │   │   ├── qmc5883l.h     # QMC5883L + QMC5883P magnetometer (I2C1)
│   │   │   ├── hmc5883l.h     # HMC5883L magnetometer (I2C1)
│   │   │   ├── gps.h          # GPS (UART3) — position, speed, quality, smoothed
│   │   │   ├── nrf24.h        # NRF24L01 Radio (SPI1) — RC input + Data_Package
│   │   │   ├── pid.h          # PID controller struct + compute prototypes
│   │   │   ├── motors.h       # PWM motor output (TIM3 CH1-4)
│   │   │   ├── flash_store.h  # Flash storage struct + magic number
│   │   │   └── auto_survey.h  # Waypoint queue + survey state machine
│   │   └── Src/               # Source files
│   │       ├── main.c         # Main flight loop (250Hz via TIM4 ISR)
│   │       ├── mpu6500.c      # MPU6500 driver + complementary filter + centrifugal comp
│   │       ├── mahony.c       # Mahony AHRS quaternion IMU fusion
│   │       ├── bmp280.c       # BMP280 barometer driver + altitude calibration
│   │       ├── qmc5883l.c     # QMC5883L + QMC5883P magnetometer drivers
│   │       ├── hmc5883l.c     # HMC5883L + disguised QMC detection
│   │       ├── gps.c          # NMEA parser (GGA + RMC) + checksum + EMA smoothing
│   │       ├── nrf24.c        # NRF24L01 SPI driver (RX mode, 250kbps)
│   │       ├── pid.c          # PID with D-on-measurement, feedforward, TPA
│   │       ├── motors.c       # PWM output to ESCs via TIM3
│   │       ├── flash_store.c  # STM32H7 internal flash read/write (32-byte words)
│   │       ├── auto_survey.c  # Survey waypoint queue (50 WP max)
│   │       └── stm32h7xx_it.c # Interrupt handlers (TIM4 250Hz ISR, UART, USB)
│   ├── STM32H743VITX_FLASH.ld # Flash linker script
│   └── UAV_main_program.ioc   # STM32CubeMX pin configuration
│
├── ESP32_Remote/              # RC Transmitter (Arduino)
│   └── ESP32_Remote.ino       # Joystick → NRF24L01 TX (50Hz)
│
├── ESP32_Telemetry/           # WiFi Telemetry Bridge (Arduino)
│   └── ESP32_Telemetry.ino    # UART ↔ TCP bridge to Node.js server
│
└── Telemetry_Web_App/         # Ground Station Dashboard
    ├── server.js              # Express + Socket.IO + USB Serial + TCP + Fleet
    ├── package.json           # Dependencies (express, socket.io, serialport)
    └── public/
        ├── index.html         # Command Dashboard (telemetry, PID, waypoints)
        ├── script.js          # Dashboard logic, charts, mag calibration, 3D drone
        ├── style.css          # Shared styles (glassmorphism dark theme)
        ├── survey.html        # Aeromagnetic Survey Dashboard
        ├── survey.js          # Survey map, heatmaps, geophysics, mission planner
        ├── fleet.html         # Multi-Drone Fleet Management Dashboard
        ├── fleet.js           # Fleet status, drone assignment, connection pool
        ├── fleet.css          # Fleet dashboard styles
        ├── login.html         # Fleet admin authentication page
        └── drone_login.html   # Per-drone PIN authentication page
```

---

## 1. STM32 Flight Controller — Detailed Reference

### 1.1 Hardware Peripherals (STM32H743)

| Peripheral | Pins | Connected Device | Config |
|-----------|------|------------------|--------|
| SPI1 | PA5(SCK)/PA6(MISO)/PA7(MOSI) + PA4(CSN) + PB0(CE) | NRF24L01+ Radio | 250kbps, Ch76 |
| SPI2 | PB13(SCK)/PB14(MISO)/PB15(MOSI) + PB12(CS_MPU) + PB11(CS_BMP) | MPU6500 IMU + BMP280 Baro | 500dps, 8G / Filter 16 |
| I2C1 | PB6(SCL)/PB7(SDA) | QMC5883L/HMC5883L/QMC5883P Mag | Auto-detected |
| UART2 | PA2(TX)/PA3(RX) | ESP32 Telemetry Bridge | 115200 baud |
| UART3 | PD8(TX)/PD9(RX) | GPS Module (NMEA) | 115200 baud |
| USB FS | — | CDC Virtual COM Port | Telemetry + Commands |
| TIM3 | CH1(PC6)/CH2(PC7)/CH3(PC8)/CH4(PB1) | ESC Motor PWM | 1100-2000µs |
| TIM4 | — | 250Hz PID loop ISR | Sets `pid_loop_flag` |
| ADC1 | — | Battery voltage (4:1 divider) | 16-bit, 3.3V ref |
| GPIO | PE3(K1)/PE4(K2) | Hardware buttons | Arm/Disarm |
| GPIO | PA1 | Status LED | Armed/Failsafe indicator |

### 1.2 Pin Definitions (main.h)

```c
#define BTN_K1_Pin       GPIO_PIN_3   // PE3 — Primary arm/disarm button
#define BTN_K2_Pin       GPIO_PIN_4   // PE4 — Secondary button
#define LED_STATUS_Pin   GPIO_PIN_1   // PA1 — Status LED
#define NRF_CSN_Pin      GPIO_PIN_4   // PA4 — NRF24 chip select
#define NRF_CE_Pin       GPIO_PIN_0   // PB0 — NRF24 chip enable
#define BMP_CS_Pin       GPIO_PIN_11  // PB11 — BMP280 chip select
#define MPU_CS_Pin       GPIO_PIN_12  // PB12 — MPU6500 chip select
#define CH1_M_BL_Pin     GPIO_PIN_6   // PC6 — Motor 1 (Rear Left)
#define CH2_M_BR_Pin     GPIO_PIN_7   // PC7 — Motor 2 (Rear Right)
#define CH3_M_FL_Pin     GPIO_PIN_8   // PC8 — Motor 3 (Front Left)
#define CH4_M_FR_Pin     GPIO_PIN_1   // PB1 — Motor 4 (Front Right)
```

### 1.3 Global Variables (main.h externs)

```c
extern int16_t mag_x, mag_y, mag_z;        // Raw magnetometer data (shared across mag drivers)
extern char esp_buffer[100];                 // ESP32 UART receive buffer
extern volatile uint8_t esp_string_ready;    // Flag: complete UART2 line received
extern char usb_buffer[100];                 // USB CDC receive buffer
extern volatile uint8_t usb_string_ready;    // Flag: complete USB line received
```

---

### 1.4 Main Flight Loop (main.c) — 1892 lines

#### Loop Architecture

- **250Hz PID loop** driven by TIM4 ISR (`pid_loop_flag` in `stm32h7xx_it.c`)
- **50Hz sensor polling** (barometer + magnetometer, every 5th loop via `sensor_counter`)
- **5Hz telemetry** output via USB CDC + UART2 (every 50th loop via `print_counter`)
- **GPS parsing** in main loop (event-driven from UART3 RX interrupt)

#### Boot Sequence (`main()` initialization)

1. `MPU_Config()` — Configure MPU (Memory Protection Unit)
2. `HAL_Init()` — Initialize HAL subsystem
3. `SystemClock_Config()` — PLL from HSI → 80MHz (HSI48 for USB)
4. All `MX_*_Init()` functions — Initialize GPIO, SPI1, SPI2, I2C1, UART1-3, TIM3, TIM4, ADC1, USB
5. `Set_Motor_PWM(1100,1100,1100,1100)` — Ensure motors off
6. `HAL_TIM_PWM_Start()` — Start all 4 motor PWM channels
7. `MPU6500_Init()` + `MPU6500_Calibrate()` — Initialize and calibrate IMU (500 samples)
8. I2C scan for magnetometer (addresses 0x0D, 0x1E, 0x2C) → Initialize matching driver
9. `BMP280_Init()` + `BMP280_Calibrate_Altitude()` — Initialize barometer + set zero baseline
10. Start UART interrupts for GPS (UART3) and ESP32 (UART2)
11. `NRF_Init_RX()` — Initialize NRF24L01 as receiver
12. `Flash_Load()` — Load PID gains + mag offsets from internal flash
13. Set `current_state = STATE_DISARMED` — Boot complete
14. `HAL_TIM_Base_Start_IT(&htim4)` — Start 250Hz timer interrupt

#### Flight State Machine

```
STATE_BOOT → STATE_DISARMED ↔ STATE_ARMED → STATE_FAILSAFE
                                  ↕
                            STATE_FAILSAFE
```

| State | Description | LED Behavior |
|-------|-------------|-------------|
| `STATE_BOOT` | Initialization phase | OFF |
| `STATE_DISARMED` | Safe, motors off | OFF |
| `STATE_ARMED` | Flight ready, motors active | Solid ON |
| `STATE_FAILSAFE` | Radio lost | Fast blink (100ms) |

#### Flight Modes

| Mode | Enum | Behavior | Conditions |
|------|------|----------|-----------|
| STABILIZE | `MODE_STABILIZE` | Manual throttle + attitude PID only | Default mode |
| ALT_HOLD | `MODE_ALTHOLD` | Barometer altitude hold + attitude PID | Captures current alt on entry |
| LOITER | `MODE_LOITER` | GPS position hold + altitude hold | Requires GPS fix |
| RTL | `MODE_RTL` | Return to home GPS coordinates | Sets target to `home_lat/lon` |
| AUTO | `MODE_AUTO` | Fly to `wp_lat`/`wp_lon` waypoint | Single WP or survey mission |

#### Arming Logic

Three arming methods:
1. **Stick command**: Throttle down + Yaw right > 400 for 1 second
2. **Dashboard command**: `A,1` via USB/UART
3. **Hardware button**: PE3 (BTN_K1) with 500ms debounce

All methods require `base_throttle ≤ 1120` (throttle at zero).

On arm:
- `setpoint_yaw` = current yaw (prevents yaw snap)
- `home_lat/lon` = current GPS position

Disarming:
1. **Stick command**: Throttle down + Yaw left < -400 for 1 second
2. **Dashboard command**: `A,0`
3. **Hardware button**: Same button toggles

#### Safety Systems

| System | Trigger | Action |
|--------|---------|--------|
| Radio Failsafe | No NRF packets for 500ms | If airborne + GPS → RTL; No GPS → AltHold descend (0.5m/s); Idling → Disarm |
| Dashboard Failsafe | No heartbeat for 3s | Abort AUTO mode → RTL |
| Geofence (soft) | >500m from home | Force RTL |
| Geofence (hard) | >1000m from home | Instant disarm (hard kill) |
| Crash Detection | >45° tilt while armed | Instant disarm |
| Stalled Motor | >45° tilt + throttle >1500 for 2s | Instant disarm |
| Low Battery | <10.2V (3S LiPo) | If idling → disarm; if airborne → AltHold descend |
| GPS Loss | GPS lost in autonomous mode | Fallback to AltHold, begin descent |

#### Motor Mixing (Quad-X, Props In)

```
      FRONT
   FL(m3)  FR(m4)
     CW    CCW
   
   BL(m1)  BR(m2)
    CCW     CW
      REAR

CH1 (m1) = Rear Left  (CCW): throttle + roll - pitch + yaw
CH2 (m2) = Rear Right (CW):  throttle - roll - pitch - yaw
CH3 (m3) = Front Left (CW):  throttle + roll + pitch - yaw
CH4 (m4) = Front Right(CCW): throttle - roll + pitch + yaw
```

**AirMode**: If any motor drops below idle speed (1150µs), all motors are boosted equally to keep the lowest at 1150µs. This preserves PID rotational authority at zero throttle.

**Upper limit**: All motors clamped to 1940µs (leaves 60µs headroom below ESC max).

#### Throttle Mapping

```
Raw NRF throttle (1000-2000) → mapped_throttle:
  ≤ 1120: Zero throttle, motors at 1100 (off)
  > 1120: Linear ramp from 1150 to 2000

TPA (Throttle PID Attenuation):
  Below 1500: tpa_factor = 1.0 (full PID)
  1500-2000:  Linear reduction, max 30% reduction at full throttle
  Safety clamp: tpa_factor ≥ 0.1
```

#### Magnetometer Heading (`Get_Mag_Heading`)

1. Apply hard-iron calibration offsets (`mag_offset_x/y/z`)
2. Get current roll/pitch from IMU for tilt compensation
3. Compute tilt-compensated horizontal magnetic components:
   - `X_h = x*cos_p + y*sin_r*sin_p + z*cos_r*sin_p`
   - `Y_h = y*cos_r - z*sin_r`
4. `heading = atan2(Y_h, X_h)`
5. Apply magnetic declination for Katubedda, Sri Lanka: -1° 56' (-1.9333°)
6. Normalize to 0-360°

#### Magnetometer Averaging (Survey)

Every 50Hz mag read, raw values are accumulated into `mag_sum_x/y/z` and `mag_avg_count`. At the 5Hz telemetry output, averages are computed and sent as `amx/amy/amz` fields. This provides 10-sample averaging for survey noise reduction without affecting flight-critical heading computation.

#### I2C Recovery

If 5 consecutive I2C errors occur (`i2c_error_count >= 5`):
1. Disable I2C peripheral
2. Toggle SCL 9 times as GPIO to force slave to release SDA
3. Send manual STOP condition
4. Re-initialize I2C
5. Re-initialize the active magnetometer driver

#### UART Handling

Both GPS (UART3) and ESP32 (UART2) use interrupt-driven single-byte reception:
- Characters accumulate in `gps_buffer[]` / `esp_buffer[]`
- On `\n`, the complete line is copied to a parse buffer (`gps_parse_buffer` / `esp_parse_buffer`)
- A `_string_ready` flag is set for main loop processing
- A `_parsing` lock prevents overwriting during active parsing
- `HAL_UART_ErrorCallback` restarts reception after ORE (overrun) errors

USB CDC commands share the same parsing path via `usb_buffer`.

---

### 1.5 PID Controller (pid.c / pid.h)

#### Data Structure

```c
typedef struct {
    float Kp;              // Proportional gain
    float Ki;              // Integral gain
    float Kd;              // Derivative gain
    float Kf;              // Feedforward gain
    float integral;        // Accumulated integral
    float prev_measured;   // Previous measurement (for D-on-measurement)
    float prev_setpoint;   // Previous setpoint (for feedforward)
    float prev_derivative; // Previous D-term (for EMA low-pass)
    float out_max;         // Output upper clamp
    float out_min;         // Output lower clamp
} PID_Controller;
```

#### PID Instances

| Controller | P | I | D | F | Output Range | Purpose |
|-----------|---|---|---|---|-------------|---------|
| `pid_roll` | 0.50 | 0.00 | 0.005 | 0.00 | ±200 | Roll stabilization |
| `pid_pitch` | 0.50 | 0.00 | 0.005 | 0.00 | ±200 | Pitch stabilization |
| `pid_yaw` | 0.50 | 0.00 | 0.005 | 0.00 | ±200 | Yaw rate (angular) |
| `pid_alt` | 50.0 | 10.0 | 5.0 | 0.00 | -200 to +300 | Altitude → throttle |
| `pid_gps_pitch` | 0.05 | 0.00 | 0.02 | 0.00 | ±20° | GPS nav → pitch angle |
| `pid_gps_roll` | 0.05 | 0.00 | 0.02 | 0.00 | ±20° | GPS nav → roll angle |

#### Functions

**`float PID_Compute(PID_Controller *pid, float setpoint, float measured, float dt, float tpa_factor)`**

Standard PID with:
1. **P-term**: `(Kp * tpa_factor) * error`
2. **D-on-measurement**: `-(measured - prev_measured) / dt`, prevents derivative kick from setpoint changes
3. **D-term EMA filter**: 30% new / 70% old, prevents high-frequency noise amplification
4. **Feedforward**: `Kf * (setpoint - prev_setpoint) / dt`, instant response to setpoint changes
5. **Dynamic anti-windup**: Conditional integration — skips integral accumulation when output is saturated AND error pushes in the same direction
6. **Hard integral clamp**: ±400 safety net
7. **Output clamping**: `out_min` to `out_max`

**`float PID_Compute_Angular(PID_Controller *pid, float setpoint, float measured, float dt, float tpa_factor)`**

Same as above but handles 360° wrap-around:
- Error calculation: shortest angular path (`if error > 180 → error -= 360`)
- Measurement derivative: wrap-around aware
- Feedforward: wrap-around aware

**`void Reset_PID_Integrals(PID_Controller *pid_roll, PID_Controller *pid_pitch, PID_Controller *pid_yaw)`**

Zeroes the integral accumulators for all three attitude PIDs. Called on:
- Disarm
- Zero throttle while armed
- Ground idle (throttle < 1150)
- Every loop while disarmed (prevents mathematical buildup)

---

### 1.6 IMU — MPU6500 (mpu6500.c / mpu6500.h)

#### Configuration
- **Gyroscope**: 500 dps (register 0x1B = 0x08, sensitivity 65.5 LSB/°/s)
- **Accelerometer**: ±8g (register 0x1C = 0x10, sensitivity 4096 LSB/g)
- **DLPF**: 41Hz hardware low-pass filter (register 0x1A = 0x03)

#### IMU Offset Constants (mpu6500.h)

```c
#define IMU_OFFSET_X  0.0f  // Forward/Backward offset from CoG (meters)
#define IMU_OFFSET_Y  0.0f  // Right/Left offset from CoG (meters)
#define IMU_OFFSET_Z  0.0f  // Up/Down offset from CoG (meters)
```

#### Calibration (`MPU6500_Calibrate`)

Takes 500 samples while stationary:
- Averages gyroscope readings → `GyroX/Y/Z_Offset`
- Computes accelerometer roll/pitch angles → `Accel_Roll/Pitch_Offset`

#### Processing Pipeline (`MPU6500_Read_Angles`)

1. **Raw read**: 14 bytes from register 0x3B (3 × accel + temp + 3 × gyro)
2. **Gyro bias removal**: Subtract calibrated offsets
3. **PT1 gyro low-pass filter**: Dynamic alpha based on dt, ~40Hz cutoff
   - `alpha = dt / (dt + 1/(2π×40))`
4. **Centrifugal force compensation**: For off-center IMU mounting
   - Converts gyro rates to rad/s
   - Computes centrifugal acceleration: `a_c = ω × (ω × r)`
   - Subtracts from accelerometer readings
5. **Accelerometer EMA filter**: 20% new / 80% old
6. **Mahony AHRS quaternion update**: Fuses gyro + accelerometer
7. **Quaternion to Euler**: Roll and Pitch from quaternion
8. **Calibration offset application**: Subtract `Accel_Roll/Pitch_Offset`
9. **Yaw fusion**: If magnetometer available:
   - 98% gyro integration + 2% magnetometer correction (complementary filter)
   - Shortest angular path correction
   - NaN/Infinity guard (resets to 0° if detected)

---

### 1.7 Mahony AHRS Filter (mahony.c / mahony.h)

#### Parameters

```c
#define twoKpDef  (2.0f * 1.0f)    // Proportional gain (Kp = 1.0)
#define twoKiDef  (2.0f * 0.005f)  // Integral gain (Ki = 0.005, corrects gyro drift)
```

#### Algorithm (`MahonyAHRSupdateIMU`)

6-DOF quaternion-based AHRS (accelerometer + gyroscope):
1. Convert gyro from deg/s to rad/s
2. Normalize accelerometer vector
3. Estimate gravity direction from quaternion
4. Compute cross-product error between estimated and measured gravity
5. Apply integral feedback (drift correction over time)
6. Apply proportional feedback (immediate correction)
7. Integrate quaternion rate of change (first-order)
8. Normalize quaternion to unit length

Uses fast inverse square root (Quake III algorithm) for performance.

---

### 1.8 Barometer — BMP280 (bmp280.c / bmp280.h)

#### Configuration
- Temperature oversampling: ×2
- Pressure oversampling: ×16
- IIR filter coefficient: 16
- Standby time: 0.5ms
- Mode: Normal (continuous)

#### Functions

| Function | Description |
|----------|-------------|
| `BMP280_Init(hspi)` | Read 24 calibration bytes, configure sensor |
| `BMP280_Read_Data(hspi)` | Read 6 bytes (pressure + temperature), Bosch compensation algorithm, hypsometric altitude formula |
| `BMP280_Calibrate_Altitude(hspi)` | 5 dummy reads + 20-sample average → sets `altitude_offset` for relative 0m |
| `BMP280_GetAltitude()` | Returns EMA-filtered altitude (5% new / 95% old) |
| `BMP280_GetTemp()` | Returns temperature in °C |
| `BMP280_GetPressure()` | Returns pressure in hPa |

#### Altitude Calculation

```
raw_alt = 44330.0 × (1 - (pressure / 1013.25)^0.1903)
altitude = EMA(raw_alt - altitude_offset)
```

---

### 1.9 Magnetometer Drivers (qmc5883l.c / hmc5883l.c)

#### Auto-Detection (main.c I2C scan)

Scans I2C addresses 0x01-0x7F, matches first supported device:

| Address | Sensor | `mag_type` value |
|---------|--------|-----------------|
| 0x0D | QMC5883L | 13 |
| 0x1E | HMC5883L (or disguised QMC) | 30 |
| 0x2C | QMC5883P | 44 |

#### QMC5883L Driver

- **Init**: Soft reset (0x80), Set/Reset period (0x01), Continuous mode 200Hz 8G OSR=512 (0x1D)
- **Read**: Check status register (0x06) bit 0, read 6 bytes LSB-first from 0x00
- Byte order: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB

#### QMC5883P Driver

- **Init**: Soft reset, Control 1 (Normal mode, 50Hz, OSR=4, DSR=2 → 0x55), Control 2 (Range 8G → 0x08)
- **Read**: Check status (0x09) bit 0, read 6 bytes from 0x01 (LSB-first)

#### HMC5883L Driver (with disguised QMC detection)

- **Init**: Reads ID registers 0x0A-0x0C
  - If `'H','4','3'` → Genuine Honeywell: Config A (8-avg, 75Hz → 0x78), Config B (gain → 0x20), Continuous mode
  - Otherwise → Disguised QMC5883L on HMC address: Initialize as QMC5883L
- **Read (Genuine HMC)**: Status register 0x09, read 6 bytes from 0x03 MSB-first
  - Byte order: X_MSB, X_LSB, **Z_MSB, Z_LSB**, Y_MSB, Y_LSB (note Z before Y!)
- **Read (Disguised QMC)**: Same as QMC5883L driver

All drivers write to shared global `mag_x`, `mag_y`, `mag_z` and track `i2c_error_count`.

---

### 1.10 GPS (gps.c / gps.h)

#### Global Variables

```c
float gps_lat, gps_lon;          // Flight-critical position (decimal degrees)
uint8_t gps_fix;                  // Fix type (0=none, 1=GPS, 2=DGPS)
uint8_t gps_satellites;           // Satellite count
float gps_hdop;                   // Horizontal dilution (default 99.0 = no info)
float gps_lat_smooth, gps_lon_smooth; // EMA-smoothed (survey only, NOT used by flight)
float gps_speed;                  // Ground speed (m/s, from RMC)
```

#### NMEA Parsing (`GPS_Parse`)

**GGA sentence** (`$GPGGA` / `$GNGGA`):
1. Validate XOR checksum (between `$` and `*`)
2. Parse fields: lat, lat_dir, lon, lon_dir, fix, satellites, HDOP
3. Convert NMEA DDMM.MMMM → decimal degrees (`NMEA_to_Decimal`)
4. Only update position if `gps_fix > 0` AND coordinates non-zero (prevents zeroing on lost fix)
5. Update EMA-smoothed coordinates: `α = 0.3`, seeded on first fix

**RMC sentence** (`$GPRMC` / `$GNRMC`):
1. Validate checksum
2. Parse field 7: speed over ground (knots)
3. Convert to m/s: `knots × 0.514444`

#### Utility Functions

| Function | Algorithm |
|----------|-----------|
| `GPS_Distance(lat1,lon1,lat2,lon2)` | Haversine formula, returns meters |
| `GPS_Bearing(lat1,lon1,lat2,lon2)` | Initial bearing in degrees (0-360) |

---

### 1.11 NRF24L01 Radio (nrf24.c / nrf24.h)

#### Configuration (Receiver Mode)

| Setting | Value |
|---------|-------|
| Data Rate | 250 kbps (best range) |
| Power | 0 dBm (maximum) |
| Channel | 76 (default Arduino RF24) |
| Address Width | 5 bytes |
| RX Address | `{'0','0','0','0','1'}` |
| Payload Size | 8 bytes (sizeof Data_Package) |
| CRC | 2-byte |
| Auto-ACK | Enabled on Pipe 0 |

#### Data Package Structure

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

#### Signal Strength

Calculated once per second: `nrf_sig_strength = (packets_received / 50) × 100` (max 50 packets/sec = 100%).

---

### 1.12 Flash Storage (flash_store.c / flash_store.h)

#### Storage Structure

```c
#define EEPROM_MAGIC        0xDEADBEE1
#define FLASH_USER_START_ADDR 0x081E0000  // Sector 7, Bank 2

typedef struct {
    uint32_t magic;
    float r_p, r_i, r_d, r_f;      // Roll PID gains
    float p_p, p_i, p_d, p_f;      // Pitch PID gains
    float y_p, y_i, y_d, y_f;      // Yaw PID gains
    float mag_offset_x, mag_offset_y, mag_offset_z;  // Hard-iron offsets
} Flash_Data;
```

#### Flash_Save

1. Unlock flash
2. Erase Sector 7, Bank 2 (voltage range 3)
3. Pad struct to 32-byte boundary (STM32H7 requires 256-bit flash words)
4. Set magic number
5. Write in 32-byte flash words using `HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, ...)`
6. Lock flash

#### Flash_Load

Direct memory-mapped read from `0x081E0000`. Returns 1 if magic matches, 0 otherwise.

---

### 1.13 Auto Survey Module (auto_survey.c / auto_survey.h)

#### Constants

```c
#define SURVEY_MAX_WAYPOINTS 50     // ~400 bytes RAM
#define SURVEY_WP_RADIUS     3.0f   // meters (GPS CEP ≈ 2.5m)
```

#### State Machine

```
SURVEY_IDLE → SURVEY_RUNNING ↔ SURVEY_PAUSED → SURVEY_DONE
                    ↓
              SURVEY_DONE
```

#### Functions

| Function | Description |
|----------|-------------|
| `Survey_Reset()` | Clear all waypoints, reset to IDLE |
| `Survey_AddWaypoint(lat, lon)` | Add to queue, returns 0 if full (>50) |
| `Survey_Start()` | Set current=0, state=RUNNING |
| `Survey_Pause()` | RUNNING → PAUSED |
| `Survey_Resume()` | PAUSED → RUNNING |
| `Survey_Abort()` | Any → IDLE (waypoints preserved) |
| `Survey_Update(lat, lon, *wp_lat, *wp_lon)` | Check distance to current WP, advance if ≤3m, returns 1 if navigating |
| `Survey_GetState()` | Returns current `Survey_State` enum |
| `Survey_GetCurrentIndex()` | Current waypoint index |
| `Survey_GetTotalCount()` | Total loaded waypoints |

#### Integration with main.c

Every 250Hz loop iteration:
- If `SURVEY_RUNNING` and GPS fix: call `Survey_Update()` → updates `wp_lat/wp_lon`
- Ensures `MODE_AUTO` + `gps_hold_active`
- On `SURVEY_DONE`: switch to `MODE_LOITER`, force re-lock position

---

### 1.14 Interrupt Handlers (stm32h7xx_it.c)

```c
TIM4_IRQHandler:    Sets pid_loop_flag = 1 (250Hz)
USART2_IRQHandler:  HAL_UART_IRQHandler → triggers HAL_UART_RxCpltCallback (ESP32)
USART3_IRQHandler:  HAL_UART_IRQHandler → triggers HAL_UART_RxCpltCallback (GPS)
OTG_FS_IRQHandler:  HAL_PCD_IRQHandler → USB CDC
```

---

## 2. Command Protocol (USB / UART2)

Commands sent from dashboard → STM32 as newline-terminated strings:

| Prefix | Format | Action | Conditions |
|--------|--------|--------|-----------|
| `P` | `P,axis,Kp*100,Ki*100,Kd*100,Kf*100` | Update PID gains | axis = roll/pitch/yaw |
| `M` | `M,mode_name` | Change flight mode | stabilize/althold/loiter/rtl |
| `W` | `W,lat,lon` | Single waypoint → AUTO mode | Auto-switches to MODE_AUTO |
| `A` | `A,1` or `A,0` | Arm / Disarm | Arm requires throttle ≤ 1120 |
| `B` | `B` | Burn PIDs + mag offsets to flash | Only when DISARMED |
| `C` | `C,offset_x,offset_y,offset_z` | Set magnetometer offsets | Auto-saves to flash if disarmed |
| `H` | `H` | Dashboard heartbeat | Resets 3s failsafe timer |
| `S` | `S,RESET` | Clear survey waypoint queue | — |
| `S` | `S,WP,lat,lon` | Add waypoint to survey queue | Max 50 waypoints |
| `S` | `S,START` | Begin survey mission | Requires ARMED + waypoints > 0 |
| `S` | `S,PAUSE` | Pause survey → LOITER | — |
| `S` | `S,RESUME` | Resume survey → AUTO | — |
| `S` | `S,ABORT` | Abort survey → LOITER | — |

---

## 3. Telemetry JSON (5Hz, STM32 → Dashboard)

```json
{
  "v": 12.60,        // Battery voltage (V)
  "r": -2.3,         // Roll (degrees)
  "p": 1.5,          // Pitch (degrees)
  "y": 180.0,        // Yaw (degrees)
  "a": 5.2,          // Altitude (meters, barometric)
  "d": 270.0,        // Compass heading (degrees)
  "glat": 6.92710,   // GPS latitude (decimal degrees)
  "glon": 79.86120,  // GPS longitude (decimal degrees)
  "gf": 1,           // GPS fix type (0=none, 1=GPS, 2=DGPS)
  "t": 1500,         // Throttle value (1000-2000)
  "mt": 13,          // Magnetometer type (0x0D=QMC, 0x1E=HMC, 0x2C=QMC-P)
  "pid_r": [0.50, 0.00, 0.01, 0.00],  // Roll PID [P, I, D, F]
  "pid_p": [0.50, 0.00, 0.01, 0.00],  // Pitch PID [P, I, D, F]
  "pid_y": [0.50, 0.00, 0.01, 0.00],  // Yaw PID [P, I, D, F]
  "md": 0,           // Flight mode (0=STAB, 1=ALT, 2=LOIT, 3=RTL, 4=AUTO)
  "mx": 150,         // Raw mag X (LSB)
  "my": -200,        // Raw mag Y (LSB)
  "mz": 400,         // Raw mag Z (LSB)
  "ry": 0, "rp": 0, "rr": 0,  // Raw NRF joystick values
  "arm": 1,          // Armed state (0/1)
  "sig": 85,         // NRF signal strength (0-100%)
  "m1": 1500, "m2": 1500, "m3": 1500, "m4": 1500,  // Motor PWM values
  "gsat": 8,         // GPS satellites
  "ghdop": 1.2,      // GPS HDOP
  "gspd": 2.5,       // GPS ground speed (m/s)
  "slat": 6.92711,   // EMA-smoothed latitude (survey only)
  "slon": 79.86121,  // EMA-smoothed longitude (survey only)
  "amx": 148, "amy": -199, "amz": 401,  // Averaged mag readings (10-sample)
  "swp": 5,          // Survey: current waypoint index
  "swt": 24,         // Survey: total waypoint count
  "sst": 1           // Survey: state (0=idle, 1=running, 2=paused, 3=done)
}
```

---

## 4. ESP32 Remote Controller (ESP32_Remote.ino)

### Hardware

| Component | Connection | Purpose |
|-----------|-----------|---------|
| ADS1115 | I2C (SDA=21, SCL=22) | 4-channel 16-bit joystick ADC |
| NRF24L01 | SPI (SCK=18, MISO=19, MOSI=23, CE=4, CSN=5) | 2.4GHz radio TX |
| SSD1306 OLED | I2C (0x3C) | Live display of throttle/yaw/pitch/roll |

### Configuration

```c
#define ADC_MIN 0         // Joystick ADC minimum
#define ADC_MAX 21500     // Joystick ADC maximum
#define ADC_YAW_MID 13115 // Center calibration for each axis
#define ADC_PITCH_MID 13115
#define ADC_ROLL_MID 13115
```

### ADC Channel Mapping

| ADS1115 Channel | Axis | Output Range |
|----------------|------|-------------|
| A0 | Yaw (Vrx) | -500 to +500 |
| A1 | Throttle (Vry) | 1000 to 2000 |
| A2 | Pitch (Vry) | -500 to +500 |
| A3 | Roll (Vrx) | -500 to +500 |

### Joystick Mapping (`map_joystick`)

Custom midpoint-aware mapping with ±25 deadband:
- Below midpoint: linear map from -500 to 0
- Above midpoint: linear map from 0 to +500
- Within ±25 of center: forced to 0

### Timing

- **NRF TX**: 50Hz (20ms interval)
- **OLED update**: 4Hz (250ms interval, prevents I2C bottleneck)
- **ADS1115**: 860 SPS (eliminates blocking read delays)

### Radio Config

```c
radio.setPALevel(RF24_PA_MAX);        // Maximum power
radio.setDataRate(RF24_250KBPS);      // Best range
radio.setChannel(76);                  // Matches STM32
radio.setPayloadSize(sizeof(Data_Package)); // 8 bytes
```

---

## 5. ESP32 Telemetry Bridge (ESP32_Telemetry.ino)

### WiFi Configuration

```c
const char* ssid = "SLT-Fiber-2.4G";
const char* password = "Nimsara1";
const char* server_ip = "172.20.10.8";
const uint16_t server_port = 5000;
```

### UART Configuration

```c
#define RXD2 18  // ESP32 D18 ← STM32 PA3 (TX)
#define TXD2 17  // ESP32 D17 → STM32 PA2 (RX)
// 115200 baud, 8N1
```

### Data Flow

```
STM32 UART2 TX → ESP32 RX (D18) → WiFi TCP → Node.js server
Node.js server → WiFi TCP → ESP32 TX (D17) → STM32 UART2 RX
```

### Buffer Protection

- 512-byte maximum per accumulated line
- On overflow: buffer is cleared (prevents OOM crash from dropped newlines)

### Connection Management

- Auto-reconnect WiFi on disconnect
- Auto-reconnect TCP on disconnect (2-second retry interval)
- Always drains UART even when disconnected (prevents STM32 buffer overflow)

---

## 6. Web Dashboard — Server (server.js)

### Dependencies

```json
{
  "express": "^4.18.2",
  "serialport": "^13.0.0",
  "socket.io": "^4.6.1"
}
```

### Ports

| Port | Protocol | Purpose |
|------|---------|---------|
| 3000 | HTTP | Web dashboard (Express static files) |
| 5000 | TCP | ESP32 telemetry connections |

### Multi-Drone Fleet Architecture

```
┌────────────────────────────────┐
│      droneRegistry (Map)       │
│  droneId → {socket, telemetry, │
│             connectionType,    │
│             lastSeen}          │
├────────────────────────────────┤
│   pendingConnections (Array)   │
│  Unnamed TCP connections       │
│  waiting to be assigned names  │
└────────────────────────────────┘
```

### Connection Types

1. **WiFi (TCP)**: ESP32 connects to port 5000 → goes to `pendingConnections` → admin assigns a name → moves to `droneRegistry`
2. **USB Serial**: Auto-detected by STM32 Vendor ID `0483` → auto-registered as `"USB-Direct"` with default PIN `0000`

### Authentication

1. **Fleet Admin**: Password-based (`admin123`). Generates a UUID token stored in `fleetTokens` Set. Required for assigning/removing drones.
2. **Drone Access**: PIN-based per drone (stored in `dronePins` Map). Generates a UUID token stored in `droneTokens` Map. Required for sending commands.
3. **Heartbeat**: Exempt from auth (keeps connection alive silently)

### WebSocket Events

**Fleet Management:**
| Event | Direction | Description |
|-------|----------|-------------|
| `fleet:status` | Server → Client | Current fleet state (2s broadcast) |
| `fleet:assign` | Client → Server | Name a pending connection |
| `fleet:remove` | Client → Server | Remove a drone |
| `fleet:auto_assign` | Client → Server | Auto-name all pending |
| `fleet:error` | Server → Client | Error messages |

**Drone Telemetry:**
| Event | Direction | Description |
|-------|----------|-------------|
| `drone:telemetry` | Server → Client | Tagged telemetry `{droneId, data}` |
| `telemetry` | Server → Client | Legacy single-drone (backward compat) |

**Drone Commands:**
| Event | Direction | Description |
|-------|----------|-------------|
| `drone:command` | Client → Server | Routed command `{droneId, type, payload, droneToken}` |
| `drone:auth_error` | Server → Client | Unauthorized command |

**Legacy Events** (backward compatibility): `tune_pid`, `send_waypoint`, `set_mode`, `save_pid`, `toggle_arm`, `calibrate_mag`, `heartbeat`, `survey_*` — all route to first drone in registry.

### REST API

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/login/fleet` | POST | Fleet admin login |
| `/api/login/drone` | POST | Drone PIN login |
| `/api/fleet` | GET | Current fleet status |

---

## 7. Web Dashboard — Command Dashboard (index.html + script.js)

### Features

1. **3D Attitude Indicator** (Three.js)
   - Full 3D drone model with body, arms, motors, propeller discs, blades, landing gear, LEDs
   - Real-time roll/pitch/yaw from telemetry
   - Animated spinning propellers
   - LED color changes: cyan (disarmed) → red (armed)

2. **Live Telemetry Chart** (Chart.js)
   - 4 datasets: Roll, Pitch, Yaw, Altitude
   - 50-point sliding window
   - Throttled rendering: 1Hz to prevent UI freezing

3. **PID Tuning Panels**
   - 3 axes × 4 gains (P, I, D, F) = 12 sliders
   - Auto-populated from first telemetry packet
   - "Current" readout showing drone's actual values
   - Send button per axis → `P,axis,Kp*100,...` command
   - Burn to Flash button → `B` command (disarmed only)

4. **Magnetometer 3D Calibration** (Plotly scatter3d)
   - Accumulates raw mag X/Y/Z points (max 300)
   - Sphere visualization for hard-iron calibration
   - Calculate button: min/max method → `(max + min) / 2` per axis
   - Auto-sends offsets to drone + burns to flash

5. **GPS Waypoint Map** (Leaflet)
   - Click to place waypoint
   - Drone position marker (auto-updated)
   - Home position marker (set on arm)
   - Dashed flight path line from drone to waypoint
   - Distance calculation (Haversine)
   - Manual lat/lon input with map sync
   - "Fly to Waypoint" button with safety confirmation

6. **Motor Output Bars**
   - 4 visual bars with PWM values (1100-2000)
   - Color coding: green (normal), yellow (>50%), red (>80%)

7. **Status Indicators**
   - Battery voltage with low-battery warning (<10.2V)
   - NRF signal strength (green/orange/red)
   - GPS fix status
   - Flight mode selector (synced with telemetry)
   - ARM/DISARM buttons with safety confirmation

8. **RC Transmitter Display**
   - 4 progress bars showing raw NRF joystick values
   - Throttle, Yaw, Pitch, Roll with numeric readouts

### Multi-Drone Awareness

- URL parameter `?drone=DroneID` activates fleet mode
- All commands routed to specific drone via `drone:command` event
- Telemetry filtered by `droneId`
- Requires drone PIN authentication (redirects to `drone_login.html`)
- Drone name badge displayed in header

---

## 8. Web Dashboard — Survey Dashboard (survey.html + survey.js)

### Features

1. **Full-Screen Satellite Map** (Leaflet + ESRI tiles)
   - Dark mode toggle for survey ("scanner mode" removes satellite tiles)
   - GPS quality badge (EXCELLENT/GOOD/FAIR/POOR based on HDOP/satellites)

2. **Magnetic Anomaly Scanner**
   - Real-time heat map overlay (leaflet.heat)
   - Gradient: blue → green → yellow → red
   - Adaptive distance threshold based on HDOP

3. **Baseline Calibration**
   - **Manual**: One-click captures current TMI (Total Magnetic Intensity)
   - **Auto**: 10-second median sampling (robust against outliers)

4. **Data Quality Pipeline**
   - **GPS gating**: Reject points with HDOP > 2.5 or satellites < 5
   - **Speed filtering**: Reject if speed < 0.3 m/s (stationary) or > 15 m/s (too fast)
   - **Survey line detection**: Track heading rate of change; reject data during turns (threshold: 8°/update)
   - **Turn reject counter**: Tracks rejected points

5. **Geophysics Processing**
   - **Altitude normalization**: Inverse cube law correction (`B_norm = B_meas × (h/h_ref)³`)
   - **Diurnal drift correction**: Linear regression on 10-minute rolling window, compensates Earth's field drift (~30-50 nT/hour)
   - **Fourth-difference noise estimation**: Standard geophysics QC metric, RMS of FD values / 6.72
   - **Spatial gradient computation**: `ΔB/Δdistance` (µT/m) between consecutive points
   - **LSB to µT conversion**: Based on sensor type (QMC: 3000 LSB/Gauss, HMC: 1090 LSB/Gauss)

6. **Visualization**
   - **Anomaly heatmap**: Color-coded by intensity (threshold: 5µT, max: 50µT)
   - **Gradient heatmap**: Separate layer, purple-pink gradient (threshold: 0.1 µT/m)
   - **Grid coverage overlay**: 5m × 5m cells, color by measurement density
   - **2D heatmap** (Plotly): Quality-weighted IDW interpolation
   - **3D surface plot** (Plotly): Same data as 3D surface
   - Client-side rolling average: 5-sample window on top of firmware 10-sample averaging

7. **Lawnmower Mission Planner**
   - Click two corners on map to define survey rectangle
   - Configurable line spacing (default 5m)
   - Generates boustrophedon (alternating direction) waypoints
   - Maximum 50 waypoints (matches STM32 queue size)
   - Upload sequence: RESET → send all WPs (50ms delay each) → START
   - Live controls: Start → Pause/Resume → Abort
   - Progress tracking: waypoint counter + progress bar
   - State display: IDLE / FLYING / PAUSED / DONE

8. **CSV Export**
   - Columns: Latitude, Longitude, Raw_Anomaly, Corrected_Anomaly, Gradient, Altitude, HDOP, Speed, Heading, Flight_Segment, Timestamp
   - Metadata footer: baseline, reference altitude, drift rate, noise level, point count, grid cells, turn rejects

9. **Survey Statistics Panel**
   - Data points collected
   - Survey area (m² or hectares)
   - Elapsed time
   - Noise level with quality indicator
   - Diurnal drift rate (µT/hour)
   - Survey line / turning indicator
   - Grid cell coverage percentage

### Multi-Drone Awareness

Same pattern as Command Dashboard: URL parameter `?drone=DroneID`, authenticated commands via `emitSurveyCommand()`.

---

## 9. Web Dashboard — Fleet Dashboard (fleet.html + fleet.js)

### Features

1. **Fleet Overview**: Grid of drone cards showing status, voltage, GPS fix, connection type
2. **Pending Connections Pool**: Shows unnamed TCP connections with preview telemetry
3. **Drone Assignment**: Name a pending connection + set PIN
4. **Auto-Assign**: Automatically name all pending connections with sequential IDs
5. **Connection Status**: Online/offline detection (5-second timeout)
6. **Navigation**: Click any drone card → redirects to Command or Survey dashboard with `?drone=` parameter

### Authentication Flow

1. User visits `/fleet.html` → redirected to `/login.html`
2. Enter fleet password (`admin123`)
3. POST `/api/login/fleet` → receives UUID token
4. Token stored in `localStorage` as `fleetToken`
5. All fleet management operations include token for verification

### Per-Drone Auth Flow

1. User clicks drone card → redirected to `/drone_login.html?drone=DroneID`
2. Enter drone PIN
3. POST `/api/login/drone` → receives UUID token
4. Token stored as `localStorage['droneToken_DroneID']`
5. Redirected to target page (`/index.html?drone=DroneID` or `/survey.html?drone=DroneID`)

---

## 10. NRF24L01 Data Package

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

This exact structure is defined identically in:
- `ESP32_Remote.ino` (transmitter)
- `nrf24.h` (STM32 receiver)

Both use `#pragma pack(push, 1)` to ensure no padding bytes.

---

## 11. RC Input Processing (main.c)

### Joystick → Setpoint Mapping

```
Pitch/Roll: joy_value × (30.0 / 500.0) = ±30° max tilt
Yaw: joy_value × (90.0 / 500.0) × dt = max 90°/sec heading change
Throttle: Direct passthrough (1000-2000)
```

### RC Smoothing

Exponential Moving Average on pitch/roll setpoints:
```c
float rc_alpha = 0.15f;  // 15% new, 85% old
setpoint_pitch += rc_alpha * (target_pitch - setpoint_pitch);
```

Prevents derivative kick from sharp stick movements.

### Deadband

±20 on pitch, roll, yaw after mapping but before smoothing. Prevents joystick drift.

---

## 12. GPS Navigation (main.c)

### Position Hold (LOITER / AUTO mode)

1. Calculate distance and bearing from current position to target
2. Convert to relative angle (bearing - compass heading)
3. Decompose into X (forward/pitch) and Y (right/roll) velocity vectors
4. Cap velocity vectors at ±400
5. Feed through `pid_gps_pitch` and `pid_gps_roll`
6. Output overrides `setpoint_pitch` and `setpoint_roll` (replaces joystick input)

### Altitude Hold

1. On first activation, capture current throttle as `hover_throttle`
2. `pid_alt` computes correction: `PID(target_altitude, current_altitude)`
3. `throttle_output = hover_throttle + alt_correction`

---

## 13. Build & Run

### STM32 Firmware

Build with STM32CubeIDE (GCC ARM):
```
Project file: UAV Template/UAV Template/.cproject
Linker script: STM32H743VITX_FLASH.ld
```

### ESP32 Projects

Build with Arduino IDE with:
- ESP32 board support package
- Libraries: `RF24`, `ADS1X15`, `SSD1306`, `GFX`

### Web Dashboard

```bash
cd Telemetry_Web_App
npm install
npm start
# Dashboard: http://localhost:3000
# TCP port:  5000 (ESP32 drones)
```
