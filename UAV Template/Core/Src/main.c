/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h" // USB හරහා දත්ත යැවීමට අවශ්‍ය file එක
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#include "bmp280.h"
#include "flash_store.h"
#include "gps.h"
#include "hmc5883l.h"
#include "motors.h"
#include "mpu6500.h"
#include "nrf24.h"
#include "pid.h"
#include "qmc5883l.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
char uart_buf[512]; // UART එකෙන් Serial Monitor එකට Print කරන්න

// GPS UART Variables
uint8_t gps_rx_data;
char gps_buffer[100];
uint8_t gps_buf_index = 0;

// ESP32 UART Variables (PID Tuning)
uint8_t esp_rx_data;
char esp_buffer[100];
uint8_t esp_buf_index = 0;
volatile uint8_t esp_string_ready = 0;

// Magnetometer Type and Data
uint8_t mag_type = 0; // 0 = None, 0x0D = QMC5883L, 0x1E = HMC5883L
int16_t mag_x = 0, mag_y = 0, mag_z = 0;
float mag_offset_x = 0.0f, mag_offset_y = 0.0f, mag_offset_z = 0.0f;

float Get_Mag_Heading(void) {
  if (mag_type == 0)
    return 0.0f;

  // Apply Hard Iron Offsets (Calibration)
  float x = (float)mag_x - mag_offset_x;
  float y = (float)mag_y - mag_offset_y;

  float heading_rad = atan2f(y, x);

  // Magnetic Declination can be added here
  // heading_rad += declination;

  if (heading_rad < 0)
    heading_rad += 2 * M_PI;
  if (heading_rad > 2 * M_PI)
    heading_rad -= 2 * M_PI;

  return heading_rad * (180.0f / M_PI);
}

// Timing variables
uint32_t current_time;
uint16_t prev_time_us;
float dt;

// PID Controllers (Attitude)
PID_Controller pid_roll = {0.6f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 400.0f, -400.0f};
PID_Controller pid_pitch = {0.6f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 400.0f, -400.0f};
PID_Controller pid_yaw = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 400.0f, -400.0f};

// PID Controllers (Altitude and GPS)
PID_Controller pid_alt = {1.5f, 0.5f,   0.1f,   0.0f, 0.0f,
                          0.0f, 0.0f, 0.0f, 300.0f, -200.0f}; // Output is throttle override
PID_Controller pid_gps_pitch = {
    0.05f, 0.0f,  0.02f, 0.0f, 0.0f,
    0.0f,  0.0f, 0.0f, 20.0f, -20.0f}; // Output is pitch angle (max 20 deg)
PID_Controller pid_gps_roll = {
    0.05f, 0.0f,  0.02f, 0.0f, 0.0f,
    0.0f,  0.0f, 0.0f, 20.0f, -20.0f}; // Output is roll angle (max 20 deg)

// Navigation State Variables
uint8_t alt_hold_active = 0;
float target_altitude = 0.0f;

uint8_t gps_hold_active = 0;
float target_gps_lat = 0.0f;
float target_gps_lon = 0.0f;

// Motor PWM variables
uint16_t motor1, motor2, motor3, motor4;

// Setpoints (can be updated from NRF24 later)
float setpoint_roll = 0.0f;
float setpoint_pitch = 0.0f;
float setpoint_yaw = 0.0f;
float base_throttle = 1200.0f; // TEST MODE: Motors will spin at a slow idle

// Flight State Machine
typedef enum {
  STATE_BOOT,
  STATE_DISARMED,
  STATE_ARMED,
  STATE_FAILSAFE
} Drone_State;

Drone_State current_state = STATE_BOOT;

typedef enum {
  MODE_STABILIZE,
  MODE_ALTHOLD,
  MODE_LOITER,
  MODE_RTL,
  MODE_AUTO
} Flight_Mode;

Flight_Mode current_mode = MODE_STABILIZE;

// Home and Waypoint Coordinates
float home_lat = 0.0f;
float home_lon = 0.0f;
float wp_lat = 0.0f;
float wp_lon = 0.0f;

uint32_t last_button_press = 0; // For debounce

// NRF Radio Variables
struct Data_Package nrf_data;
uint32_t last_radio_time = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI2_Init(void);
static void BareMetal_ADC1_Init(void);
static uint16_t BareMetal_ADC1_Read(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

volatile uint8_t gps_string_ready = 0;
volatile uint8_t gps_parsing = 0;
char gps_parse_buffer[100];

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// --- JOYSTICK CALIBRATION SETTINGS ---
// If your transmitter sends 0-1023, the middle is usually around 512.
// Adjust the _MID values to whatever your transmitter sends when the sticks are
// perfectly centered.
#define JOY_PITCH_MIN 0
#define JOY_PITCH_MID 626
#define JOY_PITCH_MAX 1023

#define JOY_ROLL_MIN 0
#define JOY_ROLL_MID 625
#define JOY_ROLL_MAX 1023

#define JOY_YAW_MIN 0
#define JOY_YAW_MID 625
#define JOY_YAW_MAX 1023

// Helper function to map joystick values so the exact middle is 0
int16_t map_joystick(int16_t val, int16_t min_val, int16_t mid_val,
                     int16_t max_val) {
  if (val <= mid_val) {
    if (mid_val == min_val)
      return 0; // Prevent divide by zero and extreme inputs
    return (int16_t)((((float)(val - min_val) / (mid_val - min_val)) * 500.0f) -
                     500.0f);
  } else {
    if (max_val == mid_val)
      return 0; // Prevent divide by zero and extreme inputs
    return (int16_t)(((float)(val - mid_val) / (max_val - mid_val)) * 500.0f);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART3) {
    if (gps_rx_data != '\n' && gps_rx_data != '\r' && gps_buf_index < 99) {
      gps_buffer[gps_buf_index++] = gps_rx_data;
    } else if (gps_rx_data == '\n') {
      gps_buffer[gps_buf_index] = '\0';
      // Safely hand off to main loop (only if it's not currently parsing the
      // previous one!)
      if (!gps_parsing) {
        strcpy(gps_parse_buffer, gps_buffer);
        gps_string_ready = 1;
      }
      gps_buf_index = 0;
    }
    HAL_UART_Receive_IT(&huart3, &gps_rx_data, 1);
  } else if (huart->Instance == USART2) {
    if (esp_rx_data != '\n' && esp_rx_data != '\r' && esp_buf_index < 99) {
      esp_buffer[esp_buf_index++] = esp_rx_data;
    } else if (esp_rx_data == '\n') {
      esp_buffer[esp_buf_index] = '\0';
      esp_string_ready = 1;
      esp_buf_index = 0;
    }
    HAL_UART_Receive_IT(&huart2, &esp_rx_data, 1);
  }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  // BareMetal_ADC1_Init(); // TEMPORARILY DISABLED to ensure it doesn't halt the boot sequence
  /* USER CODE BEGIN 2 */

  // 1. Start Motor PWM Timers
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  // (ESC Calibration Sequence Removed)
  // Normal startup: set motors to completely OFF (1000us) to ensure safety.
  Set_Motor_PWM(1000, 1000, 1000, 1000);

  // 2. Initialize MPU6500
  MPU6500_Init(&hspi2);
  HAL_Delay(100);

  // Calibrate MPU6500 (Ensure drone is stationary)
  MPU6500_Calibrate(&hspi2);

  // Full I2C Scanner to definitively find the Magnetometer
  for (uint8_t i = 1; i < 128; i++) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 3, 100) == HAL_OK) {
      if (i == 0x0D || i == 0x1E || i == 0x2C) {
        mag_type = i; // Save the raw hex address
        break;        // Stop at first supported device found
      }
    }
  }

  if (mag_type == 0x0D) {
    QMC5883L_Init(&hi2c1);
  } else if (mag_type == 0x1E) {
    HMC5883L_Init(&hi2c1);
  } else if (mag_type == 0x2C) {
    QMC5883P_Init(&hi2c1);
  }

  BMP280_Init(&hspi2);

  // Calibrate Altitude Baseline
  BMP280_Calibrate_Altitude(&hspi2);

  // Start UART Receptions
  HAL_UART_Receive_IT(&huart3, &gps_rx_data, 1);
  HAL_UART_Receive_IT(&huart2, &esp_rx_data, 1);

  // Initialize NRF24L01 Radio Receiver
  NRF_Init_RX();
  last_radio_time = HAL_GetTick(); // Prevent instant failsafe

  // --- LOAD FROM FLASH MEMORY ---
  Flash_Data flash_mem;
  if (Flash_Load(&flash_mem)) {
    // Apply saved PID values
    pid_roll.Kp = flash_mem.r_p;
    pid_roll.Ki = flash_mem.r_i;
    pid_roll.Kd = flash_mem.r_d;
    pid_roll.Kf = flash_mem.r_f;
    pid_pitch.Kp = flash_mem.p_p;
    pid_pitch.Ki = flash_mem.p_i;
    pid_pitch.Kd = flash_mem.p_d;
    pid_pitch.Kf = flash_mem.p_f;
    pid_yaw.Kp = flash_mem.y_p;
    pid_yaw.Ki = flash_mem.y_i;
    pid_yaw.Kd = flash_mem.y_d;
    pid_yaw.Kf = flash_mem.y_f;

    // Apply saved Magnetometer offsets
    mag_offset_x = flash_mem.mag_offset_x;
    mag_offset_y = flash_mem.mag_offset_y;
    mag_offset_z = flash_mem.mag_offset_z;
  }

  current_state = STATE_DISARMED; // Boot sequence finished, drone is safe

  // Initialize timing via ISR
  HAL_TIM_Base_Start_IT(&htim4);
  extern volatile uint8_t pid_loop_flag;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    if (pid_loop_flag) {
      pid_loop_flag = 0;

      current_time = HAL_GetTick();
      dt = 0.004f; // Perfect 250Hz timing guaranteed by ISR

      // 1. Read Battery Voltage Early
      // uint16_t adc_val = BareMetal_ADC1_Read(); // TEMPORARILY DISABLED to prevent infinite loop
      float battery_voltage = 12.6f; // Dummy voltage until ADC is fixed

      // 2. Read sensor data
      MPU6500_Read_Angles(&hspi2, dt);

      float roll_actual = MPU6500_GetRoll();
      float pitch_actual = MPU6500_GetPitch();
      float yaw_actual = MPU6500_GetYaw();
      float alt_actual = BMP280_GetAltitude();

      // --- NRF24L01 RADIO PROCESSING ---
      if (NRF_DataAvailable()) {
        NRF_ReadPayload(&nrf_data, sizeof(nrf_data));
        last_radio_time = current_time;

        // Map raw joystick values to -500 to 500 range IN PLACE so telemetry
        // also gets centered values
        nrf_data.pitch = map_joystick(nrf_data.pitch, JOY_PITCH_MIN,
                                      JOY_PITCH_MID, JOY_PITCH_MAX);
        nrf_data.roll = map_joystick(nrf_data.roll, JOY_ROLL_MIN, JOY_ROLL_MID,
                                     JOY_ROLL_MAX);
        nrf_data.yaw =
            map_joystick(nrf_data.yaw, JOY_YAW_MIN, JOY_YAW_MID, JOY_YAW_MAX);

        int16_t joy_pitch = nrf_data.pitch;
        int16_t joy_roll = nrf_data.roll;
        int16_t joy_yaw = nrf_data.yaw;

        // Add deadbands to mapped joystick inputs (±20) to prevent drift
        if (joy_pitch > -20 && joy_pitch < 20)
          joy_pitch = 0;
        if (joy_roll > -20 && joy_roll < 20)
          joy_roll = 0;
        if (joy_yaw > -20 && joy_yaw < 20)
          joy_yaw = 0;

        // Map NRF Joystick Data to Target Angles
        float target_pitch = (float)joy_pitch * (30.0f / 500.0f);
        float target_roll = (float)joy_roll * (30.0f / 500.0f);

        // RC Smoothing (Exponential Moving Average) for Cinematic Flight
        // Prevents 'Derivative Kick' on sharp stick movements
        float rc_alpha = 0.15f;
        setpoint_pitch += rc_alpha * (target_pitch - setpoint_pitch);
        setpoint_roll += rc_alpha * (target_roll - setpoint_roll);

        // Joystick updates target heading (Max 90 deg/sec)
        setpoint_yaw += (float)joy_yaw * (90.0f / 500.0f) * dt;
        if (setpoint_yaw >= 360.0f)
          setpoint_yaw -= 360.0f;
        if (setpoint_yaw < 0.0f)
          setpoint_yaw += 360.0f;

        // Map Throttle
        base_throttle = (float)nrf_data.throttle;

        // --- FLIGHT MODE LOGIC & GEOFENCING ---
        if (current_state == STATE_ARMED) {
          // Geofence & GPS-Loss Checks
          if (gps_fix > 0 && home_lat != 0.0f) {
            float dist_from_home =
                GPS_Distance(home_lat, home_lon, gps_lat, gps_lon);

            if (dist_from_home > 1000.0f) {
              current_state = STATE_DISARMED; // HARD KILL: Runaway drone!
            } else if (dist_from_home > 500.0f && current_mode != MODE_RTL) {
              current_mode = MODE_RTL; // GEOFENCE BREACH: FORCE RTL!
            }
          } else {
            // GPS LOST! If flying autonomous modes, abort to AltHold
            // soft-landing
            if (current_mode == MODE_AUTO || current_mode == MODE_RTL ||
                current_mode == MODE_LOITER) {
              current_mode = MODE_ALTHOLD;
              target_altitude -= 0.5f * dt; // Start descending
            }
          }

          if (current_mode == MODE_STABILIZE) {
            alt_hold_active = 0;
            gps_hold_active = 0;
          } else if (current_mode == MODE_ALTHOLD) {
            if (!alt_hold_active) {
              alt_hold_active = 1;
              target_altitude = alt_actual;
            }
            gps_hold_active = 0;
          } else if (current_mode == MODE_LOITER) {
            if (!alt_hold_active) {
              alt_hold_active = 1;
              target_altitude = alt_actual;
            }
            if (!gps_hold_active) {
              gps_hold_active = 1;
              target_gps_lat = gps_lat;
              target_gps_lon = gps_lon;
              pid_gps_pitch.integral = 0.0f;
              pid_gps_roll.integral = 0.0f;
            }
          } else if (current_mode == MODE_RTL) {
            if (!alt_hold_active) {
              alt_hold_active = 1;
              target_altitude = alt_actual;
            }
            if (!gps_hold_active || target_gps_lat != home_lat) {
              gps_hold_active = 1;
              target_gps_lat = home_lat;
              target_gps_lon = home_lon;
              pid_gps_pitch.integral = 0.0f;
              pid_gps_roll.integral = 0.0f;
            }
          } else if (current_mode == MODE_AUTO) {
            if (!alt_hold_active) {
              alt_hold_active = 1;
              target_altitude = alt_actual;
            }
            if (!gps_hold_active || target_gps_lat != wp_lat) {
              gps_hold_active = 1;
              target_gps_lat = wp_lat;
              target_gps_lon = wp_lon;
              pid_gps_pitch.integral = 0.0f;
              pid_gps_roll.integral = 0.0f;
            }
          }
        } else {
          alt_hold_active = 0;
          gps_hold_active = 0;
        }
      }

      // --- POLLED SENSORS (Barometer & Compass) ---
      static uint8_t sensor_counter = 0;
      sensor_counter++;
      if (sensor_counter >= 5) { // 50Hz (every 5 loops of 250Hz)
        sensor_counter = 0;
        if (mag_type == 0x0D)
          QMC5883L_Read(&hi2c1);
        else if (mag_type == 0x1E)
          HMC5883L_Read(&hi2c1);
        else if (mag_type == 0x2C)
          QMC5883P_Read(&hi2c1);
        BMP280_Read_Data(&hspi2);
      }

      // --- GPS PARSING (Low Priority) ---
      if (gps_string_ready) {
        gps_parsing = 1; // LOCK buffer from interrupt
        gps_string_ready = 0;
        GPS_Parse(gps_parse_buffer);
        gps_parsing = 0; // UNLOCK buffer
      }

      // --- ESP32 INCOMING COMMANDS (PID Tuning, Modes, Waypoints) ---
      if (esp_string_ready) {
        esp_string_ready = 0;

        // Format P: P,roll,1.20,0.05,0.01,0.00
        if (esp_buffer[0] == 'P') {
          char* token = strtok((char*)esp_buffer, ",");
          if (token != NULL && token[0] == 'P') {
            char* axis = strtok(NULL, ",");
            char* p_str = strtok(NULL, ",");
            char* i_str = strtok(NULL, ",");
            char* d_str = strtok(NULL, ",");
            char* f_str = strtok(NULL, "\n");
            
            if (axis && p_str && i_str && d_str && f_str) {
              float p = atof(p_str);
              float i = atof(i_str);
              float d = atof(d_str);
              float f = atof(f_str);
              
              if (strcmp(axis, "roll") == 0) {
                pid_roll.Kp = p; pid_roll.Ki = i; pid_roll.Kd = d; pid_roll.Kf = f;
              } else if (strcmp(axis, "pitch") == 0) {
                pid_pitch.Kp = p; pid_pitch.Ki = i; pid_pitch.Kd = d; pid_pitch.Kf = f;
              } else if (strcmp(axis, "yaw") == 0) {
                pid_yaw.Kp = p; pid_yaw.Ki = i; pid_yaw.Kd = d; pid_yaw.Kf = f;
              }
            }
          }
        }
        // Format M: M,loiter
        else if (esp_buffer[0] == 'M') {
          char mode_str[20];
          if (sscanf(esp_buffer, "M,%s", mode_str) == 1) {
            if (strcmp(mode_str, "stabilize") == 0)
              current_mode = MODE_STABILIZE;
            else if (strcmp(mode_str, "althold") == 0)
              current_mode = MODE_ALTHOLD;
            else if (strcmp(mode_str, "loiter") == 0)
              current_mode = MODE_LOITER;
            else if (strcmp(mode_str, "rtl") == 0)
              current_mode = MODE_RTL;
          }
        }
        // Format W: W,6.92710,79.86120
        else if (esp_buffer[0] == 'W') {
          char* token = strtok((char*)esp_buffer, ",");
          if (token != NULL && token[0] == 'W') {
            char* lat_str = strtok(NULL, ",");
            char* lon_str = strtok(NULL, "\n");
            if (lat_str && lon_str) {
              wp_lat = atof(lat_str);
              wp_lon = atof(lon_str);
              current_mode = MODE_AUTO; // Automatically switch to Waypoint navigation mode!
            }
          }
        }
        // Format A: A,1 (Arm) or A,0 (Disarm)
        else if (esp_buffer[0] == 'A') {
          int arm_val;
          if (sscanf(esp_buffer, "A,%d", &arm_val) == 1) {
            if (arm_val == 1) {
              // Force arm without checking throttle or previous state
              current_state = STATE_ARMED;
              setpoint_yaw = yaw_actual;
              home_lat = gps_lat;
              home_lon = gps_lon;
            } else if (arm_val == 0) {
              // Force disarm
              current_state = STATE_DISARMED;
              Reset_PID_Integrals(&pid_roll, &pid_pitch, &pid_yaw);
            }
          }
        }
        // Format B: B (Burn PIDs to Flash)
        else if (esp_buffer[0] == 'B') {
          Flash_Data fd;
          fd.r_p = pid_roll.Kp;
          fd.r_i = pid_roll.Ki;
          fd.r_d = pid_roll.Kd;
          fd.r_f = pid_roll.Kf;
          fd.p_p = pid_pitch.Kp;
          fd.p_i = pid_pitch.Ki;
          fd.p_d = pid_pitch.Kd;
          fd.p_f = pid_pitch.Kf;
          fd.y_p = pid_yaw.Kp;
          fd.y_i = pid_yaw.Ki;
          fd.y_d = pid_yaw.Kd;
          fd.y_f = pid_yaw.Kf;
          fd.mag_offset_x = mag_offset_x;
          fd.mag_offset_y = mag_offset_y;
          fd.mag_offset_z = mag_offset_z;
          Flash_Save(&fd);
        }
        // Format C: C,mag_x,mag_y,mag_z (Update Offsets)
        else if (esp_buffer[0] == 'C') {
          char* token = strtok((char*)esp_buffer, ",");
          if (token != NULL && token[0] == 'C') {
            char* ox_str = strtok(NULL, ",");
            char* oy_str = strtok(NULL, ",");
            char* oz_str = strtok(NULL, "\n");
            if (ox_str && oy_str && oz_str) {
              mag_offset_x = atof(ox_str);
              mag_offset_y = atof(oy_str);
              mag_offset_z = atof(oz_str);

            // Automatically burn to Flash memory too!
            Flash_Data fd;
            fd.r_p = pid_roll.Kp;
            fd.r_i = pid_roll.Ki;
            fd.r_d = pid_roll.Kd;
            fd.r_f = pid_roll.Kf;
            fd.p_p = pid_pitch.Kp;
            fd.p_i = pid_pitch.Ki;
            fd.p_d = pid_pitch.Kd;
            fd.p_f = pid_pitch.Kf;
            fd.y_p = pid_yaw.Kp;
            fd.y_i = pid_yaw.Ki;
            fd.y_d = pid_yaw.Kd;
            fd.y_f = pid_yaw.Kf;
            fd.mag_offset_x = mag_offset_x;
            fd.mag_offset_y = mag_offset_y;
            fd.mag_offset_z = mag_offset_z;
            Flash_Save(&fd);
            }
          }
        }
      }

      // --- SMART RADIO FAILSAFE ---
      if (current_time - last_radio_time > 500) {
        // No data received for 500ms! Connection lost.
        if (current_state == STATE_ARMED) {
          if (gps_fix > 0 && home_lat != 0.0f) {
            current_mode = MODE_RTL; // Fly home safely
          } else {
            current_mode = MODE_ALTHOLD;
            target_altitude -=
                0.5f * dt; // Soft landing: slowly descend at 0.5m/s
          }
        } else {
          current_state = STATE_FAILSAFE; // Disarmed, just stay failsafe
          base_throttle = 1000.0f;
        }
      } else if (current_state == STATE_FAILSAFE) {
        // Radio connection has recovered!
        current_state = STATE_DISARMED;
      }

      // --- STICK ARMING LOGIC (And Hardware Button Backup) ---
      static uint32_t stick_cmd_timer = 0;
      if (base_throttle <= 1120.0f) { // Adjusted for ESP32 min throttle of 1100
        // Stick Arm: Throttle Down + Yaw Full Right (> 400)
        if (nrf_data.yaw > 400 && current_state != STATE_ARMED) {
          if (stick_cmd_timer == 0)
            stick_cmd_timer = current_time;
          else if (current_time - stick_cmd_timer > 1000) { // Hold for 1 second
            current_state = STATE_ARMED;
            setpoint_yaw = yaw_actual;
            home_lat = gps_lat; // Lock home position
            home_lon = gps_lon;
            stick_cmd_timer = 0;
          }
        }
        // Stick Disarm: Throttle Down + Yaw Full Left (< -400)
        else if (nrf_data.yaw < -400 && current_state == STATE_ARMED) {
          if (stick_cmd_timer == 0)
            stick_cmd_timer = current_time;
          else if (current_time - stick_cmd_timer > 1000) { // Hold for 1 second
            current_state = STATE_DISARMED;
            Reset_PID_Integrals(&pid_roll, &pid_pitch, &pid_yaw);
            stick_cmd_timer = 0;
          }
        } else {
          stick_cmd_timer = 0;
        }
      } else {
        stick_cmd_timer = 0;
      }

      // Backup Hardware Button Arming (if joystick not available)
      if (HAL_GPIO_ReadPin(BTN_K1_GPIO_Port, BTN_K1_Pin) == GPIO_PIN_RESET) {
        if (current_time - last_button_press > 500) { // 500ms debounce
          last_button_press = current_time;

          if (current_state == STATE_DISARMED ||
              current_state == STATE_FAILSAFE) {
            if (base_throttle <= 1120.0f) {
              current_state = STATE_ARMED;
              setpoint_yaw = yaw_actual;
              home_lat = gps_lat;
              home_lon = gps_lon;
            }
          } else if (current_state == STATE_ARMED) {
            current_state = STATE_DISARMED;
            Reset_PID_Integrals(&pid_roll, &pid_pitch, &pid_yaw);
          }
        }
      }

      // --- CRASH DETECTION (KILLSWITCH) ---
      if (current_state == STATE_ARMED) {
        static uint32_t crash_timer = 0;

        // 1. 70-Degree Crash Detection Killswitch (Instant)
        if (fabsf(roll_actual) > 70.0f || fabsf(pitch_actual) > 70.0f) {
          current_state =
              STATE_DISARMED; // Drone flipped, kill motors immediately!
        }

        // 2. Stalled Motor / Tangled Propeller Protection
        // If throttle is very high but drone is extremely tilted (>45 deg) for
        // >2 seconds
        if (base_throttle > 1500.0f &&
            (fabsf(roll_actual) > 45.0f || fabsf(pitch_actual) > 45.0f)) {
          if (crash_timer == 0)
            crash_timer = current_time;
          else if (current_time - crash_timer > 2000) {
            current_state = STATE_DISARMED; // HARD KILL
            crash_timer = 0;
          }
        } else {
          crash_timer = 0;
        }

        // 4. Low Battery Failsafe (Bare-Metal)
        // If under 10.2V (3.4V per cell on 3S), trigger auto-land

        // If under 10.2V (3.4V per cell on 3S), trigger auto-land
        if (battery_voltage > 5.0f && battery_voltage < 10.2f &&
            current_state == STATE_ARMED) {
          current_state = STATE_FAILSAFE; // Force auto-land
          alt_hold_active = 1;
          target_altitude = alt_actual;
        }
      }

      // --- LED STATUS INDICATOR ---
      if (current_state == STATE_ARMED) {
        HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin,
                          GPIO_PIN_SET); // Solid ON when armed
      } else if (current_state == STATE_FAILSAFE) {
        // Fast blink for Failsafe / No Radio
        if ((current_time / 100) % 2 == 0) {
          HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
        } else {
          HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin,
                            GPIO_PIN_RESET);
        }
      } else {
        // Disarmed/Boot - LED OFF
        HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
      }

      // --- FLIGHT STATE MACHINE & PID MIXING ---
      if (current_state == STATE_ARMED) {
        if (base_throttle <= 1120.0f) { // Zero Throttle Check
          // ZERO THROTTLE MODE: Prevent PID windup and keep motors spinning at identical idle
          Reset_PID_Integrals(&pid_roll, &pid_pitch, &pid_yaw);
          Reset_PID_Integrals(&pid_alt, &pid_gps_pitch, &pid_gps_roll);
          motor1 = 1000; // User requested ZERO SPIN when armed at zero throttle
          motor2 = 1000;
          motor3 = 1000;
          motor4 = 1000;
        } else {
          // ACTIVE FLIGHT MODE

          // Calculate TPA factor (1.0 = full PID, < 1.0 = reduced PID)
          float tpa_factor = 1.0f;
          float tpa_breakpoint = 1500.0f; // Start reducing at 50% throttle
          float tpa_max_reduction = 0.3f; // At max throttle (2000), reduce P and D by 30%
          
          if (base_throttle > tpa_breakpoint) {
              tpa_factor = 1.0f - ((base_throttle - tpa_breakpoint) / (2000.0f - tpa_breakpoint)) * tpa_max_reduction;
              if (tpa_factor < 0.1f) tpa_factor = 0.1f; // Safety clamp
          }

          // 1. Altitude Hold Override
          float current_altitude = BMP280_GetAltitude();
          float throttle_output = base_throttle;
          if (alt_hold_active) {
            float alt_correction =
                PID_Compute(&pid_alt, target_altitude, current_altitude, dt, 1.0f);
            throttle_output =
                1500.0f + alt_correction; // Hover throttle is ~1500
          }

          // 2. GPS Navigation Override
          if (gps_hold_active && gps_fix > 0) {
            float dist =
                GPS_Distance(gps_lat, gps_lon, target_gps_lat, target_gps_lon);
            float bearing =
                GPS_Bearing(gps_lat, gps_lon, target_gps_lat, target_gps_lon);
            float heading = Get_Mag_Heading();

            // Convert bearing and heading to relative angle
            float relative_angle = bearing - heading;
            if (relative_angle > 180.0f)
              relative_angle -= 360.0f;
            else if (relative_angle < -180.0f)
              relative_angle += 360.0f;

            // Calculate required velocity vectors (X=Forward/Pitch,
            // Y=Right/Roll)
            float rad_angle = relative_angle * (M_PI / 180.0f);
            float target_vel_x =
                cosf(rad_angle) * dist; // Distance to move forward
            float target_vel_y =
                sinf(rad_angle) * dist; // Distance to move right

            // Cap maximum distance/velocity pull
            if (target_vel_x > 10.0f)
              target_vel_x = 10.0f;
            if (target_vel_x < -10.0f)
              target_vel_x = -10.0f;
            if (target_vel_y > 10.0f)
              target_vel_y = 10.0f;
            if (target_vel_y < -10.0f)
              target_vel_y = -10.0f;

            // Use GPS PID to generate pitch/roll angle overrides
            // Assuming current velocity is 0 for basic position hold logic
            // (Target vs 0)
            float gps_pitch_corr =
                PID_Compute(&pid_gps_pitch, target_vel_x, 0, dt, 1.0f);
            float gps_roll_corr =
                PID_Compute(&pid_gps_roll, target_vel_y, 0, dt, 1.0f);

            // Override the NRF setpoints (Note: +Pitch means tilt forward,
            // +Roll means tilt right)
            setpoint_pitch = gps_pitch_corr;
            setpoint_roll = gps_roll_corr;
          }

          // 3. Core Attitude PID Compute
          float pid_out_roll =
              PID_Compute(&pid_roll, setpoint_roll, roll_actual, dt, tpa_factor);
          float pid_out_pitch =
              PID_Compute(&pid_pitch, setpoint_pitch, pitch_actual, dt, tpa_factor);
          float pid_out_yaw =
              PID_Compute_Angular(&pid_yaw, setpoint_yaw, yaw_actual, dt, tpa_factor);

          // 4. Motor mixing (Quadcopter X Configuration)
          float idle_speed = 1150.0f;
          if (throttle_output < idle_speed) {
            throttle_output = idle_speed;
          }

          // Standard Betaflight Quad-X ("Props In") Motor Mixing
          float m1 = throttle_output - pid_out_roll + pid_out_pitch + pid_out_yaw; // Rear Right (CW)
          float m2 = throttle_output - pid_out_roll - pid_out_pitch - pid_out_yaw; // Front Right (CCW)
          float m3 = throttle_output + pid_out_roll + pid_out_pitch - pid_out_yaw; // Rear Left (CCW)
          float m4 = throttle_output + pid_out_roll - pid_out_pitch + pid_out_yaw; // Front Left (CW)

          // Limit PWM to prevent motors from stalling in mid-air
          // 1150us is the "Motor Idle Speed". It ensures motors never stop
          // spinning while the drone is actively flying, even if the PID loop
          // tries to lower them.

          if (m1 < idle_speed)
            m1 = idle_speed;
          if (m1 > 1940.0f)
            m1 = 1940.0f;
          if (m2 < idle_speed)
            m2 = idle_speed;
          if (m2 > 1940.0f)
            m2 = 1940.0f;
          if (m3 < idle_speed)
            m3 = idle_speed;
          if (m3 > 1940.0f)
            m3 = 1940.0f;
          if (m4 < idle_speed)
            m4 = idle_speed;
          if (m4 > 1940.0f)
            m4 = 1940.0f;

          motor1 = (uint16_t)m1;
          motor2 = (uint16_t)m2;
          motor3 = (uint16_t)m3;
          motor4 = (uint16_t)m4;
        }
      } else {
        // DISARMED or FAILSAFE Mode
        Reset_PID_Integrals(&pid_roll, &pid_pitch,
                            &pid_yaw); // Prevent mathematical buildup
        motor1 = 1000;
        motor2 = 1000;
        motor3 = 1000;
        motor4 = 1000;
      }

      // 5. Output to motors
      if (current_state == STATE_ARMED) {
        Set_Motor_PWM(motor1, motor2, motor3, motor4);
      } else {
        // DISARMED or FAILSAFE Mode
        Reset_PID_Integrals(&pid_roll, &pid_pitch,
                            &pid_yaw); // Prevent mathematical buildup
        motor1 = 1000; // Completely stop motors when disarmed
        motor2 = 1000;
        motor3 = 1000;
        motor4 = 1000;
        Set_Motor_PWM(motor1, motor2, motor3, motor4); // CRITICAL: Actually send the off signal!
      }

      // 6. Debug Printing via USB & ESP32 (Every 200ms / 50 loops)
      // Reduced to 5Hz to prevent the ESP32 WiFi buffer from filling up and causing dashboard latency
      static uint8_t print_counter = 0;
      print_counter++;
      if (print_counter >= 50) {
        print_counter = 0;
        float heading = Get_Mag_Heading();
        float alt = BMP280_GetAltitude();

        const char *s_r = (roll_actual < 0) ? "-" : "";
        const char *s_p = (pitch_actual < 0) ? "-" : "";
        const char *s_y = (yaw_actual < 0) ? "-" : "";
        const char *s_a = (alt < 0) ? "-" : "";
        const char *s_d = (heading < 0) ? "-" : "";
        const char *s_glat = (gps_lat < 0) ? "-" : "";
        const char *s_glon = (gps_lon < 0) ? "-" : "";

        // Added PID values, raw Mag data, and Battery Voltage to the outgoing
        // JSON!
        sprintf(
            uart_buf,
            "{\"v\":%d.%02d,\"r\":%s%d.%d,\"p\":%s%d.%d,\"y\":%s%d.%d,\"a\":%s%"
            "d.%d,\"d\":%s%d.%d,\"glat\":%s%d.%05d,\"glon\":%s%d.%05d,\"gf\":%"
            "d,\"t\":%d,\"mt\":%d,"
            "\"pid_r\":[%d.%02d,%d.%02d,%d.%02d,%d.%02d],\"pid_p\":[%d.%02d,%d.%02d,%d."
            "%02d,%d.%02d],\"pid_y\":[%d.%02d,%d.%02d,%d.%02d,%d.%02d],\"md\":%d,\"mx\":%d,"
            "\"my\":%d,\"mz\":%d,\"ry\":%d,\"rp\":%d,\"rr\":%d,\"arm\":%d}\n",
            (int)battery_voltage, (int)(battery_voltage * 100) % 100, s_r,
            (int)fabsf(roll_actual), (int)(fabsf(roll_actual) * 10) % 10, s_p,
            (int)fabsf(pitch_actual), (int)(fabsf(pitch_actual) * 10) % 10, s_y,
            (int)fabsf(yaw_actual), (int)(fabsf(yaw_actual) * 10) % 10, s_a,
            (int)fabsf(alt), (int)(fabsf(alt) * 10) % 10, s_d,
            (int)fabsf(heading), (int)(fabsf(heading) * 10) % 10, s_glat,
            (int)fabsf(gps_lat), (int)(fabsf(gps_lat) * 100000) % 100000,
            s_glon, (int)fabsf(gps_lon),
            (int)(fabsf(gps_lon) * 100000) % 100000, gps_fix,
            (int)base_throttle, mag_type, (int)pid_roll.Kp,
            (int)(pid_roll.Kp * 100) % 100, (int)pid_roll.Ki,
            (int)(pid_roll.Ki * 100) % 100, (int)pid_roll.Kd,
            (int)(pid_roll.Kd * 100) % 100, (int)pid_roll.Kf,
            (int)(pid_roll.Kf * 100) % 100, (int)pid_pitch.Kp,
            (int)(pid_pitch.Kp * 100) % 100, (int)pid_pitch.Ki,
            (int)(pid_pitch.Ki * 100) % 100, (int)pid_pitch.Kd,
            (int)(pid_pitch.Kd * 100) % 100, (int)pid_pitch.Kf,
            (int)(pid_pitch.Kf * 100) % 100, (int)pid_yaw.Kp,
            (int)(pid_yaw.Kp * 100) % 100, (int)pid_yaw.Ki,
            (int)(pid_yaw.Ki * 100) % 100, (int)pid_yaw.Kd,
            (int)(pid_yaw.Kd * 100) % 100, (int)pid_yaw.Kf,
            (int)(pid_yaw.Kf * 100) % 100, (int)current_mode, mag_x, mag_y,
            mag_z, nrf_data.yaw, nrf_data.pitch, nrf_data.roll,
            (current_state == STATE_ARMED ? 1 : 0));

        // Send to USB (Serial Monitor)
        CDC_Transmit_FS((uint8_t *)uart_buf, strlen(uart_buf));

        // Send to ESP32 Telemetry (UART2)
        HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, strlen(uart_buf), 50);
      }
    }
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
   */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00909BEB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern =
      SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern =
      SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern =
      SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern =
      SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 79;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3999; // Generates an interrupt every 4000us (250Hz)
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
}

static void BareMetal_ADC1_Init(void) {
  // 1. Enable Clocks
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN; // Enable GPIOC clock
  RCC->AHB1ENR |= RCC_AHB1ENR_ADC12EN; // Enable ADC12 clock

  // 2. Configure PC0 as Analog Mode
  GPIOC->MODER |= GPIO_MODER_MODE0; // 11: Analog mode for Pin 0

  // 3. Power up ADC voltage regulator
  ADC1->CR &= ~ADC_CR_DEEPPWD; // Exit deep power down
  ADC1->CR |= ADC_CR_ADVREGEN; // Enable ADC voltage regulator
  HAL_Delay(1);                // Wait for regulator to stabilize

  // 4. Calibrate ADC
  ADC1->CR &= ~ADC_CR_ADCALDIF; // Single-ended calibration
  ADC1->CR |= ADC_CR_ADCAL;
  while (ADC1->CR & ADC_CR_ADCAL)
    ; // Wait for calibration to finish

  // 5. Enable ADC
  ADC1->ISR |= ADC_ISR_ADRDY; // Clear ready flag
  ADC1->CR |= ADC_CR_ADEN;    // Enable ADC
  while (!(ADC1->ISR & ADC_ISR_ADRDY))
    ; // Wait for ready flag

  // 6. Configure Channel 10 (PC0)
  ADC1->PCSEL |= ADC_PCSEL_PCSEL_10; // Preselect channel 10

  // Set sampling time for channel 10 to 810.5 cycles (maximum)
  ADC1->SMPR2 |= (7U << ADC_SMPR2_SMP10_Pos);

  // Sequence 1: Channel 10
  ADC1->SQR1 &= ~ADC_SQR1_L;   // 1 conversion (L=0)
  ADC1->SQR1 &= ~ADC_SQR1_SQ1; // Clear SQ1 bits
  ADC1->SQR1 |= (10U << ADC_SQR1_SQ1_Pos);
}

static uint16_t BareMetal_ADC1_Read(void) {
  ADC1->CR |= ADC_CR_ADSTART; // Start conversion
  while (!(ADC1->ISR & ADC_ISR_EOC))
    ;                         // Wait for End Of Conversion
  uint16_t result = ADC1->DR; // Read Data
  ADC1->ISR |= ADC_ISR_EOC;   // Clear flag
  return result;
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  // SWAP PINS INTERNALLY TO BYPASS SOLDERED WIRING ISSUE
  // PA3 will now transmit (TX), and PA2 will receive (RX)
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
  huart2.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) !=
      HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BMP_CS_Pin | MPU_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BTN_K1_Pin and BTN_K2_Pin */
  GPIO_InitStruct.Pin = BTN_K1_Pin | BTN_K2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_STATUS_Pin NRF_CSN_Pin */
  GPIO_InitStruct.Pin = LED_STATUS_Pin | NRF_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF_CE_Pin */
  GPIO_InitStruct.Pin = NRF_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRF_CE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BMP_CS_Pin MPU_CS_Pin */
  GPIO_InitStruct.Pin = BMP_CS_Pin | MPU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA1, SYSCFG_SWITCH_PA1_CLOSE);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// --- IMPLEMENTATIONS ---

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void) {
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
   */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
