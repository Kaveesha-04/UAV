#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>
#include <nRF24L01.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// NRF24L01 Pins
#define CE_PIN 4
#define CSN_PIN 5

RF24 radio(CE_PIN, CSN_PIN);
const byte address[5] = {'0', '0', '0', '0',
                         '1'}; // Must perfectly match STM32 rx_addr

// I2C ADC (ADS1115)
Adafruit_ADS1115 ads;

// Data structure exactly matching the STM32 receiver
#pragma pack(push, 1)
struct Data_Package {
  uint16_t throttle; // 1000 to 2000
  int16_t yaw;       // -500 to 500
  int16_t pitch;     // -500 to 500
  int16_t roll;      // -500 to 500
};
#pragma pack(pop)

Data_Package data;

// ADC Calibration (Adjust these if your joystick doesn't reach full min/max)
// The ADS1115 with 1x Gain measures 0 to 4.096V. Usually a 3.3V joystick gives
// ~0 to 21000.
#define ADC_MIN 0
#define ADC_MAX 21500

// Center values for your specific joysticks (adjust these so the screen reads 0
// when released)
#define ADC_YAW_MID 13115 // Adjusted based on your 110 offset feedback
#define ADC_PITCH_MID 13115
#define ADC_ROLL_MID 13115

unsigned long last_oled_update = 0;
unsigned long last_tx_time = 0;

// Helper function to map joystick values so the exact middle is 0
int16_t map_joystick(int16_t val, int16_t min_val, int16_t mid_val,
                     int16_t max_val) {
  int16_t mapped_val = 0;
  if (val <= mid_val) {
    if (mid_val == min_val)
      return 0;
    mapped_val = (int16_t)((((float)(val - min_val) / (mid_val - min_val)) * 500.0f) -
                     500.0f);
  } else {
    if (max_val == mid_val)
      return 0;
    mapped_val = (int16_t)(((float)(val - mid_val) / (max_val - mid_val)) * 500.0f);
  }
  
  // Apply a deadband so it stays exactly at 0 when the joystick is released
  if (mapped_val > -25 && mapped_val < 25) {
    return 0;
  }
  
  return mapped_val;
}

void setup() {
  Serial.begin(115200);

  // Initialize I2C for ADC (SDA = 21, SCL = 22)
  Wire.begin(21, 22);

  // Initialize OLED display (I2C address 0x3C is standard for 0.96")
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 10);
    display.println("UAV Remote Init...");
    display.display();
  }

  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS1115.");
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("ERR: ADS1115 FAIL");
    display.display();
    while (1)
      ;
  }

  // Set ADC gain to 1x (Range +/- 4.096V)
  // This is perfect for 3.3V joystick signals
  ads.setGain(GAIN_ONE);

  // Speed up ADS1115 from default 128 SPS to 860 SPS
  // to eliminate the 32ms blocking read delay in the loop!
  ads.setDataRate(RATE_ADS1115_860SPS);

  // Initialize SPI for NRF24
  // SCK = 18, MISO = 19, MOSI = 23
  SPI.begin(18, 19, 23, CSN_PIN);

  if (!radio.begin()) {
    Serial.println("NRF24 Radio hardware is not responding!");
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("ERR: NRF24 FAIL");
    display.display();
    while (1)
      ;
  }

  radio.openWritingPipe(address);
  radio.setPALevel(
      RF24_PA_MAX); // Max power for best range (ensure good 3.3V supply!)
  radio.setDataRate(
      RF24_250KBPS); // 250kbps gives significantly better range than 2Mbps
  radio.setChannel(76); // Reverted to 76 (default) to match working config
  radio.setPayloadSize(
      sizeof(Data_Package)); // FORCE 8-byte payload to match STM32!
  radio.stopListening();     // Set as Transmitter

  Serial.println("ESP32 Remote Controller Initialized.");
}

void loop() {
  unsigned long current_time = millis();

  // Enforce a strict 50Hz update rate (20ms) for NRF transmission
  if (current_time - last_tx_time >= 20) {
    last_tx_time = current_time;

    // Read raw 16-bit values from the ADS1115 ADC
    int16_t raw_yaw = ads.readADC_SingleEnded(0);      // Vrx - A0
    int16_t raw_throttle = ads.readADC_SingleEnded(1); // Vry - A1
    int16_t raw_pitch = ads.readADC_SingleEnded(3);    // Vry - A3
    int16_t raw_roll = ads.readADC_SingleEnded(2);     // Vrx - A2  

    // Map the raw ADC values to the formats expected by the STM32 Firmware

    // Throttle: 1000 to 2000
    data.throttle = map(raw_throttle, ADC_MIN, ADC_MAX, 2000, 1000);
    data.throttle = constrain(data.throttle, 1000, 2000);

    // Yaw: Map to -500 to 500
    data.yaw = map_joystick(raw_yaw, ADC_MIN, ADC_YAW_MID, ADC_MAX);
    data.yaw = constrain(data.yaw, -500, 500);

    // Pitch: Map to -500 to 500
    data.pitch = map_joystick(raw_pitch, ADC_MIN, ADC_PITCH_MID, ADC_MAX);
    data.pitch = constrain(data.pitch, -500, 500);

    // Roll: Map to -500 to 500
    data.roll = - map_joystick(raw_roll, ADC_MIN, ADC_ROLL_MID, ADC_MAX);
    data.roll = constrain(data.roll, -500, 500);

    // Send the payload via NRF24
    bool success = radio.write(&data, sizeof(Data_Package));

    if (success) {
      Serial.printf("SENT OK | T: %d | Y: %d | P: %d | R: %d\n", data.throttle,
                    data.yaw, data.pitch, data.roll);
    } else {
      Serial.println("TX Failed - Check NRF24 connections on Drone");
    }

    // Update OLED Display only 4 times a second (every 250ms)
    // I2C displays take ~30ms to update, which would bottleneck our 50Hz radio
    // transmission!
    if (current_time - last_oled_update >= 250) {
      last_oled_update = current_time;

      display.clearDisplay();
      display.setCursor(0, 0);
      display.setTextSize(2);
      display.println("UAV REMOTE");

      display.setTextSize(1);
      display.setCursor(0, 20);
      display.printf("THR: %-4d", data.throttle);

      display.setCursor(0, 34);
      display.printf("YAW: %-4d  PIT: %-4d", data.yaw, data.pitch);

      display.setCursor(0, 46);
      display.printf("ROL: %-4d", data.roll);

      display.setCursor(0, 56);
      if (success) {
        display.println("NRF: TX OK");
      } else {
        display.println("NRF: TX FAIL");
      }
      display.display();
    }
  }
}
