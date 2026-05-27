#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// NRF24L01 Pins
#define CE_PIN 4
#define CSN_PIN 5

RF24 radio(CE_PIN, CSN_PIN);
const byte address[5] = {'0', '0', '0', '0', '1'}; // Must perfectly match STM32 rx_addr

// I2C ADC (ADS1115)
Adafruit_ADS1115 ads; 

// Data structure exactly matching the STM32 receiver
#pragma pack(push, 1)
struct Data_Package {
  uint16_t throttle; // 1000 to 2000
  int16_t yaw;       // 0 to 1023 (Mapped by STM32 to -500 to 500)
  int16_t pitch;     // 0 to 1023 (Mapped by STM32 to -500 to 500)
  int16_t roll;      // 0 to 1023 (Mapped by STM32 to -500 to 500)
};
#pragma pack(pop)

Data_Package data;

// ADC Calibration (Adjust these if your joystick doesn't reach full min/max)
// The ADS1115 with 1x Gain measures 0 to 4.096V. Usually a 3.3V joystick gives ~0 to 21000.
#define ADC_MIN 0
#define ADC_MAX 21500 

void setup() {
  Serial.begin(115200);

  // Initialize I2C for ADC (SDA = 21, SCL = 22)
  Wire.begin(21, 22);

  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS1115.");
    while (1);
  }
  
  // Set ADC gain to 1x (Range +/- 4.096V)
  // This is perfect for 3.3V joystick signals
  ads.setGain(GAIN_ONE);

  // Initialize SPI for NRF24
  // SCK = 18, MISO = 19, MOSI = 23
  SPI.begin(18, 19, 23, CSN_PIN);
  
  if (!radio.begin()) {
    Serial.println("NRF24 Radio hardware is not responding!");
    while (1);
  }

  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW); // Changed to LOW to prevent voltage dropouts!
  radio.setDataRate(RF24_2MBPS); // Reverted to 2Mbps to match working config
  radio.setChannel(76);          // Reverted to 76 (default) to match working config
  radio.setPayloadSize(sizeof(Data_Package)); // FORCE 8-byte payload to match STM32!
  radio.stopListening(); // Set as Transmitter

  Serial.println("ESP32 Remote Controller Initialized.");
}

void loop() {
  // Read raw 16-bit values from the ADS1115 ADC
  int16_t raw_yaw = ads.readADC_SingleEnded(0);      // Vrx - A0
  int16_t raw_throttle = ads.readADC_SingleEnded(1); // Vry - A1
  int16_t raw_pitch = ads.readADC_SingleEnded(2);    // Vry - A2
  int16_t raw_roll = ads.readADC_SingleEnded(3);     // Vrx - A3

  // Map the raw ADC values to the formats expected by the STM32 Firmware
  
  // Throttle: 1000 to 2000
  data.throttle = map(raw_throttle, ADC_MIN, ADC_MAX, 1000, 2000);
  data.throttle = constrain(data.throttle, 1000, 2000);

  // Yaw: 0 to 1023
  data.yaw = map(raw_yaw, ADC_MIN, ADC_MAX, 0, 1023);
  data.yaw = constrain(data.yaw, 0, 1023);

  // Pitch: 0 to 1023
  data.pitch = map(raw_pitch, ADC_MIN, ADC_MAX, 0, 1023);
  data.pitch = constrain(data.pitch, 0, 1023);

  // Roll: 0 to 1023
  data.roll = map(raw_roll, ADC_MIN, ADC_MAX, 0, 1023);
  data.roll = constrain(data.roll, 0, 1023);

  // Send the payload via NRF24
  bool success = radio.write(&data, sizeof(Data_Package));

  if (success) {
    Serial.printf("SENT OK | T: %d | Y: %d | P: %d | R: %d\n", 
                  data.throttle, data.yaw, data.pitch, data.roll);
  } else {
    Serial.println("TX Failed - Check NRF24 connections on Drone");
  }

  // 50Hz update rate (20ms delay) to match drone's receiving speed
  delay(20); 
}
