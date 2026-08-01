#include <WiFi.h>

// --- WIFI CONFIGURATION ---
const char* ssid = "YOUR_WIFI_SSID";           // Replace with your WiFi network name
const char* password = "YOUR_WIFI_PASSWORD";   // Replace with your WiFi password

// --- DASHBOARD SERVER ---
const char* server_ip = "YOUR_PC_LOCAL_IP";    // IP Address of the PC running the Dashboard (e.g. "192.168.1.100")
const uint16_t server_port = 5000;

WiFiClient client;

// --- UART PINS (Connected to STM32 UART2) ---
// STM32 PA3 (TX) -> ESP32 D17 (RX)
// STM32 PA2 (RX) -> ESP32 D5 (TX)
#define RXD2 17
#define TXD2 5

String stm32_buffer = "";
String web_buffer = "";

void setup() {
  Serial.begin(115200);  // USB Debugging
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); // STM32 Telemetry

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
}

void loop() {
  // 1. READ FROM STM32 -> SEND TO DASHBOARD
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      if (stm32_buffer.length() > 0) {
        Serial.println("STM32: " + stm32_buffer); // Print to USB for debug
        if (client.connected()) {
          client.println(stm32_buffer);           // Send JSON to Dashboard
        }
        stm32_buffer = "";
      }
    } else if (c != '\r') {
      if (stm32_buffer.length() < 512) stm32_buffer += c;
      else stm32_buffer = ""; // Prevent buffer overflow
    }
  }

  // 2. READ FROM DASHBOARD -> SEND TO STM32
  while (client.available()) {
    char c = client.read();
    if (c == '\n') {
      if (web_buffer.length() > 0) {
        Serial2.println(web_buffer); // Send command to STM32
        web_buffer = "";
      }
    } else if (c != '\r') {
      if (web_buffer.length() < 512) web_buffer += c;
      else web_buffer = ""; 
    }
  }

  // 3. AUTO-RECONNECT LOGIC
  static unsigned long last_connect_attempt = 0;
  if (millis() - last_connect_attempt > 2000) {
    last_connect_attempt = millis();
    
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.disconnect();
      WiFi.begin(ssid, password);
    } else if (!client.connected()) {
      client.connect(server_ip, server_port);
    }
  }
}
