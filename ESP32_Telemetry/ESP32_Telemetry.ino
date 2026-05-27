#include <WiFi.h>

const char* ssid = "SLT-Fiber-2.4G";
const char* password = "Nimsara1";

const char* server_ip = "172.20.10.8"; // PC'S LOCAL IP ADDRESS (Updated automatically)
const uint16_t server_port = 5000;

WiFiClient client;

// Pins are physically soldered: STM32 PA2 -> ESP32 D17, STM32 PA3 -> ESP32 D18
// Since the STM32 software already swapped them, we must NOT swap them here!
// D17 must be TX (to match STM32 PA2 RX), and D18 must be RX (to match STM32 PA3 TX).
#define RXD2 18
#define TXD2 17

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
}

String stm32_buffer = "";
String web_buffer = "";

void loop() {
  // Always read from STM32 to prevent UART buffer overflow and allow USB debugging
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      if (stm32_buffer.length() > 0) {
        Serial.println("STM32: " + stm32_buffer); // ALWAYS PRINT TO USB FOR DEBUGGING
        if (client.connected()) {
          client.println(stm32_buffer);
        }
        stm32_buffer = "";
      }
    } else if (c != '\r') {
      if (stm32_buffer.length() < 512) {
        stm32_buffer += c;
      } else {
        stm32_buffer = ""; // Prevent Out-of-Memory crash if STM32 drops newline
      }
    }
  }

  // Read incoming commands from Node.js (Web Dashboard) and forward to STM32
  while (client.available()) {
    char c = client.read();
    if (c == '\n') {
      if (web_buffer.length() > 0) {
        Serial.println("From Web: " + web_buffer); // Print to USB for debugging
        Serial2.println(web_buffer); // Send to STM32 over UART2
        web_buffer = "";
      }
    } else if (c != '\r') {
      if (web_buffer.length() < 512) {
        web_buffer += c;
      } else {
        web_buffer = ""; // Prevent Out-of-Memory crash
      }
    }
  }

  // Manage TCP and WiFi Connection
  static unsigned long last_connect_attempt = 0;
  if (millis() - last_connect_attempt > 2000) {
    last_connect_attempt = millis();
    
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi Disconnected. Reconnecting...");
      WiFi.disconnect();
      WiFi.begin(ssid, password);
    } else if (!client.connected()) {
      Serial.println("Connecting to TCP Server (" + String(server_ip) + ")...");
      if (client.connect(server_ip, server_port)) {
        Serial.println("Connected to Node.js Backend!");
      }
    }
  }
}
