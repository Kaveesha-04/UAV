#include <WiFi.h>

const char* ssid = "SLT-Fiber-2.4G";
const char* password = "Nimsara1";

const char* server_ip = "172.20.10.8"; // PC'S LOCAL IP ADDRESS (Updated automatically)
const uint16_t server_port = 5000;

WiFiClient client;

// Pins are physically soldered: STM32 PA2 -> ESP32 D17, STM32 PA3 -> ESP32 D18
// We have swapped the pins in software on BOTH sides!
#define RXD2 17
#define TXD2 18

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

void loop() {
  // Always read from STM32 to prevent UART buffer overflow and allow USB debugging
  if (Serial2.available()) {
    String data = Serial2.readStringUntil('\n');
    if (data.length() > 0) {
      Serial.println("STM32: " + data); // ALWAYS PRINT TO USB FOR DEBUGGING
      
      // If connected to Node.js, send it over Wi-Fi
      if (client.connected()) {
        client.println(data);
      }
    }
  }

  // Read incoming commands from Node.js (Web Dashboard) and forward to STM32
  if (client.available()) {
    String incoming = client.readStringUntil('\n');
    if (incoming.length() > 0) {
      Serial.println("From Web: " + incoming); // Print to USB for debugging
      Serial2.println(incoming); // Send to STM32 over UART2
    }
  }

  // Manage TCP Connection (non-blocking style check)
  static unsigned long last_connect_attempt = 0;
  if (!client.connected() && millis() - last_connect_attempt > 2000) {
    last_connect_attempt = millis();
    Serial.println("Connecting to TCP Server (" + String(server_ip) + ")...");
    if (client.connect(server_ip, server_port)) {
      Serial.println("Connected to Node.js Backend!");
    }
  }
}
