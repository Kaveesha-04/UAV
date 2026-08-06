#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// --- WIFI CONFIGURATION (Defaults) ---
String ssid = "";
String password = "";
String server_ip = "";
const uint16_t server_port = 5000;

Preferences preferences;
WebServer server(80);
WiFiClient client;

bool apMode = false;

// --- UART PINS (Connected to STM32 UART2) ---
// STM32 PA3 (TX) -> ESP32 D17 (RX)
// STM32 PA2 (RX) -> ESP32 D5 (TX)
#define RXD2 17
#define TXD2 5

String stm32_buffer = "";
String web_buffer = "";

void handleConfig() {
  // CORS Headers
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");

  if (server.method() == HTTP_OPTIONS) {
    server.send(204);
    return;
  }

  if (server.hasArg("ssid") && server.hasArg("server_ip")) {
    ssid = server.arg("ssid");
    password = server.arg("password");
    server_ip = server.arg("server_ip");

    preferences.begin("wifi_config", false);
    preferences.putString("ssid", ssid);
    preferences.putString("password", password);
    preferences.putString("server_ip", server_ip);
    preferences.end();

    server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Configuration saved. Rebooting...\"}");
    delay(500);
    ESP.restart();
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing parameters\"}");
  }
}

void setup() {
  Serial.begin(115200);  // USB Debugging
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); // STM32 Telemetry

  // Load configuration from flash
  preferences.begin("wifi_config", true);
  ssid = preferences.getString("ssid", "");
  password = preferences.getString("password", "");
  server_ip = preferences.getString("server_ip", "");
  preferences.end();

  Serial.println("Loaded config:");
  Serial.println("SSID: " + ssid);
  Serial.println("Server IP: " + server_ip);

  // Attempt to connect to WiFi
  if (ssid.length() > 0) {
    WiFi.begin(ssid.c_str(), password.c_str());
    Serial.print("Connecting to WiFi");
    
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 20) { // 10 seconds max
      delay(500);
      Serial.print(".");
      retries++;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.println("IP address: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nWiFi connection failed or no credentials. Starting AP Mode.");
    apMode = true;
    WiFi.mode(WIFI_AP);
    WiFi.softAP("Aeronexus-Setup");
    Serial.println("AP IP address: " + WiFi.softAPIP().toString());

    server.on("/config", HTTP_POST, handleConfig);
    server.on("/config", HTTP_OPTIONS, handleConfig); // Handle preflight
    server.begin();
    Serial.println("Web server started.");
  }
}

void loop() {
  if (apMode) {
    server.handleClient();
    return; // Don't do telemetry while in setup mode
  }

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
      WiFi.begin(ssid.c_str(), password.c_str());
    } else if (!client.connected() && server_ip.length() > 0) {
      client.connect(server_ip.c_str(), server_port);
    }
  }
}
