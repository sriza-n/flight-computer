#include <SPI.h>
#include <RH_RF95.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>


// Server configuration
// #define SERVER_URL "http://192.168.1.12:5000/add_data"  // Remove this line
// Pin definitions for ESP32 and LoRa RA-02
#define LORA_CS 5   // NSS pin
#define LORA_RST 14 // RESET pin
#define LORA_INT 2  // DIO0 (Interrupt) pin

// SPI pins for ESP32 (default)
// SCK - 18
// MISO - 19
// MOSI - 23

// RF frequency - must match transmitter
#define RF95_FREQ 500

// WiFi configuration
#define AP_SSID "FlightComputer-Config"
#define AP_PASSWORD "12345678"
#define WIFI_TIMEOUT 10000  // 10 seconds

// Initialize LoRa instance
RH_RF95 rf95(LORA_CS, LORA_INT);

// WiFi components
WebServer server(80);
DNSServer dnsServer;
Preferences preferences;

void decodeReceivedData(uint8_t *buffer, uint8_t len);
void setupWiFi();
void startConfigPortal();
void startWebServer();
void handleRoot();
void handleConfigSubmit();
void handleNotFound();
void sendDataToServer(uint32_t sn, float timeValue, bool remotestate, bool nano1, bool nano2, bool nano3, bool nano4, bool valveState, float analog1, float analog2, float xPos, float yPos, float altitude, float eulerX, float eulerY, float eulerZ, float accelX, float accelY, float accelZ, float gpsLat, float gpsLng, float temperature, int rssi, float snr);

// Output pin definitions
#define NANO1_PIN 25
#define NANO2_PIN 26
#define NANO3_PIN 27
#define NANO4_PIN 32
#define REMOTE_PIN 33
#define VALVE_PIN 13

// store rssi
int rssi = 0;
float snr = 0;
int frequencyError = 0;
bool wifiConnected = false;

void setup()
{
  Serial.begin(115200);
  Serial.println("ESP32 LoRa Receiver with WiFi Config");

  // Initialize preferences
  preferences.begin("wifi", false);

  // Configure reset pin
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, HIGH);

  // Reset LoRa module
  digitalWrite(LORA_RST, LOW);
  delay(10);
  digitalWrite(LORA_RST, HIGH);
  delay(10);

  // Initialize LoRa
  if (!rf95.init())
  {
    Serial.println("LoRa initialization failed!");
    while (1)
      ;
  }
  Serial.println("LoRa initialized successfully");

  // Set frequency
  if (!rf95.setFrequency(RF95_FREQ))
  {
    Serial.println("Setting frequency failed!");
    while (1)
      ;
  }
  Serial.print("Frequency set to: ");
  Serial.println(RF95_FREQ);

  rf95.setSignalBandwidth(125000);
  rf95.setCodingRate4(5);
  rf95.setSpreadingFactor(9);
  rf95.spiWrite(RH_RF95_REG_0C_LNA, 0x23);
  rf95.setModeRx();

  // Configure output pins
  pinMode(NANO1_PIN, OUTPUT);
  pinMode(NANO2_PIN, OUTPUT);
  pinMode(NANO3_PIN, OUTPUT);
  pinMode(NANO4_PIN, OUTPUT);
  pinMode(REMOTE_PIN, OUTPUT);
  pinMode(VALVE_PIN, OUTPUT);

  // Initialize all outputs to LOW
  digitalWrite(NANO1_PIN, LOW);
  digitalWrite(NANO2_PIN, LOW);
  digitalWrite(NANO3_PIN, LOW);
  digitalWrite(NANO4_PIN, LOW);
  digitalWrite(REMOTE_PIN, LOW);
  digitalWrite(VALVE_PIN, LOW);

  // Setup WiFi
  setupWiFi();

  Serial.println("Receiver ready!");
}

void setupWiFi() {
  // Try to connect to saved WiFi credentials
  String ssid = preferences.getString("ssid", "");
  String password = preferences.getString("password", "");
  
  if (ssid.length() > 0) {
    Serial.println("Attempting to connect to saved WiFi...");
    WiFi.begin(ssid.c_str(), password.c_str());
    
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < WIFI_TIMEOUT) {
      delay(500);
      Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      wifiConnected = true;
      Serial.println();
      Serial.println("WiFi connected!");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      
      // Start web server for configuration updates
      startWebServer();
      return;
    }
  }
  
  // If connection failed or no credentials, start config portal
  Serial.println("Starting configuration portal...");
  startConfigPortal();
}

void startWebServer() {
  // Setup web server routes for station mode
  server.on("/", handleRoot);
  server.on("/config", HTTP_POST, handleConfigSubmit);
  server.onNotFound(handleNotFound);
  
  server.begin();
  Serial.println("Web server started on local IP");
  Serial.print("Configuration page available at: http://");
  Serial.println(WiFi.localIP());
}

void startConfigPortal() {
  // Start Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  
  Serial.print("Access Point started. SSID: ");
  Serial.println(AP_SSID);
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());
  
  // Start DNS server for captive portal
  dnsServer.start(53, "*", WiFi.softAPIP());
  
  // Setup web server routes
  server.on("/", handleRoot);
  server.on("/config", HTTP_POST, handleConfigSubmit);
  server.onNotFound(handleNotFound);
  
  server.begin();
  Serial.println("Web server started");
}

void handleRoot() {
  String currentSSID = preferences.getString("ssid", "");
  String currentServerURL = preferences.getString("serverurl", "192.168.1.12:5000");
  String connectionStatus = wifiConnected ? "Connected" : "Not Connected";
  String currentIP = wifiConnected ? WiFi.localIP().toString() : "N/A";
  
  String html = R"html(<!DOCTYPE html>
<html>
<head>
    <title>Flight Computer WiFi Config</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; margin: 40px; background: #f0f0f0; }
        .container { max-width: 400px; margin: 0 auto; background: white; padding: 30px; border-radius: 10px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }
        h1 { color: #333; text-align: center; margin-bottom: 30px; }
        input[type="text"], input[type="password"] { width: 100%; padding: 12px; margin: 8px 0; border: 1px solid #ddd; border-radius: 4px; box-sizing: border-box; }
        input[type="submit"] { width: 100%; background-color: #4CAF50; color: white; padding: 14px; margin: 8px 0; border: none; border-radius: 4px; cursor: pointer; font-size: 16px; }
        input[type="submit"]:hover { background-color: #45a049; }
        .info { background: #e7f3ff; padding: 15px; border-radius: 4px; margin-bottom: 20px; border-left: 4px solid #2196F3; }
        .status { background: #e8f5e8; padding: 15px; border-radius: 4px; margin-bottom: 20px; border-left: 4px solid #4CAF50; }
        .status.disconnected { background: #ffe8e8; border-left-color: #f44336; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🚀 Flight Computer</h1>
        <div class="status)html";
        
  if (!wifiConnected) {
    html += " disconnected";
  }
  
  html += R"html(">
            <strong>WiFi Status:</strong> )html";
  html += connectionStatus;
  html += R"html(<br>
            <strong>Current Network:</strong> )html";
  html += (currentSSID.length() > 0 ? currentSSID : "None");
  html += R"html(<br>
            <strong>IP Address:</strong> )html";
  html += currentIP;
  html += R"html(<br>
            <strong>Server URL:</strong> )html";
  html += currentServerURL;
  html += R"html(
        </div>
        <div class="info">
            <strong>Configuration</strong><br>
            )html";
  html += (wifiConnected ? "Update your WiFi credentials and server settings below." : "Please enter your WiFi credentials and server URL to configure the flight computer.");
  html += R"html(
        </div>
        <form action="/config" method="POST">
            <label for="ssid">WiFi Network Name (SSID):</label>
            <input type="text" id="ssid" name="ssid" required placeholder="Enter WiFi network name" value=")html";
  html += currentSSID;
  html += R"html(">
            
            <label for="password">WiFi Password:</label>
            <input type="password" id="password" name="password" required placeholder="Enter WiFi password">
            
            <label for="serverurl">Server URL (IP:Port):</label>
            <input type="text" id="serverurl" name="serverurl" required placeholder="e.g., 192.168.1.12:5000" value=")html";
  html += currentServerURL;
  html += R"html(">
            
            <input type="submit" value=")html";
  html += (wifiConnected ? "Update Configuration" : "Save & Connect");
  html += R"html(">
        </form>
    </div>
</body>
</html>)html";
  
  server.send(200, "text/html", html);
}

void handleConfigSubmit() {
  String ssid = server.arg("ssid");
  String password = server.arg("password");
  String serverurl = server.arg("serverurl");
  
  if (ssid.length() > 0 && serverurl.length() > 0) {
    // Save credentials and server URL
    preferences.putString("ssid", ssid);
    preferences.putString("password", password);
    preferences.putString("serverurl", serverurl);
    
    String html = R"html(
<!DOCTYPE html>
<html>
<head>
    <title>Flight Computer - Updating</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta http-equiv="refresh" content="10;url=/">
    <style>
        body { font-family: Arial, sans-serif; margin: 40px; background: #f0f0f0; text-align: center; }
        .container { max-width: 400px; margin: 0 auto; background: white; padding: 30px; border-radius: 10px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }
        .spinner { border: 4px solid #f3f3f3; border-top: 4px solid #3498db; border-radius: 50%; width: 40px; height: 40px; animation: spin 2s linear infinite; margin: 20px auto; }
        @keyframes spin { 0% { transform: rotate(0deg); } 100% { transform: rotate(360deg); } }
    </style>
</head>
<body>
    <div class="container">
        <h1>🚀 Flight Computer</h1>
        <div class="spinner"></div>
        <p>Updating configuration...</p>
        <p><strong>Network:</strong> )html" + ssid + R"html(</p>
        <p><strong>Server:</strong> )html" + serverurl + R"html(</p>
        <p>Device will restart in a few seconds.</p>
    </div>
</body>
</html>
)html";
    
    server.send(200, "text/html", html);
    
    Serial.println("Configuration saved. Restarting...");
    delay(2000);
    ESP.restart();
  } else {
    server.send(400, "text/plain", "Invalid SSID or Server URL");
  }
}

void handleNotFound() {
  // Redirect all unknown requests to root (captive portal behavior)
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void loop()
{
  // Handle DNS and web server
  if (!wifiConnected) {
    dnsServer.processNextRequest();
  }
  server.handleClient();

  // Continue with LoRa functionality
  if (rf95.available())
  {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len))
    {
      rssi = rf95.lastRssi();
      snr = rf95.lastSNR();
      frequencyError = rf95.frequencyError();
      decodeReceivedData(buf, len);
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
}

void decodeReceivedData(uint8_t *buffer, uint8_t len)
{
  if (len < 66)
  {
    Serial.println("Received incomplete data packet");
    Serial.print("Got only ");
    Serial.print(len);
    Serial.println(" bytes");
    return;
  }

  int index = 0;

  // Extract record serial number
  uint32_t sn;
  memcpy(&sn, &buffer[index], 4);
  index += 4;

  // Extract time as float (MM.SS format)
  float timeValue;
  memcpy(&timeValue, &buffer[index], 4);
  index += 4;

  // Convert to readable time string
  int minutes = (int)timeValue;
  int seconds = (int)((timeValue - minutes) * 100 + 0.5);
  String timeStr = String(minutes) + ":" + (seconds < 10 ? "0" : "") + String(seconds);

  // Extract nano values
  uint8_t nanoFlags = buffer[index++];
  bool nano1 = nanoFlags & 0x01;
  bool nano2 = nanoFlags & 0x02;
  bool nano3 = nanoFlags & 0x04;
  bool nano4 = nanoFlags & 0x08;
  bool remotestate = nanoFlags & 0x10;

  // Extract valve state
  bool valveState = buffer[index++];

  // Extract analog values
  float analog1, analog2;
  memcpy(&analog1, &buffer[index], 4);
  index += 4;
  memcpy(&analog2, &buffer[index], 4);
  index += 4;

  // Extract position values
  float xPos, yPos;
  memcpy(&xPos, &buffer[index], 4);
  index += 4;
  memcpy(&yPos, &buffer[index], 4);
  index += 4;

  // Extract altitude
  float altitude;
  memcpy(&altitude, &buffer[index], 4);
  index += 4;

  // Extract euler angles
  float eulerX, eulerY, eulerZ;
  memcpy(&eulerX, &buffer[index], 4);
  index += 4;
  memcpy(&eulerY, &buffer[index], 4);
  index += 4;
  memcpy(&eulerZ, &buffer[index], 4);
  index += 4;

  // Extract linear acceleration
  float accelX, accelY, accelZ;
  memcpy(&accelX, &buffer[index], 4);
  index += 4;
  memcpy(&accelY, &buffer[index], 4);
  index += 4;
  memcpy(&accelZ, &buffer[index], 4);
  index += 4;

  // Extract GPS coordinates
  float gpsLat, gpsLng;
  memcpy(&gpsLat, &buffer[index], 4);
  index += 4;
  memcpy(&gpsLng, &buffer[index], 4);
  index += 4;

  // Extract temperature
  float temperature;
  memcpy(&temperature, &buffer[index], 4);
  index += 4;

  // Set output pins according to received flag values
  digitalWrite(NANO1_PIN, nano1 ? HIGH : LOW);
  digitalWrite(NANO2_PIN, nano2 ? HIGH : LOW);
  digitalWrite(NANO3_PIN, nano3 ? HIGH : LOW);
  digitalWrite(NANO4_PIN, nano4 ? HIGH : LOW);
  digitalWrite(REMOTE_PIN, remotestate ? HIGH : LOW);
  digitalWrite(VALVE_PIN, valveState ? HIGH : LOW);

  // Print decoded data in CSV format
  Serial.print(sn);
  Serial.print(",");
  Serial.print(timeStr);
  Serial.print(",");
  Serial.print(remotestate ? "1" : "0");
  Serial.print(",");
  Serial.print(nano1 ? "1" : "0");
  Serial.print(",");
  Serial.print(nano2 ? "1" : "0");
  Serial.print(",");
  Serial.print(nano3 ? "1" : "0");
  Serial.print(",");
  Serial.print(nano4 ? "1" : "0");
  Serial.print(",");
  Serial.print(valveState ? "1" : "0");
  Serial.print(",");
  Serial.print(analog1, 2);
  Serial.print(",");
  Serial.print(analog2, 2);
  Serial.print(",");
  Serial.print(xPos, 2);
  Serial.print(",");
  Serial.print(yPos, 2);
  Serial.print(",");
  Serial.print(altitude, 2);
  Serial.print(",");
  Serial.print(eulerX, 4);
  Serial.print(",");
  Serial.print(eulerY, 4);
  Serial.print(",");
  Serial.print(eulerZ, 4);
  Serial.print(",");
  Serial.print(accelX, 4);
  Serial.print(",");
  Serial.print(accelY, 4);
  Serial.print(",");
  Serial.print(accelZ, 4);
  Serial.print(",");
  Serial.print(gpsLat, 6);
  Serial.print(",");
  Serial.print(gpsLng, 6);
  Serial.print(",");
  Serial.print(temperature, 2);
  Serial.print(",");
  Serial.print(rssi);
  Serial.print(",");
  Serial.println(snr);
  // Serial.print("RSSI: ");
  // Serial.print(rssi);
  // Serial.print(", SNR: ");
  // Serial.print(snr);
  // Serial.print(", FreqErr: ");
  // Serial.println(frequencyError);
    if (wifiConnected) {
    sendDataToServer(sn, timeValue, remotestate,nano1, nano2, nano3, nano4, valveState, analog1, analog2, xPos, yPos, altitude, eulerX, eulerY, eulerZ, accelX, accelY, accelZ, gpsLat, gpsLng, temperature, rssi, snr);
  }
}


void sendDataToServer(uint32_t sn, float timeValue, bool remotestate, bool nano1, bool nano2, bool nano3, bool nano4, bool valveState, float analog1, float analog2, float xPos, float yPos, float altitude, float eulerX, float eulerY, float eulerZ, float accelX, float accelY, float accelZ, float gpsLat, float gpsLng, float temperature, int rssi, float snr) {
  String serverURL = preferences.getString("serverurl", "192.168.1.12:5000");
  String fullURL = "http://" + serverURL + "/add_data";
  
  HTTPClient http;
  http.begin(fullURL);
  http.addHeader("Content-Type", "application/json");

  // Create JSON object
  JsonDocument doc;
  doc.shrinkToFit();
  
  // Convert time to string format for teensytime
  int minutes = (int)timeValue;
  int seconds = (int)((timeValue - minutes) * 100 + 0.5);
  String timeStr = String(minutes * 100 + seconds); // Convert MM.SS to MMSS format
  
  doc["teensytime"] = String(timeStr);
  doc["record_sn"] = String(sn);
  doc["voltage"] = analog1;
  doc["current"] = analog2;
  doc["teensytemp"] = temperature;
  doc["remote_st"] = remotestate ? 1 : 0;
  doc["valve_1"] = nano1 ? 1 : 0;
  doc["valve_2"] = nano2 ? 1 : 0;
  doc["activ_st"] = nano3 ? 1 : 0;
  doc["igni_st"] = nano4 ? 1 : 0;
  doc["para_st"] = valveState ? 1 : 0;
  doc["x_pos"] = xPos;
  doc["y_pos"] = yPos;
  doc["alt"] = altitude;
  doc["eu_x"] = eulerX;
  doc["eu_y"] = eulerY;
  doc["eu_z"] = eulerZ;
  doc["acc_x"] = accelX;
  doc["acc_y"] = accelY;
  doc["acc_z"] = accelZ;
  doc["lat"] = gpsLat;
  doc["lon"] = gpsLng;
  doc["rssi"] = rssi;
  doc["snr"] = snr;

  String jsonString;
  serializeJson(doc, jsonString);

  // Serial.print("Sending data to server: ");
  // Serial.println(jsonString);

  int httpResponseCode = http.POST(jsonString);

  // if (httpResponseCode > 0) {
  //   String response = http.getString();
  //   Serial.print("HTTP Response code: ");
  //   Serial.println(httpResponseCode);
  //   Serial.print("Response: ");
  //   Serial.println(response);
  // } else {
  //   Serial.print("Error on sending POST: ");
  //   Serial.println(httpResponseCode);
  // }

  http.end();
}