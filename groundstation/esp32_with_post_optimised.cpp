#include <SPI.h>
#include <RH_RF95.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// Pin definitions for ESP32 and LoRa
#define LORA_CS 5   // NSS pin
#define LORA_RST 14 // RESET pin
#define LORA_INT 2  // DIO0 (interrupt) pin

// RF frequency setting
#define RF95_FREQ 500

// WiFi configuration
#define AP_SSID "GroundStation📥-Config"
#define AP_PASSWORD "logger1234"
#define WIFI_TIMEOUT 10000 // 10 seconds
#define DNS_PORT 53

// Output pin definitions
#define NANO1_PIN 25
#define NANO2_PIN 26
#define NANO3_PIN 27
#define NANO4_PIN 32
#define REMOTE_PIN 33
#define VALVE_PIN 13

// Data packet expected size
#define MIN_PACKET_SIZE 66

// Initialize components
RH_RF95 rf95(LORA_CS, LORA_INT);
WebServer server(80);
DNSServer dnsServer;
Preferences preferences;

// Signal quality metrics
int rssi = 0;
float snr = 0;
bool wifiConnected = false;

// Add these at the top with other variables
unsigned long lastServerAttempt = 0;
const unsigned long SERVER_RETRY_INTERVAL = 3000; // 3 seconds between server retries
DynamicJsonDocument lastReceivedData(512);        // Use DynamicJsonDocument with capacity parameter


// Function declarations
void setupLoRa();
void setupOutputPins();
void setupWiFi();
void startConfigPortal();
void startWebServer();
void handleRoot();
void handleConfigSubmit();
void handleNotFound();
void decodeReceivedData(uint8_t *buffer, uint8_t len);
void sendDataToServer(const DynamicJsonDocument &doc);
String formatTime(float timeValue);
void printTelemetryData(uint32_t sn, String timeStr, bool remotestate, bool nano1, bool nano2,
                        bool nano3, bool nano4, bool valveState, float analog1, float analog2,
                        float xPos, float yPos, float altitude, float eulerX, float eulerY,
                        float eulerZ, float accelX, float accelY, float accelZ, float gpsLat,
                        float gpsLng, float temperature, int rssi, float snr);

void setup()
{
  Serial.begin(115200);
  Serial.println("ESP32 LoRa Receiver Starting");

  preferences.begin("wifi", false);
  setupLoRa();
  setupOutputPins();
  setupWiFi();

  Serial.println("Receiver ready!");
}

void setupLoRa()
{
  // Configure reset pin
  pinMode(LORA_RST, OUTPUT);

  // Reset LoRa module
  digitalWrite(LORA_RST, LOW);
  delay(10);
  digitalWrite(LORA_RST, HIGH);
  delay(10);

  if (!rf95.init())
  {
    Serial.println("LoRa initialization failed!");
    while (1)
    {
      delay(100);
    }
  }

  if (!rf95.setFrequency(RF95_FREQ))
  {
    Serial.println("Setting frequency failed!");
    while (1)
    {
      delay(100);
    }
  }

  Serial.println("LoRa initialized at " + String(RF95_FREQ) + " MHz");

  // Configure LoRa parameters
  rf95.setSignalBandwidth(125000);
  rf95.setCodingRate4(5);
  rf95.setSpreadingFactor(9);
  rf95.spiWrite(RH_RF95_REG_0C_LNA, 0x23);
  rf95.setModeRx();
}

void setupOutputPins()
{
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
}

void setupWiFi()
{
  // Try to connect to saved WiFi credentials
  String ssid = preferences.getString("ssid", "");
  String password = preferences.getString("password", "");

  if (ssid.length() > 0)
  {
    Serial.println("Connecting to saved WiFi: " + ssid);
    WiFi.begin(ssid.c_str(), password.c_str());

    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < WIFI_TIMEOUT)
    {
      delay(500);
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED)
    {
      wifiConnected = true;
      Serial.println("\nWiFi connected at " + WiFi.localIP().toString());
      startWebServer();
      return;
    }
    Serial.println("\nFailed to connect to saved WiFi");
  }

  // Start config portal if connection failed or no credentials
  startConfigPortal();
}

void startWebServer()
{
  server.on("/", handleRoot);
  server.on("/config", HTTP_POST, handleConfigSubmit);
  server.onNotFound(handleNotFound);
  server.begin();
}

void startConfigPortal()
{
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  Serial.println("Access Point started: " + String(AP_SSID));
  Serial.println("IP: " + WiFi.softAPIP().toString());

  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  startWebServer();
}

void handleRoot()
{
  String currentSSID = preferences.getString("ssid", "");
  String currentServerURL = preferences.getString("serverurl", "192.168.1.12:5000");
  String connectionStatus = wifiConnected ? "Connected" : "Not Connected";
  String currentIP = wifiConnected ? WiFi.localIP().toString() : "N/A";

  String html = "<!DOCTYPE html>"
                "<html>"
                "<head>"
                "<title>Flight Computer WiFi Config</title>"
                "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
                "<style>"
                "body{font-family:Arial,sans-serif;margin:40px;background:#f0f0f0;}"
                ".container{max-width:400px;margin:0 auto;background:white;padding:30px;border-radius:10px;box-shadow:0 4px 6px rgba(0,0,0,0.1);}"
                "h1{color:#333;text-align:center;margin-bottom:30px;}"
                "input[type=\"text\"],input[type=\"password\"]{width:100%;padding:12px;margin:8px 0;border:1px solid #ddd;border-radius:4px;box-sizing:border-box;}"
                "input[type=\"submit\"]{width:100%;background-color:#4CAF50;color:white;padding:14px;margin:8px 0;border:none;border-radius:4px;cursor:pointer;font-size:16px;}"
                "input[type=\"submit\"]:hover{background-color:#45a049;}"
                ".info{background:#e7f3ff;padding:15px;border-radius:4px;margin-bottom:20px;border-left:4px solid #2196F3;}"
                ".status{background:#e8f5e8;padding:15px;border-radius:4px;margin-bottom:20px;border-left:4px solid #4CAF50;}"
                ".status.disconnected{background:#ffe8e8;border-left-color:#f44336;}"
                "</style>"
                "</head>"
                "<body>"
                "<div class=\"container\">"
                "<h1>🚀 Flight Computer</h1>"
                "<div class=\"status" +
                String(!wifiConnected ? " disconnected" : "") + "\">"
                                                                "<strong>WiFi Status:</strong> " +
                connectionStatus + "<br>"
                                   "<strong>Current Network:</strong> " +
                (currentSSID.length() > 0 ? currentSSID : "None") + "<br>"
                                                                    "<strong>IP Address:</strong> " +
                currentIP + "<br>"
                            "<strong>Server URL:</strong> " +
                currentServerURL +
                "</div>"
                "<div class=\"info\">"
                "<strong>Configuration</strong><br>" +
                String(wifiConnected ? "Update your WiFi credentials and server settings below." : "Please enter your WiFi credentials and server URL to configure the flight computer.") +
                "</div>"
                "<form action=\"/config\" method=\"POST\">"
                "<label for=\"ssid\">WiFi Network Name (SSID):</label>"
                "<input type=\"text\" id=\"ssid\" name=\"ssid\" required placeholder=\"Enter WiFi network name\" value=\"" +
                currentSSID + "\">"
                              "<label for=\"password\">WiFi Password:</label>"
                              "<input type=\"password\" id=\"password\" name=\"password\" required placeholder=\"Enter WiFi password\">"
                              "<label for=\"serverurl\">Server URL (IP:Port):</label>"
                              "<input type=\"text\" id=\"serverurl\" name=\"serverurl\" required placeholder=\"e.g., 192.168.1.12:5000\" value=\"" +
                currentServerURL + "\">"
                                   "<input type=\"submit\" value=\"" +
                String(wifiConnected ? "Update Configuration" : "Save & Connect") + "\">"
                                                                                    "</form>"
                                                                                    "</div>"
                                                                                    "</body>"
                                                                                    "</html>";

  server.send(200, "text/html", html);
}

void handleConfigSubmit()
{
  String ssid = server.arg("ssid");
  String password = server.arg("password");
  String serverurl = server.arg("serverurl");

  if (ssid.length() > 0 && serverurl.length() > 0)
  {
    // Save credentials and server URL
    preferences.putString("ssid", ssid);
    preferences.putString("password", password);
    preferences.putString("serverurl", serverurl);

    String html = "<!DOCTYPE html>"
                  "<html>"
                  "<head>"
                  "<title>Flight Computer - Updating</title>"
                  "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
                  "<meta http-equiv=\"refresh\" content=\"5;url=/\">"
                  "<style>"
                  "body{font-family:Arial,sans-serif;margin:40px;background:#f0f0f0;text-align:center;}"
                  ".container{max-width:400px;margin:0 auto;background:white;padding:30px;border-radius:10px;box-shadow:0 4px 6px rgba(0,0,0,0.1);}"
                  ".spinner{border:4px solid #f3f3f3;border-top:4px solid #3498db;border-radius:50%;width:40px;height:40px;animation:spin 2s linear infinite;margin:20px auto;}"
                  "@keyframes spin{0%{transform:rotate(0deg);}100%{transform:rotate(360deg);}}"
                  "</style>"
                  "</head>"
                  "<body>"
                  "<div class=\"container\">"
                  "<h1>🚀 Flight Computer</h1>"
                  "<div class=\"spinner\"></div>"
                  "<p>Updating configuration...</p>"
                  "<p><strong>Network:</strong> " +
                  ssid + "</p>"
                         "<p><strong>Server:</strong> " +
                  serverurl + "</p>"
                              "<p>Device will restart in a few seconds.</p>"
                              "</div>"
                              "</body>"
                              "</html>";

    server.send(200, "text/html", html);
    Serial.println("Configuration saved, restarting...");
    delay(1000);
    ESP.restart();
  }
  else
  {
    server.send(400, "text/plain", "Invalid SSID or Server URL");
  }
}

void handleNotFound()
{
  // Redirect all unknown requests to root (captive portal behavior)
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void loop()
{
  // Handle DNS and web server
  if (!wifiConnected)
  {
    dnsServer.processNextRequest();
  }
  server.handleClient();

  // Check for LoRa messages
  if (rf95.available())
  {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len))
    {
      rssi = rf95.lastRssi();
      snr = rf95.lastSNR();
      decodeReceivedData(buf, len);
      if (wifiConnected)
      {
        sendDataToServer(lastReceivedData);
      }
    }
  }
}

String formatTime(float timeValue)
{
  int minutes = (int)timeValue;
  int seconds = (int)((timeValue - minutes) * 100 + 0.5);
  return String(minutes) + ":" + (seconds < 10 ? "0" : "") + String(seconds);
}

void decodeReceivedData(uint8_t *buffer, uint8_t len)
{
  if (len < MIN_PACKET_SIZE)
  {
    Serial.println("Received incomplete data packet: " + String(len) + " bytes");
    return;
  }

  int index = 0;

  // Create a JSON document for data storage
  DynamicJsonDocument doc(512); // Use DynamicJsonDocument with capacity parameter

  // Extract record serial number
  uint32_t sn;
  memcpy(&sn, &buffer[index], 4);
  index += 4;
  doc["record_sn"] = String(sn);

  // Extract time
  float timeValue;
  memcpy(&timeValue, &buffer[index], 4);
  index += 4;

  // Format time for display and JSON
  String timeStr = formatTime(timeValue);
  int minutes = (int)timeValue;
  int seconds = (int)((timeValue - minutes) * 100 + 0.5);
  doc["teensytime"] = String(minutes * 100 + seconds); // Convert MM.SS to MMSS format

  // Extract control flags
  uint8_t nanoFlags = buffer[index++];
  bool nano1 = nanoFlags & 0x01;
  bool nano2 = nanoFlags & 0x02;
  bool nano3 = nanoFlags & 0x04;
  bool nano4 = nanoFlags & 0x08;
  bool remotestate = nanoFlags & 0x10;
  doc["remote_st"] = remotestate ? 1 : 0;
  doc["valve_1"] = nano1 ? 1 : 0;
  doc["valve_2"] = nano2 ? 1 : 0;
  doc["activ_st"] = nano3 ? 1 : 0;
  doc["igni_st"] = nano4 ? 1 : 0;

  // Extract valve state
  bool valveState = buffer[index++];
  doc["para_st"] = valveState ? 1 : 0;

  // Extract analog values
  float analog1, analog2;
  memcpy(&analog1, &buffer[index], 4);
  index += 4;
  memcpy(&analog2, &buffer[index], 4);
  index += 4;
  doc["voltage"] = analog1;
  doc["current"] = analog2;

  // Extract position values
  float xPos, yPos;
  memcpy(&xPos, &buffer[index], 4);
  index += 4;
  memcpy(&yPos, &buffer[index], 4);
  index += 4;
  doc["x_pos"] = xPos;
  doc["y_pos"] = yPos;

  // Extract altitude
  float altitude;
  memcpy(&altitude, &buffer[index], 4);
  index += 4;
  doc["alt"] = altitude;

  // Extract euler angles
  float eulerX, eulerY, eulerZ;
  memcpy(&eulerX, &buffer[index], 4);
  index += 4;
  memcpy(&eulerY, &buffer[index], 4);
  index += 4;
  memcpy(&eulerZ, &buffer[index], 4);
  index += 4;
  doc["eu_x"] = eulerX;
  doc["eu_y"] = eulerY;
  doc["eu_z"] = eulerZ;

  // Extract linear acceleration
  float accelX, accelY, accelZ;
  memcpy(&accelX, &buffer[index], 4);
  index += 4;
  memcpy(&accelY, &buffer[index], 4);
  index += 4;
  memcpy(&accelZ, &buffer[index], 4);
  index += 4;
  doc["acc_x"] = accelX;
  doc["acc_y"] = accelY;
  doc["acc_z"] = accelZ;

  // Extract GPS coordinates
  float gpsLat, gpsLng;
  memcpy(&gpsLat, &buffer[index], 4);
  index += 4;
  memcpy(&gpsLng, &buffer[index], 4);
  index += 4;
  doc["lat"] = gpsLat;
  doc["lon"] = gpsLng;

  // Extract temperature
  float temperature;
  memcpy(&temperature, &buffer[index], 4);
  index += 4;
  doc["teensytemp"] = temperature;

  // Add signal quality data
  doc["rssi"] = rssi;
  doc["snr"] = snr;

  // Print decoded data in CSV format immediately - don't wait for server
  printTelemetryData(sn, timeStr, remotestate, nano1, nano2, nano3, nano4, valveState,
                     analog1, analog2, xPos, yPos, altitude, eulerX, eulerY, eulerZ,
                     accelX, accelY, accelZ, gpsLat, gpsLng, temperature, rssi, snr);

  // Set output pins according to received flag values
  digitalWrite(NANO1_PIN, nano1 ? HIGH : LOW);
  digitalWrite(NANO2_PIN, nano2 ? HIGH : LOW);
  digitalWrite(NANO3_PIN, nano3 ? HIGH : LOW);
  digitalWrite(NANO4_PIN, nano4 ? HIGH : LOW);
  digitalWrite(REMOTE_PIN, remotestate ? HIGH : LOW);
  digitalWrite(VALVE_PIN, valveState ? HIGH : LOW);

  // Store the data for later sending
  lastReceivedData = doc;

  // Don't immediately try to send data - let loop() handle it
}

// New function to separate printing from data processing
void printTelemetryData(uint32_t sn, String timeStr, bool remotestate, bool nano1, bool nano2,
                        bool nano3, bool nano4, bool valveState, float analog1, float analog2,
                        float xPos, float yPos, float altitude, float eulerX, float eulerY,
                        float eulerZ, float accelX, float accelY, float accelZ, float gpsLat,
                        float gpsLng, float temperature, int rssi, float snr)
{
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
}

void sendDataToServer(const DynamicJsonDocument &doc)
{
  String serverURL = preferences.getString("serverurl", "192.168.1.12:5000");
  String fullURL = "http://" + serverURL + "/add_data";

  HTTPClient http;
  http.begin(fullURL);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(250);        // Ultra short timeout of 100ms
  http.setConnectTimeout(250); // Also set connect timeout to 100ms

  String jsonString;
  serializeJson(doc, jsonString);

  int httpResponseCode = http.POST(jsonString);

  http.end();
}
