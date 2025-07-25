#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// --- Pin Definitions ---
constexpr uint8_t NANO1_PIN = 25;
constexpr uint8_t NANO2_PIN = 26;
constexpr uint8_t NANO3_PIN = 27;
constexpr uint8_t NANO4_PIN = 32;
constexpr uint8_t REMOTE_PIN = 33;
constexpr uint8_t VALVE_PIN = 13;

// --- Telemetry State Variables ---
volatile uint32_t sn = 0;
volatile float timeValue = 0.0;
volatile bool remotestate = false;
volatile bool nano1 = false;
volatile bool nano2 = false;
volatile bool nano3 = false;
volatile bool nano4 = false;
volatile bool valveState = false;
volatile float analog1 = 0.0;
volatile float analog2 = 0.0;
volatile float xPos = 0.0;
volatile float yPos = 0.0;
volatile float altitude = 0.0;
volatile float eulerX = 0.0;
volatile float eulerY = 0.0;
volatile float eulerZ = 0.0;
volatile float totalAccel = 0.0;
volatile float gpsLat = 0.0;
volatile float gpsLng = 0.0;
volatile float p1Value = 0.0;
volatile float p2Value = 0.0;
volatile float weight = 0.0;
volatile bool ConfigMode = false;
volatile bool TestMode = false;
volatile bool connectionState = false;
int servo1Angle = 0;
int servo2Angle = 0;

// --- Packet Processing Variables ---
String timeStr = "";
volatile unsigned long lastPacketTime = 0;
volatile bool newDataAvailable = false;

// --- WiFi & Server ---
WebServer server(80);
DNSServer dnsServer;
Preferences preferences;
bool wifiConnected = false;
int rssi = -50;
float snr = 10.0;

// --- XBee ---
#define XBEE_BAUD_RATE 115200
#define PACKET_SIZE 72
#define SERIAL_TIMEOUT 1000

// --- Config Portal ---
#define AP_SSID "GroundStation📥-Config"
#define AP_PASSWORD "logger1234"
#define WIFI_TIMEOUT 10000
#define DNS_PORT 53

// --- FreeRTOS Handles ---
TaskHandle_t xbeeTaskHandle = NULL;
TaskHandle_t serverTaskHandle = NULL;

// --- Function Declarations ---
void setupOutputPins();
void setupWiFi();
void startConfigPortal();
void startWebServer();
void handleRoot();
void handleConfigSubmit();
void handleNotFound();
void decodeReceivedData(uint8_t *buffer, uint8_t len);
String formatTime(float timeValue);
void sendDataToServer();
void receiveEvent(int numBytes);

// --- XBee Helpers ---
bool enterCommandMode();
void sendAT(String cmd);
void configureXBee();

// --- FreeRTOS Tasks ---
void xbeeTask(void *pvParameters);
void serverTask(void *pvParameters);

void setup()
{
  Serial.begin(115200);
  Serial.println("ESP32 LoRa Receiver Starting");

  preferences.begin("wifi", false);
  Serial2.setRxBufferSize(1024);
  Serial2.begin(XBEE_BAUD_RATE, SERIAL_8N1, 16, 17);
  configureXBee();
  setupOutputPins();
  setupWiFi();

  Wire.begin(0x42);
  Wire.onReceive(receiveEvent);

  // Start FreeRTOS tasks
  xTaskCreatePinnedToCore(xbeeTask, "XBeeTask", 4096, NULL, 2, &xbeeTaskHandle, 0);
  xTaskCreatePinnedToCore(serverTask, "ServerTask", 4096, NULL, 1, &serverTaskHandle, 1);

  Serial.println("Receiver ready!");
}

void setupOutputPins()
{
  pinMode(NANO1_PIN, OUTPUT);
  pinMode(NANO2_PIN, OUTPUT);
  pinMode(NANO3_PIN, OUTPUT);
  pinMode(NANO4_PIN, OUTPUT);
  pinMode(REMOTE_PIN, OUTPUT);
  pinMode(VALVE_PIN, OUTPUT);
  digitalWrite(NANO1_PIN, LOW);
  digitalWrite(NANO2_PIN, LOW);
  digitalWrite(NANO3_PIN, LOW);
  digitalWrite(NANO4_PIN, LOW);
  digitalWrite(REMOTE_PIN, LOW);
  digitalWrite(VALVE_PIN, LOW);
}

void setupWiFi()
{
  String ssid = preferences.getString("ssid", "");
  String password = preferences.getString("password", "");
  if (ssid.length() > 0)
  {
    WiFi.begin(ssid.c_str(), password.c_str());
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < WIFI_TIMEOUT)
    {
      delay(500);
    }
    if (WiFi.status() == WL_CONNECTED)
    {
      wifiConnected = true;
      startWebServer();
      return;
    }
  }
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
                "<html><head><title>Ground Station Config</title>"
                "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
                "<style>body{font-family:Arial,sans-serif;margin:40px;background:#f0f0f0;}"
                ".container{max-width:500px;margin:0 auto;background:white;padding:30px;border-radius:10px;box-shadow:0 4px 6px rgba(0,0,0,0.1);}"
                "h1{color:#333;text-align:center;margin-bottom:30px;}"
                "input[type=\"text\"],input[type=\"password\"]{width:100%;padding:12px;margin:8px 0;border:1px solid #ddd;border-radius:4px;box-sizing:border-box;}"
                "input[type=\"submit\"]{width:100%;background-color:#4CAF50;color:white;padding:14px;margin:8px 0;border:none;border-radius:4px;cursor:pointer;font-size:16px;}"
                "input[type=\"submit\"]:hover{background-color:#45a049;}"
                ".status{background:#e8f5e8;padding:15px;border-radius:4px;margin-bottom:20px;border-left:4px solid #4CAF50;}"
                ".status.disconnected{background:#ffe8e8;border-left-color:#f44336;}"
                ".footer{text-align:center;margin-top:30px;padding:15px;color:#666;font-size:14px;border-top:1px solid #ddd;}"
                "</style></head><body>"
                "<div class=\"container\">"
                "<h1>Ground Station Config</h1>"
                "<div class=\"status" +
                String(!wifiConnected ? " disconnected" : "") + "\">"
                                                                "<strong>WiFi Status:</strong> " +
                connectionStatus + "<br>"
                                   "<strong>Current Network:</strong> " +
                (currentSSID.length() > 0 ? currentSSID : "None") + "<br>"
                                                                    "<strong>IP Address:</strong> " +
                currentIP + "<br>"
                            "<strong>Server URL:</strong> " +
                currentServerURL + "</div>"
                                   "<form action=\"/config\" method=\"POST\">"
                                   "<label for=\"ssid\">WiFi Network Name (SSID):</label>"
                                   "<input type=\"text\" id=\"ssid\" name=\"ssid\" placeholder=\"Enter WiFi network name\" value=\"" +
                currentSSID + "\">"
                              "<label for=\"password\">WiFi Password:</label>"
                              "<input type=\"password\" id=\"password\" name=\"password\" placeholder=\"Leave empty to keep current password\">"
                              "<label for=\"serverurl\">Server URL (IP:Port):</label>"
                              "<input type=\"text\" id=\"serverurl\" name=\"serverurl\" placeholder=\"e.g., 192.168.1.12:5000\" value=\"" +
                currentServerURL + "\">"
                                   "<input type=\"submit\" value=\"Save Configuration & Restart\">"
                                   "</form>"
                                   "<div class=\"footer\">Developed by <strong>Srijan Koju</strong></div>"
                                   "</div></body></html>";
  server.send(200, "text/html", html);
}

void handleConfigSubmit()
{
  String ssid = server.arg("ssid");
  String password = server.arg("password");
  String serverurl = server.arg("serverurl");
  String currentSSID = preferences.getString("ssid", "");
  bool isNewNetwork = (ssid != currentSSID);

  if (ssid.length() > 0 && serverurl.length() > 0)
  {
    if (isNewNetwork && password.length() == 0)
    {
      server.send(400, "text/plain", "Password required for new WiFi network");
      return;
    }
    preferences.putString("ssid", ssid);
    if (password.length() > 0)
      preferences.putString("password", password);
    preferences.putString("serverurl", serverurl);

    String html = "<!DOCTYPE html><html><body style=\"font-family:Arial,sans-serif;text-align:center;margin:50px;\">"
                  "<div style=\"max-width:400px;margin:0 auto;padding:30px;background:white;border-radius:10px;box-shadow:0 4px 6px rgba(0,0,0,0.1);\">"
                  "<h2>Configuration Saved</h2>"
                  "<p>WiFi settings have been saved.</p>"
                  "<p>Device will restart in <span id=\"countdown\">5</span> seconds...</p>"
                  "<script>var count = 5;setInterval(function(){count--;document.getElementById('countdown').innerHTML = count;if(count <= 0) location.reload();}, 1000);</script>"
                  "</div></body></html>";
    server.send(200, "text/html", html);
    delay(5000);
    ESP.restart();
  }
  else
  {
    server.send(400, "text/plain", "Invalid SSID or Server URL");
  }
}

void handleNotFound()
{
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

String formatTime(float timeValue)
{
  int minutes = (int)timeValue;
  int seconds = (int)((timeValue - minutes) * 100 + 0.5);
  return String(minutes) + ":" + (seconds < 10 ? "0" : "") + String(seconds);
}

void sendDataToServer()
{
  String serverURL = preferences.getString("serverurl", "192.168.1.12:5000");
  String fullURL = "http://" + serverURL + "/add_data";
  HTTPClient http;
  http.begin(fullURL);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(250);
  http.setConnectTimeout(250);

  char jsonBuffer[1024];
  snprintf(jsonBuffer, sizeof(jsonBuffer),
           "{"
           "\"teensytime\":\"%s\","
           "\"record_sn\":\"%u\","
           "\"voltage\":%.2f,"
           "\"current\":%.2f,"
           "\"remote_st\":%d,"
           "\"valve_1\":%d,"
           "\"valve_2\":%d,"
           "\"activ_st\":%d,"
           "\"igni_st\":%d,"
           "\"para_st\":%d,"
           "\"x_pos\":%.2f,"
           "\"y_pos\":%.2f,"
           "\"alt\":%.2f,"
           "\"eu_x\":%.4f,"
           "\"eu_y\":%.4f,"
           "\"eu_z\":%.4f,"
           "\"acc\":%.4f,"
           "\"lat\":%.6f,"
           "\"lon\":%.6f,"
           "\"rssi\":%.1f,"
           "\"snr\":%.1f,"
           "\"p1\":%.2f,"
           "\"p2\":%.2f,"
           "\"load\":%.1f,"
           "\"servo1_angle\":%d,"
           "\"servo2_angle\":%d,"
           "\"config_mode\":%d,"
           "\"test_mode\":%d,"
           "\"connection_state\":%d"
           "}",
           timeStr.c_str(), sn, analog1, analog2,
           remotestate ? 1 : 0, nano1 ? 1 : 0, nano2 ? 1 : 0,
           nano3 ? 1 : 0, nano4 ? 1 : 0, valveState ? 1 : 0,
           xPos, yPos, altitude,
           eulerX, eulerY, eulerZ, totalAccel,
           gpsLat, gpsLng, (float)rssi, snr,
           p1Value, p2Value, weight,
           servo1Angle, servo2Angle,
           ConfigMode ? 1 : 0, TestMode ? 1 : 0, connectionState ? 1 : 0);

  http.POST(jsonBuffer);
  http.end();
}

void receiveEvent(int numBytes)
{
  if (numBytes != 5)
    return;
  byte dataPacket[5];
  int i = 0;
  unsigned long startTime = millis();
  while (i < 5 && (millis() - startTime < 5))
  {
    if (Wire.available())
      dataPacket[i++] = Wire.read();
  }
  if (i == 5)
  {
    servo1Angle = dataPacket[0];
    servo2Angle = dataPacket[1];
    ConfigMode = dataPacket[2] > 0;
    TestMode = dataPacket[3] > 0;
    connectionState = dataPacket[4] > 0;
  }
}

bool enterCommandMode()
{
  while (Serial2.available())
    Serial2.read();
  delay(1100);
  Serial2.print("+++");
  delay(1100);
  unsigned long start = millis();
  String resp = "";
  while (millis() - start < SERIAL_TIMEOUT)
  {
    if (Serial2.available())
      resp += (char)Serial2.read();
    if (resp.indexOf("OK") != -1)
      return true;
  }
  return false;
}

void sendAT(String cmd)
{
  Serial2.print(cmd + "\r");
  delay(200);
  while (Serial2.available())
    Serial2.read();
}

void configureXBee()
{
  if (!enterCommandMode())
  {
    Serial.println("XBee: Command mode failed");
    return;
  }
  sendAT("ATHP 1");
  sendAT("ATID 1011");
  sendAT("ATCM FFFFFFFE00000000");
  sendAT("ATBD 7");
  sendAT("ATAP 0");
  sendAT("ATNH 1");
  sendAT("ATMT 1");
  sendAT("ATRR 1");
  sendAT("ATCE 0");
  sendAT("ATRO 0");
  sendAT("ATWR");
  sendAT("ATCN");
  Serial.println("XBee: Configured");
}

// --- FreeRTOS Task 1: Read buffer from XBee ---
void xbeeTask(void *pvParameters)
{
  for (;;)
  {
    while (Serial2.available() >= PACKET_SIZE)
    {
      if (Serial2.peek() != 0xAA)
      {
        Serial2.read();
        continue;
      }
      if (Serial2.available() < PACKET_SIZE)
        break;
      uint8_t buffer[PACKET_SIZE];
      size_t bytesRead = Serial2.readBytes(buffer, PACKET_SIZE);
      if (buffer[0] == 0xAA && buffer[PACKET_SIZE - 1] == 0x55)
      {
        decodeReceivedData(buffer, PACKET_SIZE);
        newDataAvailable = true;
      }
      else
      {
        Serial2.read();
      }
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// --- FreeRTOS Task 2: DNS, WebServer, Serial Print, Send Data ---
void serverTask(void *pvParameters)
{
  for (;;)
  {
    if (!wifiConnected)
      dnsServer.processNextRequest();
    server.handleClient();

    if (newDataAvailable)
    {
      Serial.printf("%u,%s,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%d,%.1f,%.2f,%.2f,%.4f,%d,%d,%d,%d,%d\n",
                    sn, timeStr.c_str(),
                    remotestate ? 1 : 0, nano1 ? 1 : 0, nano2 ? 1 : 0, nano3 ? 1 : 0,
                    nano4 ? 1 : 0, valveState ? 1 : 0, analog1, analog2,
                    xPos, yPos, altitude, eulerX, eulerY, eulerZ,
                    totalAccel, gpsLat, gpsLng, rssi, snr, p1Value, p2Value, weight,
                    servo1Angle, servo2Angle, ConfigMode ? 1 : 0, TestMode ? 1 : 0,
                    connectionState ? 1 : 0);
      if (wifiConnected)
        sendDataToServer();
      newDataAvailable = false;
    }

    digitalWrite(NANO1_PIN, nano1 ? HIGH : LOW);
    digitalWrite(NANO2_PIN, nano2 ? HIGH : LOW);
    digitalWrite(NANO3_PIN, nano3 ? HIGH : LOW);
    digitalWrite(NANO4_PIN, nano4 ? HIGH : LOW);
    digitalWrite(REMOTE_PIN, remotestate ? HIGH : LOW);
    digitalWrite(VALVE_PIN, valveState ? HIGH : LOW);

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// --- Decode function ---
void decodeReceivedData(uint8_t *buffer, uint8_t len)
{
  int index = 1;
  memcpy((void *)&sn, &buffer[index], 4);
  index += 4;
  memcpy((void *)&timeValue, &buffer[index], 4);
  index += 4;
  timeStr = formatTime(timeValue);
  remotestate = buffer[index++] != 0;
  nano1 = buffer[index++] != 0;
  nano2 = buffer[index++] != 0;
  nano3 = buffer[index++] != 0;
  nano4 = buffer[index++] != 0;
  memcpy((void *)&analog1, &buffer[index], 4);
  index += 4;
  memcpy((void *)&analog2, &buffer[index], 4);
  index += 4;
  memcpy((void *)&p1Value, &buffer[index], 4);
  index += 4;
  memcpy((void *)&p2Value, &buffer[index], 4);
  index += 4;
  memcpy((void *)&weight, &buffer[index], 4);
  index += 4;
  valveState = buffer[index++] != 0;
  memcpy((void *)&xPos, &buffer[index], 4);
  index += 4;
  memcpy((void *)&yPos, &buffer[index], 4);
  index += 4;
  memcpy((void *)&altitude, &buffer[index], 4);
  index += 4;
  memcpy((void *)&eulerX, &buffer[index], 4);
  index += 4;
  memcpy((void *)&eulerY, &buffer[index], 4);
  index += 4;
  memcpy((void *)&eulerZ, &buffer[index], 4);
  index += 4;
  memcpy((void *)&totalAccel, &buffer[index], 4);
  index += 4;
  memcpy((void *)&gpsLat, &buffer[index], 4);
  index += 4;
  memcpy((void *)&gpsLng, &buffer[index], 4);
  index += 4;
}

void loop()
{
}
