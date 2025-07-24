#include <SPI.h>
#include <RH_RF95.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <Wire.h>
// #include <ArduinoJson.h>

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

// Data packet expected size
#define MIN_PACKET_SIZE 16

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
const unsigned long SERVER_RETRY_INTERVAL = 3000; // 3 seconds between server retries      // Use DynamicJsonDocument with capacity parameter

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
// volatile bool testmode = false;
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
// received from nano i2c
volatile bool ConfigMode = false;
volatile bool TestMode = false;
volatile bool connectionState = false;
int servo1Angle = 0;
int servo2Angle = 0;

// --- Packet Processing Variables ---
String timeStr = "";
volatile unsigned long lastPacketTime = 0;
volatile bool newDataAvailable = false;

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

String formatTime(float timeValue);

void sendDataToServer()
{
    String serverURL = preferences.getString("serverurl", "192.168.1.12:5000");
    String fullURL = "http://" + serverURL + "/add_data";

    HTTPClient http;
    http.begin(fullURL);
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(250);
    http.setConnectTimeout(250);

    // Optimized JSON building using snprintf instead of String concatenation
    char jsonBuffer[512]; // Pre-allocated buffer
    snprintf(jsonBuffer, sizeof(jsonBuffer),
             "{\"record_sn\":\"%u\",\"remote_st\":%d,\"valve_1\":%d,\"valve_2\":%d,"
             "\"activ_st\":%d,\"igni_st\":%d,\"para_st\":%d,\"voltage\":%.2f,"
             "\"current\":%.2f,\"x_pos\":%.2f,\"y_pos\":%.2f,\"alt\":%.2f,"
             "\"eu_x\":%.4f,\"eu_y\":%.4f,\"eu_z\":%.4f,\"acc_x\":%.4f,"
             "\"lat\":%.6f,\"lon\":%.6f,\"rssi\":%d,\"snr\":%.1f}",
             sn, remotestate ? 1 : 0, nano1 ? 1 : 0, nano2 ? 1 : 0,
             nano3 ? 1 : 0, nano4 ? 1 : 0, valveState ? 1 : 0,
             analog1, analog2, xPos, yPos, altitude,
             eulerX, eulerY, eulerZ, totalAccel,
             gpsLat, gpsLng, rssi, snr);

    // Send the JSON data
    int httpResponseCode = http.POST(jsonBuffer);

    // Optional: Handle response
    // if (httpResponseCode > 0) {
    //     String response = http.getString();
    //     Serial.printf("HTTP Response: %d\n", httpResponseCode);
    // } else {
    //     Serial.printf("HTTP Error: %d\n", httpResponseCode);
    // }

    http.end();
}

// Function to handle I2C receive events
void receiveEvent(int numBytes)
{
    if (numBytes != 5)
        return; // Early exit

    byte dataPacket[5];
    int i = 0;
    unsigned long startTime = millis();

    while (i < 5 && (millis() - startTime < 5))
    { // Reduce timeout to 5ms
        if (Wire.available())
        {
            dataPacket[i++] = Wire.read();
        }
    }

    if (i == 5)
    { // Only process if all data received
        servo1Angle = dataPacket[0];
        servo2Angle = dataPacket[1];
        ConfigMode = dataPacket[2] > 0;
        TestMode = dataPacket[3] > 0;
        connectionState = dataPacket[4] > 0;
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("ESP32 LoRa Receiver Starting");

    preferences.begin("wifi", false);
    setupLoRa();
    setupOutputPins();
    setupWiFi();

    // Wire.setClock(100000); // 100kHz instead of default 400kHz
    Wire.begin(0x42); // Initialize as slave with address 0x42
    Wire.onReceive(receiveEvent);

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

    // Get LoRa settings from preferences
    int frequency = preferences.getInt("lora_freq", RF95_FREQ);
    int bandwidth = preferences.getInt("lora_bw", 125000);
    int codingRate = preferences.getInt("lora_cr", 5);
    int spreadingFactor = preferences.getInt("lora_sf", 9);

    if (!rf95.setFrequency(frequency))
    {
        Serial.println("Setting frequency failed!");
        while (1)
        {
            delay(100);
        }
    }

    // Configure LoRa parameters from saved settings
    rf95.setSignalBandwidth(bandwidth);
    rf95.setCodingRate4(codingRate);
    rf95.setSpreadingFactor(spreadingFactor);
    rf95.spiWrite(RH_RF95_REG_0C_LNA, 0x23);
    rf95.setModeRx();

    Serial.printf("LoRa initialized: %d MHz, BW=%d Hz, CR=4/%d, SF=%d\n",
                  frequency, bandwidth, codingRate, spreadingFactor);
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

    // Get current LoRa settings
    int currentFreq = preferences.getInt("lora_freq", RF95_FREQ);
    int currentBandwidth = preferences.getInt("lora_bw", 125000);
    int currentCodingRate = preferences.getInt("lora_cr", 5);
    int currentSpreadingFactor = preferences.getInt("lora_sf", 9);

    String html = "<!DOCTYPE html>"
                  "<html>"
                  "<head>"
                  "<title>Ground Station Config</title>"
                  "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
                  "<style>"
                  "body{font-family:Arial,sans-serif;margin:40px;background:#f0f0f0;}"
                  ".container{max-width:500px;margin:0 auto;background:white;padding:30px;border-radius:10px;box-shadow:0 4px 6px rgba(0,0,0,0.1);}"
                  "h1{color:#333;text-align:center;margin-bottom:30px;}"
                  "h2{color:#555;border-bottom:2px solid #4CAF50;padding-bottom:10px;}"
                  "input[type=\"text\"],input[type=\"password\"],select{width:100%;padding:12px;margin:8px 0;border:1px solid #ddd;border-radius:4px;box-sizing:border-box;}"
                  "input[type=\"submit\"]{width:100%;background-color:#4CAF50;color:white;padding:14px;margin:8px 0;border:none;border-radius:4px;cursor:pointer;font-size:16px;}"
                  "input[type=\"submit\"]:hover{background-color:#45a049;}"
                  ".info{background:#e7f3ff;padding:15px;border-radius:4px;margin-bottom:20px;border-left:4px solid #2196F3;}"
                  ".status{background:#e8f5e8;padding:15px;border-radius:4px;margin-bottom:20px;border-left:4px solid #4CAF50;}"
                  ".status.disconnected{background:#ffe8e8;border-left-color:#f44336;}"
                  ".section{margin-bottom:30px;padding:20px;background:#f9f9f9;border-radius:8px;}"
                  ".warning{background:#fff3cd;border:1px solid #ffeaa7;padding:10px;border-radius:4px;color:#856404;margin:10px 0;}"
                  ".footer{text-align:center;margin-top:30px;padding:15px;color:#666;font-size:14px;border-top:1px solid #ddd;}"
                  "</style>"
                  "</head>"
                  "<body>"
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
                  currentServerURL +
                  "</div>"

                  "<form action=\"/config\" method=\"POST\">"

                  "<div class=\"section\">"
                  "<h2>WiFi Configuration</h2>"
                  "<label for=\"ssid\">WiFi Network Name (SSID):</label>"
                  "<input type=\"text\" id=\"ssid\" name=\"ssid\" placeholder=\"Enter WiFi network name\" value=\"" +
                  currentSSID + "\">"
                                "<label for=\"password\">WiFi Password:</label>"
                                "<input type=\"password\" id=\"password\" name=\"password\" placeholder=\"Leave empty to keep current password\">"
                                "<label for=\"serverurl\">Server URL (IP:Port):</label>"
                                "<input type=\"text\" id=\"serverurl\" name=\"serverurl\" placeholder=\"e.g., 192.168.1.12:5000\" value=\"" +
                  currentServerURL + "\">"
                                     "</div>"

                                     "<div class=\"section\">"
                                     "<h2>LoRa Configuration</h2>"
                                     "<div class=\"warning\">&#9888;&#65039; Changing LoRa settings requires restart. Ensure transmitter uses same settings!</div>"

                                     "<label for=\"lora_freq\">Frequency (MHz):</label>"
                                     "<input type=\"number\" id=\"lora_freq\" name=\"lora_freq\" min=\"410\" max=\"525\" step=\"0.1\" value=\"" +
                  String(currentFreq) + "\" placeholder=\"Enter frequency (410-525 MHz)\">"
                                        "<small style=\"color:#666;display:block;margin-top:5px;\">Valid range: 410-525 MHz</small>"

                                        "<label for=\"lora_bw\">Signal Bandwidth (Hz):</label>"
                                        "<select id=\"lora_bw\" name=\"lora_bw\">"
                                        "<option value=\"7800\"" +
                  String(currentBandwidth == 7800 ? " selected" : "") + ">7.8 kHz (Max Range)</option>"
                                                                        "<option value=\"10400\"" +
                  String(currentBandwidth == 10400 ? " selected" : "") + ">10.4 kHz</option>"
                                                                         "<option value=\"15600\"" +
                  String(currentBandwidth == 15600 ? " selected" : "") + ">15.6 kHz</option>"
                                                                         "<option value=\"20800\"" +
                  String(currentBandwidth == 20800 ? " selected" : "") + ">20.8 kHz</option>"
                                                                         "<option value=\"31250\"" +
                  String(currentBandwidth == 31250 ? " selected" : "") + ">31.25 kHz</option>"
                                                                         "<option value=\"41700\"" +
                  String(currentBandwidth == 41700 ? " selected" : "") + ">41.7 kHz</option>"
                                                                         "<option value=\"62500\"" +
                  String(currentBandwidth == 62500 ? " selected" : "") + ">62.5 kHz</option>"
                                                                         "<option value=\"125000\"" +
                  String(currentBandwidth == 125000 ? " selected" : "") + ">125 kHz (Balanced)</option>"
                                                                          "<option value=\"250000\"" +
                  String(currentBandwidth == 250000 ? " selected" : "") + ">250 kHz</option>"
                                                                          "<option value=\"500000\"" +
                  String(currentBandwidth == 500000 ? " selected" : "") + ">500 kHz (Max Speed)</option>"
                                                                          "</select>"

                                                                          "<label for=\"lora_cr\">Coding Rate (4/x):</label>"
                                                                          "<select id=\"lora_cr\" name=\"lora_cr\">"
                                                                          "<option value=\"5\"" +
                  String(currentCodingRate == 5 ? " selected" : "") + ">4/5 (Fast)</option>"
                                                                      "<option value=\"6\"" +
                  String(currentCodingRate == 6 ? " selected" : "") + ">4/6</option>"
                                                                      "<option value=\"7\"" +
                  String(currentCodingRate == 7 ? " selected" : "") + ">4/7</option>"
                                                                      "<option value=\"8\"" +
                  String(currentCodingRate == 8 ? " selected" : "") + ">4/8 (Robust)</option>"
                                                                      "</select>"

                                                                      "<label for=\"lora_sf\">Spreading Factor:</label>"
                                                                      "<select id=\"lora_sf\" name=\"lora_sf\">"
                                                                      "<option value=\"6\"" +
                  String(currentSpreadingFactor == 6 ? " selected" : "") + ">SF6 (Max Speed)</option>"
                                                                           "<option value=\"7\"" +
                  String(currentSpreadingFactor == 7 ? " selected" : "") + ">SF7 (Fast)</option>"
                                                                           "<option value=\"8\"" +
                  String(currentSpreadingFactor == 8 ? " selected" : "") + ">SF8</option>"
                                                                           "<option value=\"9\"" +
                  String(currentSpreadingFactor == 9 ? " selected" : "") + ">SF9 (Balanced)</option>"
                                                                           "<option value=\"10\"" +
                  String(currentSpreadingFactor == 10 ? " selected" : "") + ">SF10</option>"
                                                                            "<option value=\"11\"" +
                  String(currentSpreadingFactor == 11 ? " selected" : "") + ">SF11</option>"
                                                                            "<option value=\"12\"" +
                  String(currentSpreadingFactor == 12 ? " selected" : "") + ">SF12 (Max Range)</option>"
                                                                            "</select>"
                                                                            "</div>"

                                                                            "<input type=\"submit\" value=\"Save Configuration & Restart\">"
                                                                            "</form>"

                                                                            "<div class=\"footer\">"
                                                                            "Developed by <strong>Srijan Koju</strong>"
                                                                            "</div>"

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
    String loraFreq = server.arg("lora_freq");
    String loraBw = server.arg("lora_bw");
    String loraCr = server.arg("lora_cr");
    String loraSf = server.arg("lora_sf");

    // Get current saved SSID
    String currentSSID = preferences.getString("ssid", "");

    // Check if this is a new network or just updating settings
    bool isNewNetwork = (ssid != currentSSID);

    // Validate frequency range
    if (loraFreq.length() > 0)
    {
        float freq = loraFreq.toFloat();
        if (freq < 410.0 || freq > 525.0)
        {
            server.send(400, "text/plain", "Frequency must be between 410-525 MHz");
            return;
        }
    }

    if (ssid.length() > 0 && serverurl.length() > 0)
    {
        // For new networks, require password
        if (isNewNetwork && password.length() == 0)
        {
            server.send(400, "text/plain", "Password required for new WiFi network");
            return;
        }

        // Save WiFi credentials and server URL
        preferences.putString("ssid", ssid);

        // Update password only if provided or if it's a new network
        if (password.length() > 0)
        {
            preferences.putString("password", password);
        }

        preferences.putString("serverurl", serverurl);

        // Save LoRa settings
        if (loraFreq.length() > 0)
            preferences.putInt("lora_freq", loraFreq.toInt());
        if (loraBw.length() > 0)
            preferences.putInt("lora_bw", loraBw.toInt());
        if (loraCr.length() > 0)
            preferences.putInt("lora_cr", loraCr.toInt());
        if (loraSf.length() > 0)
            preferences.putInt("lora_sf", loraSf.toInt());

        // ... rest of success response
        String html = "<!DOCTYPE html>"
                      "<html>"
                      "<body style=\"font-family:Arial,sans-serif;text-align:center;margin:50px;\">"
                      "<div style=\"max-width:400px;margin:0 auto;padding:30px;background:white;border-radius:10px;box-shadow:0 4px 6px rgba(0,0,0,0.1);\">"
                      "<h2>Configuration Saved</h2>"
                      "<p>WiFi and LoRa settings have been saved.</p>"
                      "<p>Device will restart in <span id=\"countdown\">5</span> seconds...</p>"
                      "<script>"
                      "var count = 5;"
                      "setInterval(function(){"
                      "count--;"
                      "document.getElementById('countdown').innerHTML = count;"
                      "if(count <= 0) location.reload();"
                      "}, 1000);"
                      "</script>"
                      "</div>"
                      "</body>"
                      "</html>";

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
        }
    }
    // else
    // {
    //     Serial.printf("%d,%d,%d,%d,%d\n",
    //                   servo1Angle,
    //                   servo2Angle,
    //                   ConfigMode ? 1 : 0,
    //                   TestMode ? 1 : 0,
    //                   connectionState ? 1 : 0);
    //     delay(500);
    // }
}

String formatTime(float timeValue)
{
    int minutes = (int)timeValue;
    int seconds = (int)((timeValue - minutes) * 100 + 0.5);
    return String(minutes) + ":" + (seconds < 10 ? "0" : "") + String(seconds);
}

// Optimized decode function
void decodeReceivedData(uint8_t *buffer, uint8_t len)
{
    int index = 0;

    // Extract data (unchanged)
    memcpy((void *)&sn, &buffer[index], 4);
    index += 4;
    memcpy((void *)&timeValue, &buffer[index], 4);
    index += 4;
    timeStr = formatTime(timeValue);
    // testmode = buffer[index++] != 0;
    remotestate = buffer[index++] != 0;
    nano1 = buffer[index++] != 0;
    nano2 = buffer[index++] != 0;
    nano3 = buffer[index++] != 0;
    nano4 = buffer[index++] != 0;
    memcpy((void *)&analog1, &buffer[index], 4);
    index += 4;
    memcpy((void *)&analog2, &buffer[index], 4);
    index += 4;

    // Single printf call instead of 20+ Serial.print() calls
    // Serial.printf("%d,%d,%d,%d,%d,%u,%s,%d,%d,%d,%d,%d,%.2f,%.2f,%d,%.1f,",
    //               servo1Angle, servo2Angle, ConfigMode ? 1 : 0, TestMode ? 1 : 0,
    //               connectionState ? 1 : 0, sn, timeStr.c_str(),
    //               remotestate ? 1 : 0, nano1 ? 1 : 0, nano2 ? 1 : 0, nano3 ? 1 : 0,
    //               nano4 ? 1 : 0, analog1, analog2, rssi, snr);

    if (TestMode)
    {
        memcpy((void *)&p1Value, &buffer[index], 4);
        index += 4;
        memcpy((void *)&p2Value, &buffer[index], 4);
        index += 4;
        memcpy((void *)&weight, &buffer[index], 4);
        index += 4;
        // Serial.printf("%.2f,%.2f,%.4f\n", p1Value, p2Value, weight);
    }
    else
    {
        // Extract normal mode data...
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

        if (index + 8 <= len)
        {
            memcpy((void *)&gpsLat, &buffer[index], 4);
            index += 4;
            memcpy((void *)&gpsLng, &buffer[index], 4);
            index += 4;
        }

        //     Serial.printf("%d,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%.6f,%.6f\n",
        //                   valveState ? 1 : 0, xPos, yPos, altitude, eulerX, eulerY, eulerZ,
        //                   totalAccel, gpsLat, gpsLng);
    }
    Serial.printf("%u,%s,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%d,%.1f,%.2f,%.2f,%.4f,%d,%d,%d,%d,%d\n",
                  sn, timeStr.c_str(),
                  remotestate ? 1 : 0, nano1 ? 1 : 0, nano2 ? 1 : 0, nano3 ? 1 : 0,
                  nano4 ? 1 : 0, valveState ? 1 : 0, analog1, analog2,
                  xPos, yPos, altitude, eulerX, eulerY, eulerZ,
                  totalAccel, gpsLat, gpsLng, rssi, snr, p1Value, p2Value, weight,
                  servo1Angle, servo2Angle, ConfigMode ? 1 : 0, TestMode ? 1 : 0,
                  connectionState ? 1 : 0);

    // Set output pins...
    digitalWrite(NANO1_PIN, nano1 ? HIGH : LOW);
    digitalWrite(NANO2_PIN, nano2 ? HIGH : LOW);
    digitalWrite(NANO3_PIN, nano3 ? HIGH : LOW);
    digitalWrite(NANO4_PIN, nano4 ? HIGH : LOW);
    digitalWrite(REMOTE_PIN, remotestate ? HIGH : LOW);
    digitalWrite(VALVE_PIN, valveState ? HIGH : LOW);
}
