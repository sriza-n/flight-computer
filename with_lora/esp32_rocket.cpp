#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
// for i2c comm
#include <Wire.h>
#include <ESP32Servo.h>
// for nrf
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
// WiFi and WebServer includes
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <DNSServer.h>

// Nrf configuration
RF24 radio(4, 5); // CE, CSN
const byte addresses[][10] = {"00001", "00002"};
const int payloadSize = 32; // Size of payload buffer

// for i2c comm with teensy 4.1
constexpr uint8_t TEENSY_I2C_ADDRESS = 0x42;

// --- Pin Definitions ---
constexpr uint8_t BUZZER_PIN = 2;
constexpr uint8_t IGNITION_PIN = 12;
constexpr uint8_t VALVE1_SERVO_PIN = 13;
constexpr uint8_t VALVE2_SERVO_PIN = 14;
constexpr uint8_t TEENSY_POWER_PIN = 15;

// --- Configurable parameters ---
unsigned long ignitionOnDelay = 1000;       // Default 1 second
unsigned long ignitionWaitingDelay = 15000; // Default 15 seconds
unsigned long parsuitedeploy = 5;           // 5 meter change in altitude to deploy parachute

// WiFi and WebServer settings
const char *ssid = "RocketConfig🚀";
const char *password = "rocket1234";
WebServer server(80);
Preferences preferences;

DNSServer dnsServer;
#define DNS_PORT 53

// --- Output States ---
volatile bool haltState = false;
volatile bool readyState = false;
volatile bool TestMode = false;
volatile bool ConfigMode = false;

// --- State Variables ---
volatile bool masterStateHigh = false;
volatile bool manualValveControl = false;

volatile bool teensyPowerState = false;
volatile bool ignitionState = false;
volatile bool outputState1 = false;
volatile bool outputState2 = false;
volatile bool Servo1State = false;
volatile bool Servo2State = false;

// --- Previous Input States ---
// uint8_t prevInput1 = LOW, prevInput2 = LOW, prevInput3 = LOW, prevInput4 = LOW;

// Add after the existing configurable parameters
float loraFreq = 433.0;          // Default frequency in MHz
long loraBandwidth = 125000;     // Default bandwidth in Hz
uint8_t loraCodingRate = 5;      // Default coding rate (4/5)
uint8_t loraSpreadingFactor = 9; // Default spreading factor

// --- Servo Angles ---
uint16_t servo1Angle = 180;
uint16_t servo2Angle = 180;

// Task handles
TaskHandle_t ValveI2CTaskHandle = NULL;
TaskHandle_t RadioCmdTaskHandle = NULL;
TaskHandle_t WiFiServerTaskHandle = NULL;

// Task function declarations
void ValveI2CTask(void *pvParameters);
void RadioCmdTask(void *pvParameters);
void WiFiServerTask(void *pvParameters);

// // --- Utility Functions ---
inline void Buzz(uint8_t cycles = 1, uint16_t highTime = 100, uint16_t lowTime = 100)
{
    for (uint8_t i = 0; i < cycles; ++i)
    {
        digitalWrite(BUZZER_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(highTime));
        digitalWrite(BUZZER_PIN, LOW);
        if (i < cycles - 1)
            vTaskDelay(pdMS_TO_TICKS(lowTime));
    }
}

// Play a tone on the buzzer
void playTone(uint16_t frequency, uint16_t durationMs)
{
    tone(BUZZER_PIN, frequency, durationMs);
    delay(durationMs);  // Wait for tone to finish
    noTone(BUZZER_PIN); // Ensure buzzer is off
}
// --- Ignition State Machine ---
enum class IgnitionState : uint8_t
{
    IDLE,
    IGNITION_ON,
    VALVES_OPENING,
    WAITING,
    VALVES_CLOSING,
    COMPLETE
};
volatile IgnitionState ignitionStateMachine = IgnitionState::IDLE;
unsigned long ignitionStartTime = 0;

// --- Valve Class ---
class Valve
{
    Servo servo;
    uint8_t pin;
    int angle = 0;
    int targetAngle = 0;
    bool opened = false;
    bool moving = false;
    unsigned long lastMoveTime = 0;
    const uint8_t step = 1;
    const uint8_t delayMs = 1;

public:
    Valve(uint8_t p) : pin(p) {}

    void begin()
    {
        servo.attach(pin);
        angle = 0;
        targetAngle = 180;
        servo.write(angle);
        opened = false;
        moving = false;
    }

    void setTargetAngle(int angleValue)
    {
        targetAngle = angleValue;
        moving = (angle != targetAngle); // Only move if different
    }

    void open(int angleValue)
    {
        Buzz(1, 500, 0);
        if (opened && !moving)
            return;
        setTargetAngle(angleValue);
        opened = true;
    }

    void close()
    {
        if (!opened && !moving)
            return;
        setTargetAngle(0);
        opened = false;
    }

    // Call this regularly to update valve position
    void update(int desiredAngle)
    {
        // Only allow real-time update if valve is open
        if (opened && targetAngle != desiredAngle)
        {
            setTargetAngle(desiredAngle);
        }

        if (!moving)
            return;

        unsigned long currentTime = millis();
        if (currentTime - lastMoveTime >= delayMs)
        {
            if (angle < targetAngle)
            {
                angle += step;
                if (angle > targetAngle)
                    angle = targetAngle;
            }
            else if (angle > targetAngle)
            {
                angle -= step;
                if (angle < targetAngle)
                    angle = targetAngle;
            }

            servo.write(angle);
            lastMoveTime = currentTime;

            if (angle == targetAngle)
            {
                moving = false;
            }
        }
    }

    bool isOpen() const { return opened; }
    bool isMoving() const { return moving; }
};

Valve valve1(VALVE1_SERVO_PIN), valve2(VALVE2_SERVO_PIN);

void loadPreferences()
{
    preferences.begin("rocket-config", false);
    ignitionOnDelay = preferences.getULong("ignOnDelay", 1000);
    ignitionWaitingDelay = preferences.getULong("ignWaitDelay", 15000);
    parsuitedeploy = preferences.getULong("parsuitedeploy", 5);

    loraFreq = preferences.getFloat("loraFreq", 433.0);
    loraBandwidth = preferences.getLong("loraBW", 125000);
    loraCodingRate = preferences.getUChar("loraCR", 5);
    loraSpreadingFactor = preferences.getUChar("loraSF", 9);

    preferences.end();
}

void savePreferences()
{
    preferences.begin("rocket-config", false);
    preferences.putULong("ignOnDelay", ignitionOnDelay);
    preferences.putULong("ignWaitDelay", ignitionWaitingDelay);
    preferences.putULong("parsuitedeploy", parsuitedeploy);

    preferences.putFloat("loraFreq", loraFreq);
    preferences.putLong("loraBW", loraBandwidth);
    preferences.putUChar("loraCR", loraCodingRate);
    preferences.putUChar("loraSF", loraSpreadingFactor);
    preferences.end();
}

void handleRoot()
{
    String html = "<!DOCTYPE html>"
                  "<html>"
                  "<head>"
                  "<title>Rocket Engine Configuration</title>"
                  "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
                  "<style>"
                  "body{font-family:Arial,sans-serif;margin:40px;background:#f0f0f0;}"
                  ".container{max-width:500px;margin:0 auto;background:white;padding:30px;border-radius:10px;box-shadow:0 4px 6px rgba(0,0,0,0.1);}"
                  "h1{color:#333;text-align:center;margin-bottom:30px;}"
                  "h2{color:#555;border-bottom:2px solid #4CAF50;padding-bottom:10px;}"
                  "input[type=\"number\"],select{width:100%;padding:12px;margin:8px 0;border:1px solid #ddd;border-radius:4px;box-sizing:border-box;}"
                  "input[type=\"submit\"]{width:100%;background-color:#4CAF50;color:white;padding:14px;margin:8px 0;border:none;border-radius:4px;cursor:pointer;font-size:16px;}"
                  "input[type=\"submit\"]:hover{background-color:#45a049;}"
                  ".section{margin-bottom:30px;padding:20px;background:#f9f9f9;border-radius:8px;}"
                  ".warning{background:#fff3cd;border:1px solid #ffeaa7;padding:15px;border-radius:4px;color:#856404;margin:10px 0;}"
                  "label{display:block;margin:15px 0 5px 0;font-weight:bold;color:#333;}"
                  "small{color:#666;display:block;margin-top:5px;font-size:12px;}"
                  ".footer{text-align:center;margin-top:30px;padding:15px;color:#666;font-size:14px;border-top:1px solid #ddd;}"
                  "</style>"
                  "</head>"
                  "<body>"
                  "<div class=\"container\">"
                  "<h1>🚀 Rocket Engine Configuration 🚀</h1>";

    html += "<form action=\"/save\" method=\"POST\">";

    // Engine Configuration Section
    html += "<div class=\"section\">"
            "<h2>Engine Configuration</h2>"
            "<label for=\"ignDelay\">Valve Open Delay (ms):</label>"
            "<input type=\"number\" id=\"ignDelay\" name=\"ignDelay\" value=\"" +
            String(ignitionOnDelay) + "\" min=\"500\" max=\"10000\">"
                                      "<small>Time delay before valves open after ignition (500-10000ms)</small>"

                                      "<label for=\"waitDelay\">Valve Open Duration (ms):</label>"
                                      "<input type=\"number\" id=\"waitDelay\" name=\"waitDelay\" value=\"" +
            String(ignitionWaitingDelay) + "\" min=\"1000\" max=\"90000\">"
                                           "<small>How long valves stay open during ignition (1000-90000ms)</small>"

                                           "<label for=\"parsuitedeploy\">Parachute Deploy Altitude Change (m):</label>"
                                           "<input type=\"number\" id=\"parsuitedeploy\" name=\"parsuitedeploy\" value=\"" +
            String(parsuitedeploy) + "\" min=\"0\" max=\"100\">"
                                     "<small>Altitude change threshold for parachute deployment (0-100m)</small>"
                                     "</div>";

    // LoRa Configuration Section
    html += "<div class=\"section\">"
            "<h2>LoRa Configuration</h2>"
            "<div class=\"warning\">&#9888;&#65039; Changing LoRa settings requires restart. Ensure transmitter uses same settings!</div>"

            "<label for=\"lora_freq\">Frequency (MHz):</label>"
            "<input type=\"number\" id=\"lora_freq\" name=\"lora_freq\" min=\"410\" max=\"525\" step=\"0.1\" value=\"" +
            String(loraFreq) + "\">"
                               "<small>Valid range: 410-525 MHz</small>"

                               "<label for=\"lora_bw\">Signal Bandwidth (Hz):</label>"
                               "<select id=\"lora_bw\" name=\"lora_bw\">"
                               "<option value=\"7800\"" +
            String(loraBandwidth == 7800 ? " selected" : "") + ">7.8 kHz (Max Range)</option>"
                                                               "<option value=\"10400\"" +
            String(loraBandwidth == 10400 ? " selected" : "") + ">10.4 kHz</option>"
                                                                "<option value=\"15600\"" +
            String(loraBandwidth == 15600 ? " selected" : "") + ">15.6 kHz</option>"
                                                                "<option value=\"20800\"" +
            String(loraBandwidth == 20800 ? " selected" : "") + ">20.8 kHz</option>"
                                                                "<option value=\"31250\"" +
            String(loraBandwidth == 31250 ? " selected" : "") + ">31.25 kHz</option>"
                                                                "<option value=\"41700\"" +
            String(loraBandwidth == 41700 ? " selected" : "") + ">41.7 kHz</option>"
                                                                "<option value=\"62500\"" +
            String(loraBandwidth == 62500 ? " selected" : "") + ">62.5 kHz</option>"
                                                                "<option value=\"125000\"" +
            String(loraBandwidth == 125000 ? " selected" : "") + ">125 kHz (Balanced)</option>"
                                                                 "<option value=\"250000\"" +
            String(loraBandwidth == 250000 ? " selected" : "") + ">250 kHz</option>"
                                                                 "<option value=\"500000\"" +
            String(loraBandwidth == 500000 ? " selected" : "") + ">500 kHz (Max Speed)</option>"
                                                                 "</select>"

                                                                 "<label for=\"lora_cr\">Coding Rate (4/x):</label>"
                                                                 "<select id=\"lora_cr\" name=\"lora_cr\">"
                                                                 "<option value=\"5\"" +
            String(loraCodingRate == 5 ? " selected" : "") + ">4/5 (Fast)</option>"
                                                             "<option value=\"6\"" +
            String(loraCodingRate == 6 ? " selected" : "") + ">4/6</option>"
                                                             "<option value=\"7\"" +
            String(loraCodingRate == 7 ? " selected" : "") + ">4/7</option>"
                                                             "<option value=\"8\"" +
            String(loraCodingRate == 8 ? " selected" : "") + ">4/8 (Robust)</option>"
                                                             "</select>"

                                                             "<label for=\"lora_sf\">Spreading Factor:</label>"
                                                             "<select id=\"lora_sf\" name=\"lora_sf\">"
                                                             "<option value=\"6\"" +
            String(loraSpreadingFactor == 6 ? " selected" : "") + ">SF6 (Max Speed)</option>"
                                                                  "<option value=\"7\"" +
            String(loraSpreadingFactor == 7 ? " selected" : "") + ">SF7 (Fast)</option>"
                                                                  "<option value=\"8\"" +
            String(loraSpreadingFactor == 8 ? " selected" : "") + ">SF8</option>"
                                                                  "<option value=\"9\"" +
            String(loraSpreadingFactor == 9 ? " selected" : "") + ">SF9 (Balanced)</option>"
                                                                  "<option value=\"10\"" +
            String(loraSpreadingFactor == 10 ? " selected" : "") + ">SF10</option>"
                                                                   "<option value=\"11\"" +
            String(loraSpreadingFactor == 11 ? " selected" : "") + ">SF11</option>"
                                                                   "<option value=\"12\"" +
            String(loraSpreadingFactor == 12 ? " selected" : "") + ">SF12 (Max Range)</option>"
                                                                   "</select>"
                                                                   "</div>";

    html += "<input type=\"submit\" value=\"Save Configuration\">";
    html += "</form>";

    html += "<div class=\"footer\">"
            "By Srijan Koju"
            "</div>";

    html += "</div>"
            "</body>"
            "</html>";

    server.send(200, "text/html", html);
}

void handleSave()
{
    if (server.hasArg("ignDelay") && server.hasArg("waitDelay"))
    {
        // Engine parameters
        ignitionOnDelay = server.arg("ignDelay").toInt();
        ignitionWaitingDelay = server.arg("waitDelay").toInt();
        parsuitedeploy = server.arg("parsuitedeploy").toInt();

        // Apply limits for engine parameters
        ignitionOnDelay = constrain(ignitionOnDelay, 500, 10000);
        ignitionWaitingDelay = constrain(ignitionWaitingDelay, 1000, 90000);
        parsuitedeploy = constrain(parsuitedeploy, 0, 100);

        // LoRa parameters
        if (server.hasArg("lora_freq"))
        {
            loraFreq = server.arg("lora_freq").toFloat();
            loraFreq = constrain(loraFreq, 410.0, 525.0);
        }
        if (server.hasArg("lora_bw"))
        {
            loraBandwidth = server.arg("lora_bw").toInt();
        }
        if (server.hasArg("lora_cr"))
        {
            loraCodingRate = server.arg("lora_cr").toInt();
            loraCodingRate = constrain(loraCodingRate, 5, 8);
        }
        if (server.hasArg("lora_sf"))
        {
            loraSpreadingFactor = server.arg("lora_sf").toInt();
            loraSpreadingFactor = constrain(loraSpreadingFactor, 6, 12);
        }

        savePreferences();

        server.sendHeader("Location", "/");
        server.send(303);
    }
    else
    {
        server.send(400, "text/plain", "Missing parameters");
    }
}

void setup()
{
    Serial.begin(115200);

    // Load saved preferences
    loadPreferences();

    if (!radio.begin())
    {
        Serial.println("Radio hardware not responding");
        // while (1)
        //   ; // Stop if the module isn't responding
    }
    if (!radio.isChipConnected())
    {
        Serial.println("nRF24L01 module not connected properly.");
        // while (1)
        //   ; // Stop if the module isn't connected
    }
    // Enhanced radio configuration
    radio.setPALevel(RF24_PA_MAX);
    // radio.setDataRate(RF24_2MBPS);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(108);

    // Enable auto-ack
    radio.enableAckPayload();
    radio.setAutoAck(true);

    // Enable dynamic payloads
    // radio.enableDynamicPayloads();

    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1, addresses[0]);
    radio.startListening();

    pinMode(IGNITION_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(TEENSY_POWER_PIN, OUTPUT);

    digitalWrite(IGNITION_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(TEENSY_POWER_PIN, LOW);

    valve1.begin();
    valve2.begin();

    Wire.begin();
    Wire.setClock(400000); // 100kHz instead of default 400kHz

    // Create ValveI2CTask
    xTaskCreate(
        ValveI2CTask,       // Task function
        "ValveI2CTask",     // Task name
        2048,               // Stack size (bytes)
        NULL,               // Parameters
        2,                  // Priority
        &ValveI2CTaskHandle // Task handle
    );

    // Create RadioCmdTask
    xTaskCreate(
        RadioCmdTask,       // Task function
        "RadioCmdTask",     // Task name
        2048,               // Stack size (bytes)
        NULL,               // Parameters
        1,                  // Priority
        &RadioCmdTaskHandle // Task handle
    );

    // Create WiFiServerTask
    xTaskCreate(
        WiFiServerTask,       // Task function
        "WiFiServerTask",     // Task name
        4096,                 // Stack size (bytes)
        NULL,                 // Parameters
        3,                    // Priority
        &WiFiServerTaskHandle // Task handle
    );

    Serial.println("FreeRTOS tasks created successfully");
}

void loop()
{
    // Empty - FreeRTOS scheduler handles tasks
    vTaskDelay(portMAX_DELAY);
}

void RadioCmdTask(void *pvParameters)
{
    const TickType_t xDelay = pdMS_TO_TICKS(25); // 50ms delay
    unsigned long lastRadioAvailableTime = 0;

    for (;;)
    {
        // Check if data is available on the radio
        if (radio.available())
        {
            uint8_t buffer[32]; // Use payloadSize constant
            uint8_t len = radio.getDynamicPayloadSize();

            // Ensure we don't exceed buffer size
            if (len > sizeof(buffer))
            {
                len = sizeof(buffer);
            }

            radio.read(&buffer, len);

            // Process command data
            if (len >= 5) // Minimum expected packet size (1 byte flags + 4 bytes servo angles)
            {
                int index = 0;

                // Extract digital pin states from first byte
                uint8_t digitalFlags = buffer[index++];

                // Update global state variables based on received flags
                readyState = (digitalFlags & 0x01) != 0;    // Bit 0
                ignitionState = (digitalFlags & 0x02) != 0; // Bit 1
                Servo1State = (digitalFlags & 0x04) != 0;   // Bit 2
                Servo2State = (digitalFlags & 0x08) != 0;   // Bit 3
                haltState = (digitalFlags & 0x10) != 0;     // Bit 4
                TestMode = (digitalFlags & 0x20) != 0;      // Bit 5
                ConfigMode = (digitalFlags & 0x40) != 0;    // Bit 6

                // Extract servo angles (2 bytes each)
                // int16_t servo1Angle, servo2Angle;
                memcpy(&servo1Angle, &buffer[index], 2);
                index += 2;
                memcpy(&servo2Angle, &buffer[index], 2);
                index += 2;
            }

            // Serial.print(readyState);
            // Serial.print(",");
            // Serial.print(ignitionState);
            // Serial.print(",");
            // Serial.print(Servo1State);
            // Serial.print(",");
            // Serial.print(Servo2State);
            // Serial.print(",");
            // Serial.print(haltState);
            // Serial.print(",");
            // Serial.print(servo1Angle);
            // Serial.print(",");
            // Serial.println(servo2Angle);

            lastRadioAvailableTime = millis();
        }

        // Set masterStateHigh true if radio.available() in last 500ms, else false
        if (millis() - lastRadioAvailableTime <= 500)
        {
            masterStateHigh = true;
        }
        else
        {
            masterStateHigh = false;
        }

        if (masterStateHigh)
        {
            digitalWrite(TEENSY_POWER_PIN, HIGH);
            teensyPowerState = true;

            // HOLDING SWITCH LOGIC: Act while switch is held HIGH
            if (readyState == HIGH)
            {
                if (!outputState1)
                {
                    outputState1 = true;
                    Buzz(2, 500, 500);
                }
            }
            else
            {
                if (outputState1)
                {
                    outputState1 = false;
                }
            }

            if (ignitionState == HIGH && ignitionStateMachine == IgnitionState::IDLE)
            {
                if (!outputState2)
                {
                    outputState2 = true;
                    Buzz(3, 500, 500);
                    digitalWrite(IGNITION_PIN, HIGH);
                    ignitionStateMachine = IgnitionState::IGNITION_ON;
                    ignitionStartTime = millis();
                }
            }
        }
        else
        {
            // ...existing power-down logic...
            if (ignitionStateMachine == IgnitionState::IDLE)
            {
                outputState1 = false;
                digitalWrite(IGNITION_PIN, LOW);
                if (!outputState2)
                {
                    digitalWrite(TEENSY_POWER_PIN, LOW);
                    teensyPowerState = false;
                }
            }
        }
        valve1.update(servo1Angle);
        valve2.update(servo2Angle);
        // --- Ignition State Machine --- (runs independently of masterStateHigh)
        switch (ignitionStateMachine)
        {
        case IgnitionState::IGNITION_ON:
            if (millis() - ignitionStartTime >= ignitionOnDelay)
            {
                valve1.open(servo1Angle);
                valve2.open(servo2Angle);
                ignitionStateMachine = IgnitionState::VALVES_OPENING;
            }
            break;
        case IgnitionState::VALVES_OPENING:
            // Wait until both valves finish moving
            if (!valve1.isMoving() && !valve2.isMoving())
            {
                ignitionStateMachine = IgnitionState::WAITING;
                ignitionStartTime = millis();
            }
            break;
        case IgnitionState::WAITING:
            if (millis() - ignitionStartTime >= ignitionWaitingDelay)
            {
                ignitionStateMachine = IgnitionState::VALVES_CLOSING;
            }
            break;
        case IgnitionState::VALVES_CLOSING:
            valve1.close();
            valve2.close();
            ignitionStateMachine = IgnitionState::COMPLETE;
            break;
        case IgnitionState::COMPLETE:
            outputState2 = false;
            digitalWrite(IGNITION_PIN, LOW);
            ignitionStateMachine = IgnitionState::IDLE;
            break;
        case IgnitionState::IDLE:
        default:
            break;
        }

        vTaskDelay(xDelay);
    }
}

void WiFiServerTask(void *pvParameters)
{
    const TickType_t xDelay = pdMS_TO_TICKS(100); // 100ms delay
    bool wifiActive = false;

    // Initially turn off WiFi
    WiFi.mode(WIFI_OFF);

    for (;;)
    {
        // Check if halt state has changed
        // if (ConfigMode && !wifiActive)
        if (masterStateHigh && !wifiActive)
        {
            // Activate WiFi only when halt state becomes HIGH
            WiFi.mode(WIFI_AP);
            WiFi.softAP(ssid, password);

            Serial.print("HALT detected: Access Point started: ");
            Serial.println(ssid);
            Serial.print("IP address: ");
            Serial.println(WiFi.softAPIP());

            dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

            // Setup server routes
            server.on("/", handleRoot);       // Root page
            server.on("/config", handleRoot); // Config page
            server.on("/save", HTTP_POST, handleSave);
            server.onNotFound(handleRoot); // Catch all other requests

            // Start server
            server.begin();
            Serial.println("HTTP server started");

            // Buzz to indicate WiFi is now active
            // Buzz(1, 500, 0);

            wifiActive = true;
        }
        else if (!masterStateHigh && wifiActive)
        {
            // Deactivate WiFi when halt state becomes LOW
            server.close();
            dnsServer.stop();
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);

            Serial.println("HALT deactivated: WiFi Access Point stopped");

            wifiActive = false;
        }

        // Only handle client requests when WiFi is active
        if (wifiActive)
        {
            dnsServer.processNextRequest();
            server.handleClient();
        }

        vTaskDelay(xDelay);
    }
}

void ValveI2CTask(void *pvParameters)
{
    const TickType_t xDelay = pdMS_TO_TICKS(100); // 100ms delay

    for (;;)
    {

        if (masterStateHigh && ignitionStateMachine == IgnitionState::IDLE)
        {
            // HOLDING SWITCH LOGIC: Act while switch is held HIGH
            if (Servo1State == HIGH)
            {
                if (!valve1.isOpen())
                {
                    valve1.open(servo1Angle);
                }
            }
            else
            {
                if (valve1.isOpen())
                {
                    valve1.close();
                }
            }

            if (Servo2State == HIGH)
            {
                if (!valve2.isOpen())
                {
                    valve2.open(servo2Angle);
                }
            }
            else
            {
                if (valve2.isOpen())
                {
                    valve2.close();
                }
            }

            if (haltState && ignitionStateMachine != IgnitionState::IDLE)
            {
                ignitionStateMachine = IgnitionState::IDLE;
                outputState2 = false;
                digitalWrite(IGNITION_PIN, LOW);
                valve1.close();
                valve2.close();
            }
        }

        // --- I2C Data Packet ---
        uint16_t freqInt = static_cast<uint16_t>(loraFreq * 10); // Convert to integer first
        uint8_t dataPacket[15] = {                               // Increased size from 7 to 15
                                  static_cast<uint8_t>(valve1.isOpen()),
                                  static_cast<uint8_t>(valve2.isOpen()),
                                  static_cast<uint8_t>(outputState1),
                                  static_cast<uint8_t>(outputState2),
                                  static_cast<uint8_t>(masterStateHigh),
                                  static_cast<uint8_t>(TestMode),
                                  static_cast<uint8_t>(parsuitedeploy),
                                  // LoRa configuration (8 bytes)
                                  static_cast<uint8_t>(freqInt & 0xFF),               // Frequency low byte (x10 for precision)
                                  static_cast<uint8_t>((freqInt >> 8) & 0xFF),        // Frequency high byte
                                  static_cast<uint8_t>(loraBandwidth & 0xFF),         // Bandwidth byte 1
                                  static_cast<uint8_t>((loraBandwidth >> 8) & 0xFF),  // Bandwidth byte 2
                                  static_cast<uint8_t>((loraBandwidth >> 16) & 0xFF), // Bandwidth byte 3
                                  static_cast<uint8_t>((loraBandwidth >> 24) & 0xFF), // Bandwidth byte 4
                                  loraCodingRate,
                                  loraSpreadingFactor};

        // serial print the data packet
        Serial.print("Data Packet: ");
        for (int i = 0; i < 15; i++)
        {
            Serial.println(dataPacket[i]);
        }

        Wire.beginTransmission(TEENSY_I2C_ADDRESS);
        Wire.write(dataPacket, 15);
        Wire.endTransmission();
        // size_t bytesWritten = Wire.write(dataPacket, 15); // Write the data packet
        // int result = Wire.endTransmission();
        // // Add error checking
        // if (result != 0)
        // {
        //     Serial.print("I2C transmission failed, error code: ");
        //     Serial.println(result);
        // }
        // else if (bytesWritten != 15)
        // {
        //     Serial.print("Expected to write 7 bytes, actually wrote: ");
        //     Serial.println(bytesWritten);
        // }

        vTaskDelay(xDelay);
    }
}