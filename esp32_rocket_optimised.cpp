#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <DNSServer.h>

// NRF configuration
RF24 radio(4, 5); // CE, CSN
const byte addresses[][10] = {"00001", "00002"};
const int payloadSize = 32;

// I2C address for Teensy
constexpr uint8_t TEENSY_I2C_ADDRESS = 0x42;

// Pin definitions
constexpr uint8_t BUZZER_PIN = 2;
constexpr uint8_t IGNITION_PIN = 12;
constexpr uint8_t VALVE1_SERVO_PIN = 13;
constexpr uint8_t VALVE2_SERVO_PIN = 14;
constexpr uint8_t TEENSY_POWER_PIN = 15;

// Configurable parameters
unsigned long ignitionOnDelay = 1000;
unsigned long ignitionWaitingDelay = 15000;
unsigned long parsuitedeploy = 5;

// WiFi and WebServer settings
const char *ssid = "RocketConfig🚀";
const char *password = "rocket1234";
WebServer server(80);
Preferences preferences;
DNSServer dnsServer;
#define DNS_PORT 53

// Output states
volatile bool haltState = false;
volatile bool readyState = false;
volatile bool TestMode = false;
volatile bool ConfigMode = false;

// State variables
volatile bool masterStateHigh = false;
volatile bool manualValveControl = false;
volatile bool teensyPowerState = false;
volatile bool ignitionState = false;
volatile bool outputState1 = false;
volatile bool outputState2 = false;
volatile bool Servo1State = false;
volatile bool Servo2State = false;

// Servo angles
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

// Utility functions
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

void playTone(uint16_t frequency, uint16_t durationMs)
{
    tone(BUZZER_PIN, frequency, durationMs);
    delay(durationMs);
    noTone(BUZZER_PIN);
}

// Ignition state machine
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

// Valve class
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
        moving = (angle != targetAngle);
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

    void update(int desiredAngle)
    {
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
    preferences.end();
}

void savePreferences()
{
    preferences.begin("rocket-config", false);
    preferences.putULong("ignOnDelay", ignitionOnDelay);
    preferences.putULong("ignWaitDelay", ignitionWaitingDelay);
    preferences.putULong("parsuitedeploy", parsuitedeploy);
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

    html += "<input type=\"submit\" value=\"Save Configuration\">";
    html += "</form>";

    html += "<div class=\"footer\">By Srijan Koju</div>";
    html += "</div></body></html>";

    server.send(200, "text/html", html);
}

void handleSave()
{
    if (server.hasArg("ignDelay") && server.hasArg("waitDelay"))
    {
        ignitionOnDelay = server.arg("ignDelay").toInt();
        ignitionWaitingDelay = server.arg("waitDelay").toInt();
        parsuitedeploy = server.arg("parsuitedeploy").toInt();

        ignitionOnDelay = constrain(ignitionOnDelay, 500, 10000);
        ignitionWaitingDelay = constrain(ignitionWaitingDelay, 1000, 90000);
        parsuitedeploy = constrain(parsuitedeploy, 0, 100);

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
    loadPreferences();

    if (!radio.begin())
    {
        Serial.println("Radio hardware not responding");
    }
    if (!radio.isChipConnected())
    {
        Serial.println("nRF24L01 module not connected properly.");
    }
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(108);
    radio.enableAckPayload();
    radio.setAutoAck(true);
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
    Wire.setClock(400000);

    xTaskCreate(ValveI2CTask, "ValveI2CTask", 2048, NULL, 2, &ValveI2CTaskHandle);
    xTaskCreate(RadioCmdTask, "RadioCmdTask", 2048, NULL, 1, &RadioCmdTaskHandle);
    xTaskCreate(WiFiServerTask, "WiFiServerTask", 4096, NULL, 3, &WiFiServerTaskHandle);

    Serial.println("FreeRTOS tasks created successfully");
}

void loop()
{
    vTaskDelay(portMAX_DELAY);
}

void RadioCmdTask(void *pvParameters)
{
    const TickType_t xDelay = pdMS_TO_TICKS(25);
    unsigned long lastRadioAvailableTime = 0;

    for (;;)
    {
        if (radio.available())
        {
            uint8_t buffer[32];
            uint8_t len = radio.getDynamicPayloadSize();
            if (len > sizeof(buffer)) len = sizeof(buffer);
            radio.read(&buffer, len);

            if (len >= 5)
            {
                int index = 0;
                uint8_t digitalFlags = buffer[index++];
                readyState = (digitalFlags & 0x01) != 0;
                ignitionState = (digitalFlags & 0x02) != 0;
                Servo1State = (digitalFlags & 0x04) != 0;
                Servo2State = (digitalFlags & 0x08) != 0;
                haltState = (digitalFlags & 0x10) != 0;
                TestMode = (digitalFlags & 0x20) != 0;
                ConfigMode = (digitalFlags & 0x40) != 0;
                memcpy(&servo1Angle, &buffer[index], 2); index += 2;
                memcpy(&servo2Angle, &buffer[index], 2); index += 2;
            }
            lastRadioAvailableTime = millis();
        }

        masterStateHigh = (millis() - lastRadioAvailableTime <= 500);

        if (masterStateHigh)
        {
            digitalWrite(TEENSY_POWER_PIN, HIGH);
            teensyPowerState = true;
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
    const TickType_t xDelay = pdMS_TO_TICKS(100);
    bool wifiActive = false;
    WiFi.mode(WIFI_OFF);

    for (;;)
    {
        if (ConfigMode && !wifiActive)
        {
            WiFi.mode(WIFI_AP);
            WiFi.softAP(ssid, password);
            Serial.print("Access Point started: ");
            Serial.println(ssid);
            Serial.print("IP address: ");
            Serial.println(WiFi.softAPIP());
            dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
            server.on("/", handleRoot);
            server.on("/config", handleRoot);
            server.on("/save", HTTP_POST, handleSave);
            server.onNotFound(handleRoot);
            server.begin();
            Serial.println("HTTP server started");
            wifiActive = true;
        }
        else if (!ConfigMode && wifiActive)
        {
            server.close();
            dnsServer.stop();
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
            Serial.println("WiFi Access Point stopped");
            wifiActive = false;
        }
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
    const TickType_t xDelay = pdMS_TO_TICKS(100);
    for (;;)
    {
        if (masterStateHigh && ignitionStateMachine == IgnitionState::IDLE)
        {
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
        uint8_t dataPacket[7] = {
            static_cast<uint8_t>(valve1.isOpen()),
            static_cast<uint8_t>(valve2.isOpen()),
            static_cast<uint8_t>(outputState1),
            static_cast<uint8_t>(outputState2),
            static_cast<uint8_t>(masterStateHigh),
            static_cast<uint8_t>(TestMode),
            static_cast<uint8_t>(parsuitedeploy)
        };
        Wire.beginTransmission(TEENSY_I2C_ADDRESS);
        Wire.write(dataPacket, 7);
        Wire.endTransmission();
        vTaskDelay(xDelay);
    }
}
