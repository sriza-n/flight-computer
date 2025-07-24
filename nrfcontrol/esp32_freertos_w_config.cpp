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
  String html = "<!DOCTYPE html><html><head><title>Rocket Configuration</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body{font-family:Arial;margin:20px;} .form-group{margin-bottom:15px;} ";
  html += "label{display:block;margin-bottom:5px;} input[type='number']{width:100%;padding:8px;} ";
  html += "button{background-color:#4CAF50;color:white;padding:10px 15px;border:none;cursor:pointer;} ";
  html += "</style></head><body>";
  html += "<h1>🚀Rocket Engine Configuration🚀</h1>";
  html += "<form action='/save' method='post'>";
  html += "<div class='form-group'>";
  html += "<label for='ignDelay'>Valve Open Delay (ms):</label>";
  html += "<input type='number' id='ignDelay' name='ignDelay' value='" + String(ignitionOnDelay) + "' min='500' max='10000'>";
  html += "</div>";
  html += "<div class='form-group'>";
  html += "<label for='waitDelay'>Valve Open Duration (ms):</label>";
  html += "<input type='number' id='waitDelay' name='waitDelay' value='" + String(ignitionWaitingDelay) + "' min='1000' max='90000'>";
  html += "</div>";
  html += "<div class='form-group'>";
  html += "<label for='parsuitedeploy'>Parachute Deploy Altitude Change (m):</label>";
  html += "<input type='number' id='parsuitedeploy' name='parsuitedeploy' value='" + String(parsuitedeploy) + "' min='1' max='100'>";
  html += "</div>";
  html += "<button type='submit'>Save Configuration</button>";
  html += "</form>";
  html += "<p><strong>Current System Status:</strong></p>";
  html += "<p>Master State: " + String(masterStateHigh ? "CONNECTED" : "DISCONNECTED") + "</p>";
  html += "<p>Ignition State: " + String(static_cast<int>(ignitionStateMachine)) + "</p>";
  html += "<p>Valve 1: " + String(valve1.isOpen() ? "OPEN" : "CLOSED") + "</p>";
  html += "<p>Valve 2: " + String(valve2.isOpen() ? "OPEN" : "CLOSED") + "</p>";
  html += "<p>Current Valve Open Delay: " + String(ignitionOnDelay) + " ms</p>";
  html += "<p>Current Valve Open Duration: " + String(ignitionWaitingDelay) + " ms</p>";
  html += "<p>Current Parachute Deploy Altitude Change: " + String(parsuitedeploy) + " m</p>";
  html += "<script>setTimeout(function(){location.reload()},5000);</script>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleSave()
{
  if (server.hasArg("ignDelay") && server.hasArg("waitDelay"))
  {
    ignitionOnDelay = server.arg("ignDelay").toInt();
    ignitionWaitingDelay = server.arg("waitDelay").toInt();
    parsuitedeploy = server.arg("parsuitedeploy").toInt();

    // Apply limits
    ignitionOnDelay = constrain(ignitionOnDelay, 500, 10000);
    ignitionWaitingDelay = constrain(ignitionWaitingDelay, 1000, 90000);
    parsuitedeploy = constrain(parsuitedeploy, 1, 100); // Limit parachute deploy to 1-100 meters

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

  
  Wire.setClock(100000); // 100kHz instead of default 400kHz
  Wire.begin();

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
    if (ConfigMode && !wifiActive)
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
    else if (!ConfigMode && wifiActive)
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
    uint8_t dataPacket[7] = {
        static_cast<uint8_t>(valve1.isOpen()),
        static_cast<uint8_t>(valve2.isOpen()),
        static_cast<uint8_t>(outputState1),
        static_cast<uint8_t>(outputState2),
        static_cast<uint8_t>(masterStateHigh),
        static_cast<uint8_t>(TestMode),
        static_cast<uint8_t>(parsuitedeploy),
    };

    // serial print the data packet
    Serial.print("Data Packet: ");
    for (int i = 0; i < 7; i++)
    {
      Serial.println(dataPacket[i]);
    }

    Wire.beginTransmission(TEENSY_I2C_ADDRESS);
    Wire.write(dataPacket, 7);
    Wire.endTransmission();

        // Add error checking
    // if (result != 0) {
    //   Serial.print("I2C transmission failed, error code: ");
    //   Serial.println(result);
    // } else if (bytesWritten != 7) {
    //   Serial.print("Expected to write 7 bytes, actually wrote: ");
    //   Serial.println(bytesWritten);
    // }

    vTaskDelay(xDelay);
  }
}