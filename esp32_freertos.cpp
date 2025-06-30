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

// --- Output States ---
volatile bool haltState = false;
volatile bool readyState = false;

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
uint8_t prevInput1 = LOW, prevInput2 = LOW, prevInput3 = LOW, prevInput4 = LOW;

// --- Servo Angles ---
uint16_t servo1Angle = 180;
uint16_t servo2Angle = 180;

// Task handles
TaskHandle_t ValveI2CTaskHandle = NULL;
TaskHandle_t RadioCmdTaskHandle = NULL;

// Task function declarations
void ValveI2CTask(void *pvParameters);
void RadioCmdTask(void *pvParameters);

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
  bool opened = false;
  const uint8_t step = 1;
  const uint8_t delayMs = 20;

public:
  Valve(uint8_t p) : pin(p) {}
  void begin()
  {
    servo.attach(pin);
    close();
    servo.write(0);
    opened = false;
  }
  void open()
  {
    if (opened)
      return;
    for (int a = angle; a <= 180; a += step)
    {
      servo.write(a);
      vTaskDelay(delayMs / portTICK_PERIOD_MS);
    }
    angle = 180;
    opened = true;
    servo.write(angle);
  }
  void close()
  {
    if (!opened)
      return;
    for (int a = angle; a >= 0; a -= step)
    {
      servo.write(a);
      vTaskDelay(delayMs / portTICK_PERIOD_MS);
    }
    angle = 0;
    opened = false;
    servo.write(angle);
  }
  bool isOpen() const { return opened; }
};

Valve valve1(VALVE1_SERVO_PIN), valve2(VALVE2_SERVO_PIN);

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

void setup()
{
  Serial.begin(115200);
  if (!radio.begin())
  {
    Serial.println("Radio hardware not responding");
    while (1)
      ; // Stop if the module isn't responding
  }
  if (!radio.isChipConnected())
  {
    Serial.println("nRF24L01 module not connected properly.");
    while (1)
      ; // Stop if the module isn't connected
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

  Serial.println("FreeRTOS tasks created successfully");
}

void loop()
{
  // Empty - FreeRTOS scheduler handles tasks
  vTaskDelay(portMAX_DELAY);
}

void RadioCmdTask(void *pvParameters)
{
  const TickType_t xDelay = pdMS_TO_TICKS(50); // 50ms delay
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
        bool newReadyState = (digitalFlags & 0x01) != 0;    // Bit 0
        bool newIgnitionState = (digitalFlags & 0x02) != 0; // Bit 1
        bool newServo1State = (digitalFlags & 0x04) != 0;   // Bit 2
        bool newServo2State = (digitalFlags & 0x08) != 0;   // Bit 3
        bool newHaltState = (digitalFlags & 0x10) != 0;     // Bit 4

        // Extract servo angles (2 bytes each)
        // int16_t servo1Angle, servo2Angle;
        memcpy(&servo1Angle, &buffer[index], 2);
        index += 2;
        memcpy(&servo2Angle, &buffer[index], 2);
        index += 2;

        // Update global states
        readyState = newReadyState;
        haltState = newHaltState;
        ignitionState = newIgnitionState;
        Servo1State = newServo1State;
        Servo2State = newServo2State;
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
          Buzz(2, 50, 100);
          Serial.println("ready on");
        }
      }
      else
      {
        if (outputState1)
        {
          outputState1 = false;
          Serial.println("ready off");
        }
      }

      if (ignitionState == HIGH && ignitionStateMachine == IgnitionState::IDLE)
      {
        if (!outputState2)
        {
          outputState2 = true;
          Serial.println("ignition 1 on");
          Buzz(3, 50, 100);
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
      // --- Ignition State Machine --- (runs independently of masterStateHigh)
      switch (ignitionStateMachine)
      {
      case IgnitionState::IGNITION_ON:
        if (millis() - ignitionStartTime >= 1000)
        {
          valve1.open();
          valve2.open();
          ignitionStateMachine = IgnitionState::VALVES_OPENING;
        }
        break;
      case IgnitionState::VALVES_OPENING:
        ignitionStateMachine = IgnitionState::WAITING;
        ignitionStartTime = millis();
        break;
      case IgnitionState::WAITING:
        if (millis() - ignitionStartTime >= 15000)
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

void ValveI2CTask(void *pvParameters)
{
  const TickType_t xDelay = pdMS_TO_TICKS(300); // 100ms delay

  for (;;)
  {

    int curInput3 = Servo1State;
    int curInput4 = Servo2State;
    manualValveControl = false;

    if (masterStateHigh)
    {
      // HOLDING SWITCH LOGIC: Act while switch is held HIGH
      if (curInput3 == HIGH)
      {
        if (!valve1.isOpen())
        {
          valve1.open();
          Buzz(1, 50, 50);
          manualValveControl = true;
        }
      }
      else
      {
        if (valve1.isOpen())
        {
          valve1.close();
          Buzz(1, 50, 50);
          manualValveControl = true;
        }
      }

      if (curInput4 == HIGH)
      {
        if (!valve2.isOpen())
        {
          valve2.open();
          Buzz(1, 50, 50);
          manualValveControl = true;
        }
      }
      else
      {
        if (valve2.isOpen())
        {
          valve2.close();
          Buzz(1, 50, 50);
          manualValveControl = true;
        }
      }

      if (manualValveControl && ignitionStateMachine != IgnitionState::IDLE)
      {
        ignitionStateMachine = IgnitionState::IDLE;
        outputState2 = false;
        digitalWrite(IGNITION_PIN, LOW);
        valve1.close();
        valve2.close();
      }
    }
    prevInput3 = curInput3;
    prevInput4 = curInput4;

    // --- I2C Data Packet ---
    uint8_t dataPacket[5] = {
        static_cast<uint8_t>(valve1.isOpen()),
        static_cast<uint8_t>(valve2.isOpen()),
        static_cast<uint8_t>(outputState1),
        static_cast<uint8_t>(outputState2),
        static_cast<uint8_t>(masterStateHigh),
    };

    // serial print the data packet
    Serial.print("Data Packet: ");
    for (int i = 0; i < 5; i++)
    {
      Serial.println(dataPacket[i]);
    }

    Wire.beginTransmission(TEENSY_I2C_ADDRESS);
    Wire.write(dataPacket, 5);
    Wire.endTransmission();

    vTaskDelay(xDelay);
  }
}