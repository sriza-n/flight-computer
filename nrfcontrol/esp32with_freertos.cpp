#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <Servo.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"

// Nrf configuration
RF24 radio(4, 5); // CE, CSN
const byte addresses[][10] = {"00001", "00002"};
const int payloadSize = 32; // Size of payload buffer

constexpr uint8_t TEENSY_I2C_ADDRESS = 0x42;

// --- Pin Definitions ---
constexpr uint8_t BUZZER_PIN = 2;
constexpr uint8_t IGNITION_PIN = 12;
constexpr uint8_t VALVE1_SERVO_PIN = 13;
constexpr uint8_t VALVE2_SERVO_PIN = 14;
constexpr uint8_t TEENSY_POWER_PIN = 15;

// --- Output States ---
volatile bool readyState = false;
volatile bool ignitionState = false;
volatile bool servo1State = false;
volatile bool servo2State = false;
volatile bool haltState = false;
volatile bool connectionState = false;
volatile bool outputState2 = false;
volatile bool manualValveControl = false;

// --- Servo Angles ---
uint16_t servo1Angle = 180;
uint16_t servo2Angle = 180;

// --- Previous Input States ---
uint8_t prevInput1 = LOW, prevInput2 = LOW, prevInput3 = LOW, prevInput4 = LOW;

// --- FreeRTOS Synchronization ---
SemaphoreHandle_t xStateMutex;
SemaphoreHandle_t xServoMutex;

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

// // --- Valve Class ---
class Valve
{
  static Servo servoControl; // Static shared servo control object
  uint8_t pin;
  int angle = 0;
  bool opened = false;
  const uint8_t step = 1;
  const uint8_t delayMs = 25;

public:
  Valve(uint8_t p) : pin(p) {}
  void begin()
  {
    servoControl.attach(pin);
    close();
    opened = false;
  }

  void open(int targetAngle = 180)
  {
    // Constrain target angle to valid servo range
    targetAngle = constrain(targetAngle, 0, 180);

    if (opened && angle == targetAngle)
      return;

    // Move to target angle
    if (angle < targetAngle)
    {
      for (int a = angle; a <= targetAngle; a += step)
      {
        servoControl.write(pin, a);
        vTaskDelay(pdMS_TO_TICKS(delayMs));
      }
    }
    else if (angle > targetAngle)
    {
      for (int a = angle; a >= targetAngle; a -= step)
      {
        servoControl.write(pin, a);
        vTaskDelay(pdMS_TO_TICKS(delayMs));
      }
    }

    angle = targetAngle;
    opened = (targetAngle > 0);
    servoControl.write(pin, angle);
  }

  void close()
  {
    if (!opened)
      return;
    for (int a = angle; a >= 0; a -= step)
    {
      servoControl.write(pin, a);
      vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
    angle = 0;
    opened = false;
    servoControl.write(pin, angle);
  }
  bool isOpen() const { return opened; }
  int getCurrentAngle() const { return angle; }
};

// Initialize the static member here
Servo Valve::servoControl;

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

// --- FreeRTOS Task Functions ---

// Task for handling radio communication
void radioTask(void *pvParameters)
{
  for (;;)
  {
    if (radio.available())
    {
      uint8_t buffer[16];
      uint8_t len = radio.getDynamicPayloadSize();
      radio.read(&buffer, len);

      // Check if this is a ping packet (1 byte with value 0xFF)
      if (len == 1 && buffer[0] == 0xFF)
      {
        // Switch to transmit mode and send acknowledgment
        radio.stopListening();
        uint8_t ackData = 0xAA; // Acknowledgment byte
        bool success = radio.write(&ackData, 1);

        if (xSemaphoreTake(xStateMutex, portMAX_DELAY))
        {
          connectionState = success;
          xSemaphoreGive(xStateMutex);
        }

        radio.startListening(); // Return to listening mode
        continue;
      }

      int index = 0;

      // Extract digital pin states from first byte
      uint8_t digitalFlags = buffer[index++];

      if (xSemaphoreTake(xStateMutex, portMAX_DELAY))
      {
        readyState = (digitalFlags & 0x01) != 0;    // Bit 0
        ignitionState = (digitalFlags & 0x02) != 0; // Bit 1
        servo1State = (digitalFlags & 0x04) != 0;   // Bit 2
        servo2State = (digitalFlags & 0x08) != 0;   // Bit 3
        haltState = (digitalFlags & 0x10) != 0;     // Bit 4
        xSemaphoreGive(xStateMutex);
      }

      // Extract servo angles (2 bytes each)
      if (xSemaphoreTake(xServoMutex, portMAX_DELAY))
      {
        memcpy(&servo1Angle, &buffer[index], 2);
        index += 2;
        memcpy(&servo2Angle, &buffer[index], 2);
        index += 2;
        xSemaphoreGive(xServoMutex);
      }

      Serial.print("Binary data received (");
      Serial.print(len);
      Serial.println(" bytes):");
      Serial.print("Ready=");
      Serial.print(readyState);
      Serial.print(", Ignition=");
      Serial.print(ignitionState);
      Serial.print(", Servo1=");
      Serial.print(servo1State);
      Serial.print(", Servo2=");
      Serial.print(servo2State);
      Serial.print(", Halt=");
      Serial.print(haltState);
      Serial.print(" | Servo1Angle=");
      Serial.print(servo1Angle);
      Serial.print(", Servo2Angle=");
      Serial.print(servo2Angle);
      // Serial.print(", Timestamp="); Serial.println(timestamp);
      Serial.println("---");
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent task starvation
  }
}

// Task for processing commands and ignition state machine
void commandProcessingTask(void *pvParameters)
{
  for (;;)
  {
    bool localConnectionState, localReadyState, localIgnitionState;

    if (xSemaphoreTake(xStateMutex, portMAX_DELAY))
    {
      localConnectionState = connectionState;
      localReadyState = readyState;
      localIgnitionState = ignitionState;
      xSemaphoreGive(xStateMutex);
    }

    if (localConnectionState)
    {
      digitalWrite(TEENSY_POWER_PIN, HIGH);

      if (localReadyState)
      {
        Buzz(2, 50, 500);
      }

      if (localIgnitionState && localReadyState && ignitionStateMachine == IgnitionState::IDLE)
      {
        outputState2 = !outputState2;
        if (outputState2)
        {
          Buzz(3, 50, 500);
          digitalWrite(IGNITION_PIN, HIGH);
          ignitionStateMachine = IgnitionState::IGNITION_ON;
          ignitionStartTime = millis();
        }
      }
    }
    else
    {
      // Instead of turning off ignition immediately, we only power down if not in the ignition sequence
      if (ignitionStateMachine == IgnitionState::IDLE)
      {
        digitalWrite(IGNITION_PIN, LOW);
        if (!outputState2)
        {
          digitalWrite(TEENSY_POWER_PIN, LOW);
        }
      }
    }

    // --- Ignition State Machine --- (runs independently of readyState)
    switch (ignitionStateMachine)
    {
    case IgnitionState::IGNITION_ON:
      if (millis() - ignitionStartTime >= 1000)
      {
        uint16_t localServo1Angle, localServo2Angle;
        if (xSemaphoreTake(xServoMutex, portMAX_DELAY))
        {
          localServo1Angle = servo1Angle;
          localServo2Angle = servo2Angle;
          xSemaphoreGive(xServoMutex);
        }
        valve1.open(localServo1Angle);
        valve2.open(localServo2Angle);
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

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// Task for valve control
void valveControlTask(void *pvParameters)
{
  for (;;)
  {
    int curInput3 = servo1State ? HIGH : LOW;
    int curInput4 = servo2State ? HIGH : LOW;
    manualValveControl = false;

    if (connectionState)
    {
      if (curInput3 == HIGH )
      {
        if (valve1.isOpen())
          valve1.close();
        else
          valve1.open(servo1Angle);
        Buzz(1, 50, 50);
        manualValveControl = true;
      }
      if (curInput4 == HIGH)
      {
        if (valve2.isOpen())
          valve2.close();
        else
          valve2.open(servo2Angle);
        Buzz(1, 50, 50);
        manualValveControl = true;
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

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Task for I2C communication
void i2cTask(void *pvParameters)
{
  for (;;)
  {
    bool localReadyState;

    if (xSemaphoreTake(xStateMutex, portMAX_DELAY))
    {
      localReadyState = readyState;
      xSemaphoreGive(xStateMutex);
    }

    // --- I2C Data Packet ---
    uint8_t dataPacket[5] = {
        static_cast<uint8_t>(valve1.isOpen()),
        static_cast<uint8_t>(valve2.isOpen()),
        static_cast<uint8_t>(localReadyState),
        static_cast<uint8_t>(outputState2),
        static_cast<uint8_t>(haltState),
    };

    Wire.beginTransmission(TEENSY_I2C_ADDRESS);
    Wire.write(dataPacket, 5);
    Wire.endTransmission();

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup()
{
  Serial.begin(115200);

  // Create mutexes
  xStateMutex = xSemaphoreCreateMutex();
  xServoMutex = xSemaphoreCreateMutex();

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
  radio.setDataRate(RF24_2MBPS);
  radio.setChannel(108);

  // Enable auto-ack
  radio.enableAckPayload();
  radio.setAutoAck(true);

  // Enable dynamic payloads
  radio.enableDynamicPayloads();

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

  // Create FreeRTOS tasks
  xTaskCreate(radioTask, "RadioTask", 4096, NULL, 3, NULL);
  xTaskCreate(commandProcessingTask, "CommandTask", 4096, NULL, 2, NULL);
  xTaskCreate(valveControlTask, "ValveTask", 4096, NULL, 2, NULL);
  xTaskCreate(i2cTask, "I2CTask", 2048, NULL, 1, NULL);

  Serial.println("FreeRTOS tasks created successfully");
}

void loop()
{
  // Empty loop - all functionality handled by FreeRTOS tasks
  // vTaskDelay(pdMS_TO_TICKS(1000));
}
