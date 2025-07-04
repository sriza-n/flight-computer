#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Servo.h>
#include <Wire.h>
#include <util/atomic.h>

// --- Pin Definitions ---
constexpr uint8_t MASTER_INPUT_PIN = A0;
constexpr uint8_t INPUT_PIN1 = 3;
constexpr uint8_t INPUT_PIN2 = 4;
constexpr uint8_t INPUT_PIN3 = 5;
constexpr uint8_t INPUT_PIN4 = 6;
constexpr uint8_t IGNITION_PIN = 8;
constexpr uint8_t VALVE1_SERVO_PIN = 9;
constexpr uint8_t VALVE2_SERVO_PIN = 10;
constexpr uint8_t TEENSY_POWER_PIN = 11;
constexpr uint8_t BUZZER_PIN = 2;
constexpr uint8_t RF433_PIN = 12;

// --- Constants ---
constexpr int MASTER_THRESHOLD = 400;
constexpr int MASTER_DEBOUNCE_COUNT = 20;
constexpr uint8_t TEENSY_I2C_ADDRESS = 0x42;

// --- Task Handles ---
TaskHandle_t TaskSensorHandle, TaskCommHandle;

// --- Semaphore ---
SemaphoreHandle_t xSerialSemaphore;

// --- State Variables ---
volatile bool masterStateHigh = false;
volatile bool manualValveControl = false;
volatile int stableMasterState = LOW;
volatile int masterReadings = 0;

// --- Output States ---
volatile bool teensyPowerState = false;
volatile bool ignitionState = false;
volatile bool outputState1 = false;
volatile bool outputState2 = false;

// --- Previous Input States ---
uint8_t prevInput1 = LOW, prevInput2 = LOW, prevInput3 = LOW, prevInput4 = LOW;

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
    close();
    servo.write(0);
    opened = false;
  }

  void open()
  {
    if (opened && !moving)
      return;
    targetAngle = 180;
    moving = true;
    opened = true;
  }

  void close()
  {
    if (!opened && !moving)
      return;
    targetAngle = 0;
    moving = true;
    opened = false;
  }

  // Call this regularly to update valve position
  void update()
  {
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

// --- Utility Functions ---
inline void Buzz(uint8_t cycles = 1, uint16_t highTime = 100, uint16_t lowTime = 100)
{
  for (uint8_t i = 0; i < cycles; ++i)
  {
    digitalWrite(BUZZER_PIN, HIGH);
    vTaskDelay(highTime / portTICK_PERIOD_MS);
    digitalWrite(BUZZER_PIN, LOW);
    if (i < cycles - 1)
      vTaskDelay(lowTime / portTICK_PERIOD_MS);
  }
}

int getStableMasterState()
{
  int value = analogRead(MASTER_INPUT_PIN);
  int rawState = (value > MASTER_THRESHOLD) ? HIGH : LOW;
  if (rawState == stableMasterState)
  {
    masterReadings = 0;
  }
  else
  {
    masterReadings++;
    if (masterReadings >= MASTER_DEBOUNCE_COUNT)
    {
      stableMasterState = rawState;
      masterReadings = 0;
      Buzz(3, 50, 50);
    }
  }
  return stableMasterState;
}

// --- Task Prototypes ---
void TaskSensorRead(void *);
void TaskCommunication(void *);

// --- Setup ---
void setup()
{
  // Serial.begin(9600);
  xSerialSemaphore = xSemaphoreCreateMutex();
  if (!xSerialSemaphore)
    while (1)
      ;

  xTaskCreate(TaskSensorRead, "Sensor", 128, nullptr, 2, &TaskSensorHandle);
  xTaskCreate(TaskCommunication, "Comm", 128, nullptr, 1, &TaskCommHandle);

  Wire.begin();
  pinMode(MASTER_INPUT_PIN, INPUT);
  pinMode(INPUT_PIN1, INPUT);
  pinMode(INPUT_PIN2, INPUT);
  pinMode(INPUT_PIN3, INPUT);
  pinMode(INPUT_PIN4, INPUT);
  pinMode(IGNITION_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TEENSY_POWER_PIN, OUTPUT);
  pinMode(RF433_PIN, OUTPUT);

  digitalWrite(IGNITION_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(TEENSY_POWER_PIN, LOW);
  digitalWrite(RF433_PIN, HIGH);

  valve1.begin();
  valve2.begin();
}

// --- Main Loop ---
void loop() { /* Empty: all logic in tasks */ }

// --- Task: Sensor Read ---
void TaskSensorRead(void *)
{
  for (;;)
  {
    if (xSemaphoreTake(xSerialSemaphore, 15) == pdTRUE)
    {
      // Update valves continuously
      valve1.update();
      valve2.update();
      int curInput1 = digitalRead(INPUT_PIN1);
      int curInput2 = digitalRead(INPUT_PIN2);

      masterStateHigh = (getStableMasterState() == HIGH);

      if (masterStateHigh)
      {
        digitalWrite(TEENSY_POWER_PIN, HIGH);
        teensyPowerState = true;

        if (curInput1 == HIGH && prevInput1 == LOW)
        {
          outputState1 = !outputState1;
          Buzz(2, 50, 100);
        }

        if (curInput2 == LOW && outputState1 && ignitionStateMachine == IgnitionState::IDLE)
        {
          outputState2 = !outputState2;
          if (outputState2)
          {
            Buzz(3, 50, 100);
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
          outputState1 = false;
          digitalWrite(IGNITION_PIN, LOW);
          if (!outputState2)
          {
            digitalWrite(TEENSY_POWER_PIN, LOW);
            teensyPowerState = false;
          }
        }
        // If ignition sequence is active, let it continue
      }
      prevInput1 = curInput1;

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
        // Wait until both valves finish moving
        if (!valve1.isMoving() && !valve2.isMoving())
        {
          ignitionStateMachine = IgnitionState::WAITING;
          ignitionStartTime = millis();
        }
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
      xSemaphoreGive(xSerialSemaphore);
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// --- Task: Communication & Manual Valve Control ---
void TaskCommunication(void *)
{
  for (;;)
  {
    if (xSemaphoreTake(xSerialSemaphore, 25) == pdTRUE)
    {
      int curInput3 = digitalRead(INPUT_PIN3);
      int curInput4 = digitalRead(INPUT_PIN4);
      manualValveControl = false;

      if (masterStateHigh)
      {
        if (curInput3 == HIGH && prevInput3 == LOW)
        {
          if (valve1.isOpen())
            valve1.close();
          else
            valve1.open();
          Buzz(1, 50, 50);
          manualValveControl = true;
        }
        if (curInput4 == HIGH && prevInput4 == LOW)
        {
          if (valve2.isOpen())
            valve2.close();
          else
            valve2.open();
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
      // Serial.print("Data Packet: ");
      // for (int i = 0; i < 5; i++)
      // {
      //   Serial.println(dataPacket[i]);
      // }

      Wire.beginTransmission(TEENSY_I2C_ADDRESS);
      Wire.write(dataPacket, 5);
      Wire.endTransmission();

      xSemaphoreGive(xSerialSemaphore);
    }
    vTaskDelay(400 / portTICK_PERIOD_MS);
  }
}
