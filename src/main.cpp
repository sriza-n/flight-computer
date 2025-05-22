#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Servo.h>
#include <Wire.h>

// for freertos tasks-----------------------------------------------
TaskHandle_t Task1Handle;
TaskHandle_t Task2Handle;
TaskHandle_t Task3Handle;
TaskHandle_t Task4Handle;

// Semaphore for protecting shared resources
SemaphoreHandle_t xSerialSemaphore;

// Task functions prototypes
void TaskSensorRead(void *pvParameters);
// void TaskDataProcess(void *pvParameters);
// void TaskActuatorControl(void *pvParameters);
void TaskCommunication(void *pvParameters);

// ----------input pins to get status of 433mhz receiver-----------------
const int masterInputPin = A0; // Master control input
const int inputPin1 = 3;
const int inputPin2 = 4;
const int inputPin3 = 5;
const int inputPin4 = 6;
// Define custom voltage threshold for master input (assuming 5V reference)
const int masterThreshold = 400;

// Master input debounce variables
const int masterDebounceCount = 20; // Number of consecutive readings needed to change state
int masterReadings = 0;             // Counter for consistent readings
int stableMasterState = LOW;        // Current stable state of master input

const int ignitionpin = 8;
const int teensypower = 11;
const int buzzerPin = 2;
const int rf433pin = 12;

bool teensystate = LOW;
bool ignitionstate = LOW;
bool valve1state = LOW;
bool valve2state = LOW;

// Variables to store previous input states for edge detection
int prevInput1 = LOW;
int prevInput2 = LOW;
int prevInput3 = LOW;
int prevInput4 = LOW;

// Variables to store output states
int outputState1 = LOW;
int outputState2 = LOW;
int outputState3 = LOW;
int outputState4 = LOW;
// Define the Teensy's I2C address
#define TEENSY_I2C_ADDRESS 0x42

// Ignition state machine variables
enum IgnitionState
{
  IDLE,
  IGNITION_ON,
  VALVES_OPENING,
  WAITING,
  VALVES_CLOSING,
  COMPLETE
};

IgnitionState currentIgnitionState = IDLE;
unsigned long ignitionStartTime = 0;

// Valve class to encapsulate servo movement for valve operations
class Valve
{
private:
  Servo servo;      // The servo motor controlling this valve
  int servoPin;     // Pin the servo is attached to
  int currentAngle; // Current angle of the servo
  bool isOpened;    // Current state of the valve
  int delayMs;      // Delay between angle increments (ms)

public:
  // Constructor
  Valve(int pin, int delay = 10)
  {
    servoPin = pin;
    delayMs = delay;
    currentAngle = 0;
    isOpened = false;
  }

  // Initialize the valve
  void begin()
  {
    servo.attach(servoPin);
    servo.write(0); // Start at closed position (0 degrees)
    currentAngle = 0;
  }

  // Open the valve (sweep from 0 to 180 degrees)
  void open()
  {
    if (isOpened)
      return; // Skip if already open

    Serial.println("Opening valve on pin " + String(servoPin));
    // Step by 5 degrees instead of 1 degree (5x faster)
    for (int angle = 0; angle <= 180; angle += 1)
    {
      servo.write(angle);
      currentAngle = angle;
      delay(delayMs);
    }
    // Make sure it reaches the final position
    servo.write(180);
    currentAngle = 180;
    isOpened = true;
  }

  // Close the valve (sweep from 180 to 0 degrees)
  void close()
  {
    if (!isOpened)
      return; // Skip if already closed

    Serial.println("Closing valve on pin " + String(servoPin));
    // Step by 5 degrees instead of 1 degree (5x faster)
    for (int angle = 180; angle >= 0; angle -= 1)
    {
      servo.write(angle);
      currentAngle = angle;
      delay(delayMs);
    }
    // Make sure it reaches the final position
    servo.write(0);
    currentAngle = 0;
    isOpened = false;
  }

  // Check if valve is open
  bool isOpen()
  {
    return isOpened;
  }

  // Get current position
  int getPosition()
  {
    return currentAngle;
  }
};

// Create valve objects
Valve valve1(9); // First valve using servo on pin 9
Valve valve2(10);

// Function to control the buzzer
/**
 * Custom buzzer function
 * @param cycles Number of HIGH/LOW cycles
 * @param highTime Time in ms to stay HIGH
 * @param lowTime Time in ms to stay LOW
 */
void Buzz(int cycles = 1, int highTime = 100, int lowTime = 100)
{
  for (int i = 0; i < cycles; i++)
  {
    digitalWrite(buzzerPin, HIGH);
    delay(highTime);
    digitalWrite(buzzerPin, LOW);

    // Only delay after buzzer cycles (except the last one)
    if (i < cycles - 1)
    {
      delay(lowTime);
    }
  }
}

/**
 * Get the current stable state of the master input
 * @return Current debounced state of the master input (HIGH or LOW)
 */
int getStableMasterState()
{
  // Read the analog value of master input pin
  int masterValue = analogRead(masterInputPin);
  int rawMasterState = (masterValue > masterThreshold) ? HIGH : LOW;

  // Debounce the master input
  if (rawMasterState == stableMasterState)
  {
    // Input matches current state, reset the counter
    masterReadings = 0;
  }
  else
  {
    // Input differs from current state, increment the counter
    masterReadings++;

    // If we've seen enough consistent readings, change the state
    if (masterReadings >= masterDebounceCount)
    {
      stableMasterState = rawMasterState;
      masterReadings = 0;

      Serial.print("Stable master state changed to: ");
      Serial.println(stableMasterState ? "HIGH" : "LOW");
      Buzz(3, 100, 100); // 3 buzzes, 100ms on, 100ms off
    }
  }
  return stableMasterState;
}

void setup()
{
  // Initialize serial communication
  Serial.begin(9600);

  // // Wait for serial port to connect for native USB devices
  // while (!Serial) {
  //   ; // wait for serial port to connect
  // }

  // Create semaphore before starting tasks
  xSerialSemaphore = xSemaphoreCreateMutex();

  if (xSerialSemaphore == NULL)
  {
    Serial.println("Mutex creation failed");
    // Don't continue if we couldn't create the semaphore
    while (1)
      ;
  }

  // Create tasks
  xTaskCreate(
      TaskSensorRead, // Task function
      "SensorRead",   // Task name for humans
      128,            // Stack size
      NULL,           // Parameters
      1,              // Priority (higher number = higher priority)
      &Task1Handle    // Task handle
  );

  // xTaskCreate(
  //     TaskDataProcess,
  //     "DataProcess",
  //     128,
  //     NULL,
  //     3,
  //     &Task2Handle);

  // xTaskCreate(
  //     TaskActuatorControl,
  //     "ActuatorControl",
  //     128,
  //     NULL,
  //     4, // Higher priority for control tasks
  //     &Task3Handle);

  xTaskCreate(
      TaskCommunication,
      "Communication",
      128,
      NULL,
      2,
      &Task4Handle);

  // The scheduler is started automatically after setup()
  Serial.println("Initializing nano...");
  Wire.begin(); // Initialize I2C as master
  pinMode(masterInputPin, INPUT);
  pinMode(inputPin1, INPUT);
  pinMode(inputPin2, INPUT);
  pinMode(inputPin3, INPUT);
  pinMode(inputPin4, INPUT);

  // Initialize output pins
  pinMode(ignitionpin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(teensypower, OUTPUT);
  pinMode(rf433pin, OUTPUT);

  // Initialize all outputs to LOW
  digitalWrite(ignitionpin, LOW);
  digitalWrite(buzzerPin, LOW);
  digitalWrite(teensypower, LOW);
  digitalWrite(rf433pin, HIGH); // Set RF433 pin HIGH
  // Optional: Start serial for debugging

  // Initialize valves
  valve1.begin();
  valve2.begin();
  Serial.println("Valves initialized.");
}

void loop()
{
  // Empty. Things are done in Tasks.
}

// Task 1: Read sensors
void TaskSensorRead(void *pvParameters)
{
  for (;;)
  { // A Task shall never return or exit
    // Take the semaphore before accessing Serial
    if (xSemaphoreTake(xSerialSemaphore, (TickType_t)10) == pdTRUE)
    {
      Serial.println("Task 1");
      // Read current input states
      // Read current input states
      int currentInput1 = digitalRead(inputPin1);
      int currentInput2 = digitalRead(inputPin2);

      // read state of master input
      if (getStableMasterState() == HIGH)
      {
        digitalWrite(teensypower, HIGH);

        if (currentInput1 == HIGH && prevInput1 == LOW)
        {
          outputState1 = !outputState1; // Toggle output state
          Serial.print("Output 1 toggled to: ");
          Serial.println(outputState1 ? "HIGH" : "LOW");
          Buzz(2, 100, 900);  // 2 buzzes, 100ms on, 100ms off
        }

        // Only allow input2 to work if outputState1 is HIGH (input1 has been activated)
        if (currentInput2 == LOW && outputState1 == HIGH && currentIgnitionState == IDLE)
        {
          outputState2 = !outputState2;
          Serial.print("Output 2 toggled to: ");
          Serial.println(outputState2 ? "HIGH" : "LOW");

          if (outputState2)
          {
            // Start the ignition sequence
            Buzz(3, 100, 900);
            digitalWrite(ignitionpin, HIGH);
            digitalWrite(rf433pin, LOW); // Set RF433 pin LOW
            currentIgnitionState = IGNITION_ON;
            ignitionStartTime = millis();
          }
        }
      }
      else
      {
        // Do something when master input is LOW
        outputState1 = LOW;
        // digitalWrite(ignitionpin, LOW);
        // Only set teensypower LOW if outputState2 is also LOW
        if (outputState2 == LOW)
        {
          digitalWrite(teensypower, LOW);
        }
      }
      prevInput1 = currentInput1;
      // prevInput2 = currentInput2;

      // Handle ignition state machine
      switch (currentIgnitionState)
      {
      case IGNITION_ON:
        if (millis() - ignitionStartTime >= 1000)
        {
          Serial.println("Opening valves...");
          valve1.open();
          valve2.open();
          currentIgnitionState = VALVES_OPENING;
        }
        break;

      case VALVES_OPENING:
        currentIgnitionState = WAITING;
        ignitionStartTime = millis();
        Serial.println("Waiting for 15 seconds...");
        break;

      case WAITING:
        if (millis() - ignitionStartTime >= 15000)
        {
          Serial.println("Closing valves...");
          currentIgnitionState = VALVES_CLOSING;
        }
        break;

      case VALVES_CLOSING:
        valve1.close();
        valve2.close();
        currentIgnitionState = COMPLETE;
        Serial.println("Ignition sequence complete");
        break;

      case COMPLETE:
        currentIgnitionState = IDLE;
        break;

      case IDLE:
        // Nothing to do in idle state
        break;
      }

      // ---------------------------------------------------
      xSemaphoreGive(xSerialSemaphore); // Release the semaphore
    }

    vTaskDelay(40 / portTICK_PERIOD_MS); // 100ms delay
  }
}

// // Task 2: Process data
// void TaskDataProcess(void *pvParameters)
// {
//   for (;;)
//   {
//     if (xSemaphoreTake(xSerialSemaphore, (TickType_t)10) == pdTRUE)
//     {
//       Serial.println("Running Task 2");

//       xSemaphoreGive(xSerialSemaphore);
//     }

//     vTaskDelay(130 / portTICK_PERIOD_MS); // 200ms delay
//   }
// }

// // Task 3: Control actuators
// void TaskActuatorControl(void *pvParameters)
// {
//   for (;;)
//   {
//     if (xSemaphoreTake(xSerialSemaphore, (TickType_t)10) == pdTRUE)
//     {
//       Serial.println("Running Task 3");

//       xSemaphoreGive(xSerialSemaphore);
//     }

//     vTaskDelay(130 / portTICK_PERIOD_MS); // 500ms delay
//   }
// }

// Task 4: Communication/logging
void TaskCommunication(void *pvParameters)
{
  for (;;)
  {
    if (xSemaphoreTake(xSerialSemaphore, (TickType_t)10) == pdTRUE)
    {
      Serial.println("Task 2");
      // ------------------valve control-----------------------
      int currentInput3 = digitalRead(inputPin3);
      int currentInput4 = digitalRead(inputPin4);
      if (getStableMasterState() == HIGH)
      { 
        // Add this check to see if manual valve control is triggered
        bool manualControl = false;
        if (currentInput3 == HIGH && prevInput3 == LOW)
        { 
          // manualControl = true;
          outputState3 = !outputState3;
          Serial.print("Output 3 toggled to: ");
          Serial.println(outputState3 ? "HIGH" : "LOW");
          // toggle valve1
          if (outputState3 == HIGH)
          {
            valve1.open();     // Open the first valve
            Buzz(1, 100, 100); // 3 buzzes, 100ms on, 900ms off
          }
          else
          {
            valve1.close(); // Close the first valve
            Buzz(1, 100, 100);
          }
        }

        if (currentInput4 == HIGH && prevInput4 == LOW)
        { 
          manualControl = true;
          outputState4 = !outputState4;
          Serial.print("Output 4 toggled to: ");
          Serial.println(outputState4 ? "HIGH" : "LOW");
          // toggle valve2
          if (outputState4 == HIGH)
          {
            valve2.open();     // Open the second valve
            Buzz(1, 100, 100); // 3 buzzes, 100ms on, 900ms off
          }
          else
          {
            valve2.close(); // Close the second valve
            Buzz(1, 100, 100);
          }
        }
        // If manual control occurred, interrupt the ignition sequence
        if (manualControl && currentIgnitionState != IDLE) {
            Serial.println("Manual valve control detected - interrupting ignition sequence");
            currentIgnitionState = IDLE;
            outputState2 = LOW; // Reset output state
        }
      }
      // else
      // {
      // }
      prevInput3 = currentInput3;
      prevInput4 = currentInput4;
      // Create data packet with 4 bytes to transmit output states
      byte dataPacket[4];
      dataPacket[0] = valve1.isOpen() ? 1 : 0;
      dataPacket[1] = valve2.isOpen() ? 1 : 0;
      dataPacket[2] = outputState1 ? 1 : 0;
      dataPacket[3] = outputState2 ? 1 : 0; // Add the new value

    //serial print the data packet
      // Serial.print("Data Packet: ");
      // for (int i = 0; i < 4; i++)
      // {
      //   Serial.println(dataPacket[i]);
      // }

      // Send data to Teensy
      Wire.beginTransmission(TEENSY_I2C_ADDRESS);
      Wire.write(dataPacket, 4); // Now sending 4 bytes
      // byte error = Wire.endTransmission();
      Wire.endTransmission();

      // if (error == 0)
      // {
      //   Serial.println("Data sent successfully");
      // }
      // else
      // {
      //   Serial.print("Error sending data: ");
      //   Serial.println(error);
      // }
      xSemaphoreGive(xSerialSemaphore);
    }

    vTaskDelay(100 / portTICK_PERIOD_MS); // 1 second delay
  }
}