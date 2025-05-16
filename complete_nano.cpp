#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
// Define pin numbers for inputs
const int masterInputPin = A0; // Master control input
const int inputPin1 = 3;
const int inputPin2 = 4;
const int inputPin3 = 5;
const int inputPin4 = 6;

// Define custom voltage threshold for master input (assuming 5V reference)
const int masterThreshold = 400;

// Master input debounce variables
const int masterDebounceCount = 10; // Number of consecutive readings needed to change state
int masterReadings = 0;             // Counter for consistent readings
int stableMasterState = LOW;        // Current stable state of master input

const int ignitionpin = 8;
// const int outputPin4 = 11;

int buzzerPin = 2;

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

// Second valve using servo on pin 8

// Define the Teensy's I2C address
#define TEENSY_I2C_ADDRESS 0x42

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

void setup()
{
  Serial.begin(9600);
  // Initialize input pins
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

  // Initialize all outputs to LOW
  digitalWrite(ignitionpin, LOW);
  digitalWrite(buzzerPin, LOW);
  // Optional: Start serial for debugging

  // Initialize valves
  valve1.begin();
  valve2.begin();
  Serial.println("Valves initialized.");
}

void loop()
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

  // Read current input states
  int currentInput1 = digitalRead(inputPin1);
  int currentInput2 = digitalRead(inputPin2);
  int currentInput3 = digitalRead(inputPin3);
  int currentInput4 = digitalRead(inputPin4);



  // If master input is HIGH, check for input transitions to toggle outputs
  if (stableMasterState == HIGH)
  {
    // Check for transitions
    if (currentInput1 == HIGH && prevInput1 == LOW)
    {
      outputState1 = !outputState1; // Toggle output state
      Serial.print("Output 1 toggled to: ");
      Serial.println(outputState1 ? "HIGH" : "LOW");
      // Buzz(2, 100, 100);  // 2 buzzes, 100ms on, 100ms off
    }

    // Only allow input2 to work if outputState1 is HIGH (input1 has been activated)
    if (currentInput2 == LOW && outputState1 == HIGH)
    {
      outputState2 = !outputState2;
      Serial.print("Output 2 toggled to: ");
      Serial.println(outputState2 ? "HIGH" : "LOW");
      Buzz(3,100,900); // 3 buzzes, 100ms on, 900ms off
      //ignitionpin enable
      digitalWrite(ignitionpin, outputState2);
      delay(1000); // Wait for 1 second
      valve1.open();  // Open the first valve
      valve2.open();  // Open the second valve
      delay(15000); // Wait for 15 second
      valve1.close();  // Close the first valve
      valve2.close();  // Close the second valve
    }

    if (currentInput3 == HIGH && prevInput3 == LOW)
    {
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
  }
  else if (stableMasterState == LOW)
  {
    // If master input is LOW, all outputs are set to LOW
    outputState1 = LOW;
    outputState2 = LOW;
    outputState3 = LOW;
    outputState4 = LOW;
    valve1.close();                 // Close the first valve
    valve2.close();                 // Close the second valve
    digitalWrite(ignitionpin, LOW); // Turn off ignition
  }

  // Store current input states for next iteration
  prevInput1 = currentInput1;
  prevInput2 = currentInput2;
  prevInput3 = currentInput3;
  prevInput4 = currentInput4;

  // Create data packet with 4 bytes to transmit output states
  // byte dataPacket[4];
  // dataPacket[0] = outputState3 ? 1 : 0;
  // dataPacket[1] = outputState4 ? 1 : 0;
  // dataPacket[2] = outputState1 ? 1 : 0;
  // dataPacket[3] = outputState2 ? 1 : 0;  // Add the new value

  //   // Send data to Teensy
  // Wire.beginTransmission(TEENSY_I2C_ADDRESS);
  // Wire.write(dataPacket, 4);  // Now sending 4 bytes
  // byte error = Wire.endTransmission();

  // if (error == 0) {
  //   Serial.println("Data sent successfully");
  // } else {
  //   Serial.print("Error sending data: ");
  //   Serial.println(error);
  // }
  // Small delay to prevent excessive looping
  delay(20);
}