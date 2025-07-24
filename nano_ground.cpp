#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
// for i2c comm
#include <Wire.h>

// Nrf configuration
RF24 radio(9, 10); // CE, CSN
const byte addresses[][10] = {"00001", "00002"};
const int payloadSize = 32; // Size of payload buffer

// --- Pin Definitions ---
constexpr uint8_t Ready_PIN = 2;
constexpr uint8_t IGNITION_PIN = 3;
constexpr uint8_t SERVO1_DigitalPin = 4;
constexpr uint8_t SERVO2_DigitalPin = 5;
constexpr uint8_t Halt_PIN = 6;
constexpr uint8_t TestMode_PIN = 7;
constexpr uint8_t ConfigMode_PIN = 8;
constexpr uint8_t SERVO1_AnalogPin = 14; // A0
constexpr uint8_t SERVO2_AnalogPin = 15; // A1

// servo angles
int servo1Angle = 0;
int servo2Angle = 0;

volatile bool readyState = false;
volatile bool ignitionState = false;
volatile bool servo1State = false;
volatile bool servo2State = false;
volatile bool haltState = false;
volatile bool connectionState = false;
volatile bool TestMode = false;
volatile bool ConfigMode = false;

volatile bool latchReadyState = false; // Latch for readyState

// for i2c comm with esp32
constexpr uint8_t esp32_I2C_ADDRESS = 0x42;

void setup()
{
  Serial.begin(9600);

  Wire.begin();

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

  radio.setRetries(15, 15); // Set retries (delay, count)

  // Enable dynamic payloads
  // radio.enableDynamicPayloads();

  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);

  pinMode(Ready_PIN, INPUT);
  pinMode(IGNITION_PIN, INPUT);
  pinMode(SERVO1_DigitalPin, INPUT);
  pinMode(SERVO2_DigitalPin, INPUT);
  pinMode(Halt_PIN, INPUT);
}

// Function to transmit pin states using binary encoding
void transmitPinStates()
{
  uint8_t buffer[16]; // Buffer for binary data
  int index = 0;

  bool currentReadyPinState = digitalRead(Ready_PIN);
  if (currentReadyPinState == HIGH && lastReadyPinState == LOW)
  {
    // toggle readyState
    readyState = !readyState;
  }
  latchReadyState = currentReadyPinState;
  // Read the current state of all input pins
  // readyState = digitalRead(Ready_PIN);
  bool ignitionRaw = digitalRead(IGNITION_PIN);
  servo1State = digitalRead(SERVO1_DigitalPin);
  servo2State = digitalRead(SERVO2_DigitalPin);
  haltState = digitalRead(Halt_PIN);
  TestMode = digitalRead(TestMode_PIN);
  ConfigMode = digitalRead(ConfigMode_PIN);

  // Set ignitionState high only if readyState is also high
  ignitionState = readyState && ignitionRaw;

  // Pack digital pin states into a single byte
  uint8_t digitalFlags = 0;
  digitalFlags |= (readyState << 0);    // Bit 0: Ready pin
  digitalFlags |= (ignitionState << 1); // Bit 1: Ignition pin
  digitalFlags |= (servo1State << 2);   // Bit 2: Servo1 pin
  digitalFlags |= (servo2State << 3);   // Bit 3: Servo2 pin
  digitalFlags |= (haltState << 4);     // Bit 4: Halt pin
  digitalFlags |= (TestMode << 5);      // Bit 5: Test Mode pin
  digitalFlags |= (ConfigMode << 6);    // Bit 6: Config Mode pin
  buffer[index++] = digitalFlags;

  // Pack servo angles as 16-bit integers (2 bytes each)
  uint16_t servo1_16 = (uint16_t)servo1Angle;
  uint16_t servo2_16 = (uint16_t)servo2Angle;
  memcpy(&buffer[index], &servo1_16, 2);
  index += 2;
  memcpy(&buffer[index], &servo2_16, 2);
  index += 2;


  // Transmit the binary packet
  radio.stopListening(); // Switch to transmit mode
  bool success = radio.write(buffer, index);

  if (success)
  {
    connectionState = true;
  }
  else
  {
    connectionState = false;
  }

  radio.startListening(); // Return to listening mode
}

void readServoAngles()
{
  // Read 10-bit analog values (0-1023)
  int servo1Raw = analogRead(SERVO1_AnalogPin);
  int servo2Raw = analogRead(SERVO2_AnalogPin);

  // Convert to servo angles (0-180 degrees)
  servo1Angle = map(servo1Raw, 0, 1010, 0, 180);
  servo2Angle = map(servo2Raw, 0, 1010, 0, 180);

  // Constrain values to valid servo range (safety check)
  servo1Angle = constrain(servo1Angle, 0, 180);
  servo2Angle = constrain(servo2Angle, 0, 180);

  // Optional: Print values for debugging
  // Serial.print(readyState);
  // Serial.print(",");
  // Serial.print(ignitionState);
  // Serial.print(",");
  // Serial.print(servo1State);
  // Serial.print(",");
  // Serial.print(servo2State);
  // Serial.print(",");
  // Serial.print(haltState);
  // Serial.print(",");
  // Serial.print(servo1Angle);
  // Serial.print(",");
  // Serial.print(servo2Angle);
  // Serial.print(',');
  // Serial.println(connectionState);
}

// Main loop
void loop()
{ // Check connection status
  // checkConnectionState();
  // read servo angle
  readServoAngles();
  transmitPinStates();
  // --- I2C Data Packet ---
  uint8_t dataPacket[5] = {
      static_cast<uint8_t>(servo1Angle),
      static_cast<uint8_t>(servo2Angle),
      static_cast<uint8_t>(ConfigMode),
      static_cast<uint8_t>(TestMode),
      static_cast<uint8_t>(connectionState),
  };

  // serial print the data packet
  // Serial.print("Data Packet: ");
  // for (int i = 0; i < 5; i++)
  // {
  //   Serial.println(dataPacket[i]);
  // }

  Wire.beginTransmission(esp32_I2C_ADDRESS);
  Wire.write(dataPacket, 5);
  Wire.endTransmission();
  delay(100);
}