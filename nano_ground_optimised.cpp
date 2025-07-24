#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
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
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);

  radio.enableAckPayload();
  radio.setAutoAck(true);

  radio.setRetries(15, 15);

  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);

  pinMode(Ready_PIN, INPUT);
  pinMode(IGNITION_PIN, INPUT);
  pinMode(SERVO1_DigitalPin, INPUT);
  pinMode(SERVO2_DigitalPin, INPUT);
  pinMode(Halt_PIN, INPUT);
}

void transmitPinStates()
{
  uint8_t buffer[16];
  int index = 0;

  bool currentReadyPinState = digitalRead(Ready_PIN);
  if (currentReadyPinState == HIGH && latchReadyState == LOW)
  {
    readyState = !readyState;
  }
  latchReadyState = currentReadyPinState;

  bool ignitionRaw = digitalRead(IGNITION_PIN);
  servo1State = digitalRead(SERVO1_DigitalPin);
  servo2State = digitalRead(SERVO2_DigitalPin);
  haltState = digitalRead(Halt_PIN);
  TestMode = digitalRead(TestMode_PIN);
  ConfigMode = digitalRead(ConfigMode_PIN);

  ignitionState = readyState && ignitionRaw;

  uint8_t digitalFlags = 0;
  digitalFlags |= (readyState << 0);
  digitalFlags |= (ignitionState << 1);
  digitalFlags |= (servo1State << 2);
  digitalFlags |= (servo2State << 3);
  digitalFlags |= (haltState << 4);
  digitalFlags |= (TestMode << 5);
  digitalFlags |= (ConfigMode << 6);
  buffer[index++] = digitalFlags;

  uint16_t servo1_16 = (uint16_t)servo1Angle;
  uint16_t servo2_16 = (uint16_t)servo2Angle;
  memcpy(&buffer[index], &servo1_16, 2);
  index += 2;
  memcpy(&buffer[index], &servo2_16, 2);
  index += 2;

  radio.stopListening();
  bool success = radio.write(buffer, index);

  if (success)
  {
    connectionState = true;
  }
  else
  {
    connectionState = false;
  }

  radio.startListening();
}

void readServoAngles()
{
  int servo1Raw = analogRead(SERVO1_AnalogPin);
  int servo2Raw = analogRead(SERVO2_AnalogPin);

  servo1Angle = map(servo1Raw, 0, 1010, 0, 180);
  servo2Angle = map(servo2Raw, 0, 1010, 0, 180);

  servo1Angle = constrain(servo1Angle, 0, 180);
  servo2Angle = constrain(servo2Angle, 0, 180);
}

void loop()
{
  readServoAngles();
  transmitPinStates();

  uint8_t dataPacket[5] = {
      static_cast<uint8_t>(servo1Angle),
      static_cast<uint8_t>(servo2Angle),
      static_cast<uint8_t>(ConfigMode),
      static_cast<uint8_t>(TestMode),
      static_cast<uint8_t>(connectionState),
  };

  Wire.beginTransmission(esp32_I2C_ADDRESS);
  Wire.write(dataPacket, 5);
  Wire.endTransmission();
  delay(100);
}
