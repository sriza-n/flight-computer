#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>

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
uint16_t servo1Angle = 0;
uint16_t servo2Angle = 0;

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
  Servo servo;
  uint8_t pin;
  int angle = 0;
  bool opened = false;
  const uint8_t step = 1;
  const uint8_t delayMs = 25;

public:
  Valve(uint8_t p) : pin(p) {}
  void begin()
  {
    servo.attach(pin);
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
        servo.write(pin, a);
        vTaskDelay(delayMs / portTICK_PERIOD_MS);
      }
    }
    else if (angle > targetAngle)
    {
      for (int a = angle; a >= targetAngle; a -= step)
      {
        servo.write(pin, a);
        vTaskDelay(delayMs / portTICK_PERIOD_MS);
      }
    }

    angle = targetAngle;
    opened = (targetAngle > 0);
    servo.write(pin, angle);
  }
  
  void close()
  {
    if (!opened)
      return;
    for (int a = angle; a >= 0; a -= step)
    {
      servo.write(pin, a);
      vTaskDelay(delayMs / portTICK_PERIOD_MS);
    }
    angle = 0;
    opened = false;
    servo.write(pin, angle);
  }
  bool isOpen() const { return opened; }
  int getCurrentAngle() const { return angle; }
};

Valve valve1(VALVE1_SERVO_PIN), valve2(VALVE2_SERVO_PIN);

// // --- Utility Functions ---
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
  radio.setDataRate(RF24_2MBPS);
  radio.setChannel(108);

  // Enable auto-ack
  radio.enableAckPayload();
  radio.setAutoAck(true);

  // Enable dynamic payloads
  radio.enableDynamicPayloads();

  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);

  pinMode(IGNITION_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TEENSY_POWER_PIN, OUTPUT);

  digitalWrite(IGNITION_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(TEENSY_POWER_PIN, LOW);

  valve1.begin();
  // valve2.begin();

  Wire.begin();
}

// New function to handle ping packets and send acknowledgment
void handlePingPacket()
{
  // Switch to transmit mode and send acknowledgment
  radio.stopListening();
  uint8_t ackData = 0xAA; // Acknowledgment byte
  bool success = radio.write(&ackData, 1);

  if (success)
  {
    connectionState = true;
    Serial.println("ACK sent successfully");
  }
  else
  {
    connectionState = false;
    Serial.println("Failed to send ACK");
  }

  radio.startListening(); // Return to listening mode
}

// Function to receive and process binary transmitted pin states
void receiveBinaryPinStates()
{
  radio.startListening(); // Switch to receive mode

  if (radio.available())
  {
    uint8_t buffer[16];
    uint8_t len = radio.getDynamicPayloadSize();
    radio.read(&buffer, len);

    // Check if this is a ping packet (1 byte with value 0xFF)
    if (len == 1 && buffer[0] == 0xFF)
    {
      handlePingPacket();
      return;
    }

    int index = 0;

    // Extract digital pin states from first byte
    uint8_t digitalFlags = buffer[index++];
    readyState = (digitalFlags & 0x01) != 0;    // Bit 0
    ignitionState = (digitalFlags & 0x02) != 0; // Bit 1
    servo1State = (digitalFlags & 0x04) != 0;   // Bit 2
    servo2State = (digitalFlags & 0x08) != 0;   // Bit 3
    haltState = (digitalFlags & 0x10) != 0;     // Bit 4

    // Extract servo angles (2 bytes each)
    memcpy(&servo1Angle, &buffer[index], 2);
    index += 2;
    memcpy(&servo2Angle, &buffer[index], 2);
    index += 2;

    // Extract timestamp (4 bytes)
    // uint32_t timestamp;
    // memcpy(&timestamp, &buffer[index], 4);
    // index += 4;

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
}

// Function to process received commands
void processCommands()
{
  if (connectionState)
  {
    digitalWrite(TEENSY_POWER_PIN, HIGH);

    if (readyState)
    {
      Buzz(2, 50, 500);
    }

    if (ignitionState && readyState && ignitionStateMachine == IgnitionState::IDLE)
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
      valve1.open(servo1Angle);
      valve2.open(servo2Angle);
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
}

void processValves()
{
  manualValveControl = false;
  if (connectionState)
  {
    if (servo1State)
    {
      valve1.open(servo1Angle);
      Buzz(1, 50, 50);
      manualValveControl = true;
    }
    else
    {
      valve1.close();
      Buzz(1, 50, 50);
      manualValveControl = true;
    }
    if (servo2State)
    {
      valve2.open();
      Buzz(1, 50, 50);
      manualValveControl = true;
    }
    else
    {
      valve2.close();
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
}

void transmitI2CData()
{

  // --- I2C Data Packet ---
  uint8_t dataPacket[5] = {
      static_cast<uint8_t>(valve1.isOpen()),
      static_cast<uint8_t>(valve2.isOpen()),
      static_cast<uint8_t>(readyState),
      static_cast<uint8_t>(outputState2),
      static_cast<uint8_t>(haltState),
  };

  Wire.beginTransmission(TEENSY_I2C_ADDRESS);
  Wire.write(dataPacket, 5);
  Wire.endTransmission();
}

// Main loop
void loop()
{
  receiveBinaryPinStates();
  processCommands();
  processValves();
  // I2c communication
  transmitI2CData();
  delay(100);
}
