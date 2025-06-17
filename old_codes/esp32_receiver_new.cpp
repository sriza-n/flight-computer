#include <SPI.h>
#include <RH_RF95.h>

// Pin definitions for ESP32 and LoRa RA-02
#define LORA_CS 5   // NSS pin
#define LORA_RST 14 // RESET pin
#define LORA_INT 2  // DIO0 (Interrupt) pin

// SPI pins for ESP32 (default)
// SCK - 18
// MISO - 19
// MOSI - 23

// RF frequency - must match transmitter
#define RF95_FREQ 467.5

// Initialize LoRa instance
RH_RF95 rf95(LORA_CS, LORA_INT);

void decodeReceivedData(uint8_t *buffer, uint8_t len);

// Output pin definitions
#define NANO1_PIN 25
#define NANO2_PIN 26
#define NANO3_PIN 27
#define NANO4_PIN 32
#define REMOTE_PIN 33
#define VALVE_PIN 13

// store rssi
int rssi = 0;
float snr = 0;
int frequencyError = 0;
// unsigned long lastPacketTime = 0;
// unsigned long packetCount = 0;

void setup()
{
  Serial.begin(115200);
  // while (!Serial)
  //   delay(10);
  Serial.println("ESP32 LoRa Receiver");

  // Configure reset pin
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, HIGH);

  // Reset LoRa module
  digitalWrite(LORA_RST, LOW);
  delay(10);
  digitalWrite(LORA_RST, HIGH);
  delay(10);

  // Initialize LoRa
  if (!rf95.init())
  {
    Serial.println("LoRa initialization failed!");
    while (1)
      ;
  }
  Serial.println("LoRa initialized successfully");

  // Set frequency
  if (!rf95.setFrequency(RF95_FREQ))
  {
    Serial.println("Setting frequency failed!");
    while (1)
      ;
  }
  Serial.print("Frequency set to: ");
  Serial.println(RF95_FREQ);

  // // Configure LoRa parameters (should match transmitter)
  // rf95.setSignalBandwidth(125000);
  // rf95.setCodingRate4(5);
  // rf95.setSpreadingFactor(7);

  rf95.setSignalBandwidth(125000); // Reduce from 125000 to 62500 Hz
  rf95.setCodingRate4(5);         // Increase from 5 to 8 (4/8 coding rate)
  rf95.setSpreadingFactor(9);    // Increase from 7 to 12

  // For significantly better range
  // rf95.setSpreadingFactor(8);    // Increase from SF7 to SF10
  // rf95.setCodingRate4(5);         // Increase from 4/5 to 4/8
  // rf95.setSignalBandwidth(62500); // Decrease from 125kHz to 62.5kHz
  // Optional: Set LNA settings for better reception sensitivity
  // The first parameter sets LNA boost (0 = default, 1 = improved sensitivity)
  // The second parameter sets LNA gain (0-6, higher = better sensitivity but more current consumption)
  rf95.spiWrite(RH_RF95_REG_0C_LNA, 0x23); // LNA boost on, max gain

  // Set transmit power (this replaces the erroneous setLnaMode line)
  rf95.setModeRx();

  // rf95.setTxPower(23, false);

  // Configure output pins for nano flags and valve state
  pinMode(NANO1_PIN, OUTPUT);
  pinMode(NANO2_PIN, OUTPUT);
  pinMode(NANO3_PIN, OUTPUT);
  pinMode(NANO4_PIN, OUTPUT);
  pinMode(REMOTE_PIN, OUTPUT);
  pinMode(VALVE_PIN, OUTPUT);

  // Initialize all outputs to LOW
  digitalWrite(NANO1_PIN, LOW);
  digitalWrite(NANO2_PIN, LOW);
  digitalWrite(NANO3_PIN, LOW);
  digitalWrite(NANO4_PIN, LOW);
  digitalWrite(REMOTE_PIN, LOW);
  digitalWrite(VALVE_PIN, LOW);

  Serial.println("Receiver ready!");
}

void loop()
{
  if (rf95.available())
  {
    // Buffer for received data
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len))
    {
      // Print RSSI for signal quality monitoring
      // Serial.print("Received packet with RSSI: ");
      rssi = rf95.lastRssi();
      snr = rf95.lastSNR();
      frequencyError = rf95.frequencyError();

      // Serial.println(rssi, DEC);

      // Decode the binary data
      decodeReceivedData(buf, len);
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
}

void decodeReceivedData(uint8_t *buffer, uint8_t len)
{
  // Serial.print("Raw data: ");
  // for (int i = 0; i < len && i < 20; i++)
  // { // Print first 20 bytes
  //   Serial.print(buffer[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println("...");

  if (len < 66)
  {
    Serial.println("Received incomplete data packet");
    Serial.print("Got only ");
    Serial.print(len);
    Serial.println(" bytes");
    return;
  }

  int index = 0;

  // Extract record serial number
  uint32_t sn;
  memcpy(&sn, &buffer[index], 4);
  index += 4;

  // Extract time as float (MM.SS format)
  float timeValue;
  memcpy(&timeValue, &buffer[index], 4);
  index += 4;

  // Convert to readable time string
  int minutes = (int)timeValue;
  int seconds = (int)((timeValue - minutes) * 100 + 0.5);
  String timeStr = String(minutes) + ":" + (seconds < 10 ? "0" : "") + String(seconds);

  // Extract nano values
  uint8_t nanoFlags = buffer[index++];
  bool nano1 = nanoFlags & 0x01;
  bool nano2 = nanoFlags & 0x02;
  bool nano3 = nanoFlags & 0x04;
  bool nano4 = nanoFlags & 0x08;
  bool remotestate = nanoFlags & 0x10;

  // Extract valve state
  bool valveState = buffer[index++];

  // Extract analog values
  float analog1, analog2;
  memcpy(&analog1, &buffer[index], 4);
  index += 4;
  memcpy(&analog2, &buffer[index], 4);
  index += 4;

  // Extract position values
  float xPos, yPos;
  memcpy(&xPos, &buffer[index], 4);
  index += 4;
  memcpy(&yPos, &buffer[index], 4);
  index += 4;

  // Extract altitude
  float altitude;
  memcpy(&altitude, &buffer[index], 4);
  index += 4;

  // Extract euler angles
  float eulerX, eulerY, eulerZ;
  memcpy(&eulerX, &buffer[index], 4);
  index += 4;
  memcpy(&eulerY, &buffer[index], 4);
  index += 4;
  memcpy(&eulerZ, &buffer[index], 4);
  index += 4;

  // Extract linear acceleration
  float accelX, accelY, accelZ;
  memcpy(&accelX, &buffer[index], 4);
  index += 4;
  memcpy(&accelY, &buffer[index], 4);
  index += 4;
  memcpy(&accelZ, &buffer[index], 4);
  index += 4;

  // Extract GPS coordinates
  float gpsLat, gpsLng;
  memcpy(&gpsLat, &buffer[index], 4);
  index += 4;
  memcpy(&gpsLng, &buffer[index], 4);
  index += 4;

  // Extract temperature
  float temperature;
  memcpy(&temperature, &buffer[index], 4);
  index += 4;

  // Set output pins according to received flag values
  digitalWrite(NANO1_PIN, nano1 ? HIGH : LOW);
  digitalWrite(NANO2_PIN, nano2 ? HIGH : LOW);
  digitalWrite(NANO3_PIN, nano3 ? HIGH : LOW);
  digitalWrite(NANO4_PIN, nano4 ? HIGH : LOW);
  digitalWrite(REMOTE_PIN, remotestate ? HIGH : LOW);
  digitalWrite(VALVE_PIN, valveState ? HIGH : LOW);

  // Print decoded data in CSV format
  Serial.print(sn);
  Serial.print(",");
  Serial.print(timeStr);
  Serial.print(",");
  Serial.print(remotestate ? "1" : "0");
  Serial.print(",");
  Serial.print(nano1 ? "1" : "0");
  Serial.print(",");
  Serial.print(nano2 ? "1" : "0");
  Serial.print(",");
  Serial.print(nano3 ? "1" : "0");
  Serial.print(",");
  Serial.print(nano4 ? "1" : "0");
  Serial.print(",");
  Serial.print(valveState ? "1" : "0");
  Serial.print(",");
  Serial.print(analog1, 2);
  Serial.print(",");
  Serial.print(analog2, 2);
  Serial.print(",");
  Serial.print(xPos, 2);
  Serial.print(",");
  Serial.print(yPos, 2);
  Serial.print(",");
  Serial.print(altitude, 2);
  Serial.print(",");
  Serial.print(eulerX, 4);
  Serial.print(",");
  Serial.print(eulerY, 4);
  Serial.print(",");
  Serial.print(eulerZ, 4);
  Serial.print(",");
  Serial.print(accelX, 4);
  Serial.print(",");
  Serial.print(accelY, 4);
  Serial.print(",");
  Serial.print(accelZ, 4);
  Serial.print(",");
  Serial.print(gpsLat, 6);
  Serial.print(",");
  Serial.print(gpsLng, 6);
  Serial.print(",");
  Serial.print(temperature, 2);
  Serial.print(",");
  Serial.print(rssi);
  Serial.print(",");
  Serial.println(snr);
  Serial.print("RSSI: ");
  Serial.print(rssi);
  Serial.print(", SNR: ");
  Serial.print(snr);
  Serial.print(", FreqErr: ");
  Serial.println(frequencyError);

  // Optional: JSON output can be updated similarly if needed
}