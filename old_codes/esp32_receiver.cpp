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
  rf95.setSignalBandwidth(125000);
  rf95.setCodingRate4(5);
  rf95.setSpreadingFactor(7);

  // For significantly better range
  // rf95.setSpreadingFactor(8);    // Increase from SF7 to SF10
  // rf95.setCodingRate4(5);         // Increase from 4/5 to 4/8
  // rf95.setSignalBandwidth(62500); // Decrease from 125kHz to 62.5kHz

  // Set transmit power (this replaces the erroneous setLnaMode line)
  rf95.setTxPower(23, false);

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
      Serial.print("Received packet with RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);

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

  Serial.print("Raw data: ");
  for (int i = 0; i < len && i < 20; i++)
  { // Print first 20 bytes
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println("...");

  if (len < 68)
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

  // Extract time
  uint8_t min = buffer[index++];
  uint8_t sec = buffer[index++];
  String timeStr = String(min) + ":" + String(sec);

  // Extract nano values
  uint8_t nanoFlags = buffer[index++];
  bool nano1 = nanoFlags & 0x01;
  bool nano2 = nanoFlags & 0x02;
  bool nano3 = nanoFlags & 0x04;
  bool nano4 = nanoFlags & 0x08;

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

  // Extract hardcoded values
  float hc1, hc2, hc3;
  memcpy(&hc1, &buffer[index], 4);
  index += 4;
  memcpy(&hc2, &buffer[index], 4);
  index += 4;
  memcpy(&hc3, &buffer[index], 4);
  index += 4;

  // Extract temperature
  float temperature;
  memcpy(&temperature, &buffer[index], 4);
  index += 4;

  // Print decoded data in CSV format (same as original)
  Serial.print(sn);
  Serial.print(",");
  Serial.print(timeStr);
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
  Serial.print(analog1, 4);
  Serial.print(",");
  Serial.print(analog2, 4);
  Serial.print(",");
  Serial.print(xPos, 4);
  Serial.print(",");
  Serial.print(yPos, 4);
  Serial.print(",");
  Serial.print(altitude, 4);
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
  Serial.print(hc1, 6);
  Serial.print(",");
  Serial.print(hc2, 6);
  Serial.print(",");
  Serial.print(hc3, 2);
  Serial.print(",");
  Serial.println(temperature, 4);

  // Optional: Output in JSON format for easier parsing by other systems
  /*
  Serial.print("{\"sn\":"); Serial.print(sn);
  Serial.print(",\"time\":\""); Serial.print(timeStr); Serial.print("\"");
  Serial.print(",\"nano\":[");
  Serial.print(nano1 ? "true" : "false"); Serial.print(",");
  Serial.print(nano2 ? "true" : "false"); Serial.print(",");
  Serial.print(nano3 ? "true" : "false"); Serial.print(",");
  Serial.print(nano4 ? "true" : "false"); Serial.print("]");
  // Continue with other fields
  Serial.println("}");
  */
}