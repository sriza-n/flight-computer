#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Pin assignments for Arduino Nano
#define CE_PIN   9
#define CSN_PIN  10
// MOSI - Pin 11 (default SPI)
// MISO - Pin 12 (default SPI) 
// SCK  - Pin 13 (default SPI)

// Create RF24 object
RF24 radio(CE_PIN, CSN_PIN);

// Communication address
const byte address[6] = "00001";

// Data structure to send
struct TelemetryData {
  float altitude;
  float velocity;
  float acceleration;
  int battery_voltage;
  bool status;
};

TelemetryData data;

void setup() {
  Serial.begin(9600);
  
  // Add delay for radio initialization
  delay(100);
  
  // Initialize radio
  if (!radio.begin()) {
    Serial.println("Radio hardware not responding!");
    while(1);
  }
  
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);     // Try LOW instead of MIN
  radio.setDataRate(RF24_250KBPS);   // Lower data rate for better range
  radio.setChannel(76);              // Set specific channel
  radio.enableAckPayload();          // Enable acknowledgment
  radio.setRetries(15, 15);          // Set retries (delay, count)
  radio.stopListening();             // Set as transmitter
  
  // Print radio details
  radio.printDetails();
  
  Serial.println("NRF24L01 Transmitter Ready");
  
  // Initialize data
  data.altitude = 0.0;
  data.velocity = 0.0;
  data.acceleration = 0.0;
  data.battery_voltage = 100;
  data.status = true;
}

void loop() {
  // Update data (replace with actual sensor readings)
  data.altitude += 0.1;
  data.velocity = random(-50, 50) / 10.0;
  data.acceleration = random(-100, 100) / 10.0;
  data.battery_voltage = random(80, 100);
  
  // Send data
  bool result = radio.write(&data, sizeof(data));
  
  if (result) {
    Serial.println("Data sent successfully");
    Serial.print("Altitude: "); Serial.println(data.altitude);
    Serial.print("Velocity: "); Serial.println(data.velocity);
    Serial.print("Acceleration: "); Serial.println(data.acceleration);
    Serial.print("Battery: "); Serial.println(data.battery_voltage);
    Serial.println("---");
  } else {
    Serial.println("Data send failed - Check connections and receiver");
  }
  
  delay(1000);  // Send data every second
}