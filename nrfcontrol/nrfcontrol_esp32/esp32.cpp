#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Pin assignments for ESP32
#define CE_PIN 4
#define CSN_PIN 5

// SPI pins are defined by default:
// SCLK = 18
// MOSI = 23
// MISO = 19

// Create RF24 object
RF24 radio(CE_PIN, CSN_PIN);

// Communication address (must match transmitter)
const byte address[6] = "00001";

// Data structure to receive (must match transmitter)
struct TelemetryData {
  float altitude;
  float velocity;
  float acceleration;
  int battery_voltage;
  bool status;
};

TelemetryData receivedData;
unsigned long lastReceived = 0;

void setup() {
  Serial.begin(115200);
  
  // Add delay for radio initialization
  delay(100);
  
  // Initialize radio
  if (!radio.begin()) {
    Serial.println("Radio hardware not responding!");
    while(1);
  }
  
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);     // Match transmitter
  radio.setDataRate(RF24_250KBPS);   // Match transmitter
  radio.setChannel(76);              // Match transmitter
  radio.enableAckPayload();          // Enable acknowledgment
  radio.startListening();            // Set as receiver
  
  // Print radio details
  radio.printDetails();
  
  Serial.println("NRF24L01 Receiver Ready");
  Serial.println("Waiting for data...");
}

void loop() {
  // Check if data is available
  if (radio.available()) {
    // Read the data
    radio.read(&receivedData, sizeof(receivedData));
    lastReceived = millis();
    
    // Print received data
    Serial.println("=== Data Received ===");
    Serial.print("Altitude: "); 
    Serial.print(receivedData.altitude);
    Serial.println(" m");
    
    Serial.print("Velocity: "); 
    Serial.print(receivedData.velocity);
    Serial.println(" m/s");
    
    Serial.print("Acceleration: "); 
    Serial.print(receivedData.acceleration);
    Serial.println(" m/s²");
    
    Serial.print("Battery: "); 
    Serial.print(receivedData.battery_voltage);
    Serial.println("%");
    
    Serial.print("Status: "); 
    Serial.println(receivedData.status ? "OK" : "ERROR");
    
    Serial.println("====================");
    Serial.println();
  }
  
  // Check for timeout
  if (millis() - lastReceived > 5000 && lastReceived != 0) {
    Serial.println("No data received for 5 seconds - Check transmitter");
    lastReceived = millis();
  }
  
  delay(50);
}