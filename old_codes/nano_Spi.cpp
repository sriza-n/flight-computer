#include <Wire.h>
#include <Arduino.h>

// Define the Teensy's I2C address
#define TEENSY_I2C_ADDRESS 0x42

// Define digital pin inputs
const int DIGITAL_PIN1 = 2;  // Use digital pin 2
const int DIGITAL_PIN2 = 3;  // Use digital pin 3
const int DIGITAL_PIN3 = 4;  // Use digital pin 4
const int DIGITAL_PIN4 = 5;  // Added fourth digital pin

// Boolean data to send
bool sensorValue1 = false;
bool sensorValue2 = false;
bool sensorValue3 = false;
bool sensorValue4 = false;  // Added fourth sensor value

void setup() {
  Serial.begin(9600);
  Wire.begin(); // Initialize I2C as master
  
  // Configure digital pins as inputs with pull-up resistors
  pinMode(DIGITAL_PIN1, INPUT_PULLUP);
  pinMode(DIGITAL_PIN2, INPUT_PULLUP);
  pinMode(DIGITAL_PIN3, INPUT_PULLUP);
  pinMode(DIGITAL_PIN4, INPUT_PULLUP);  // Configure the new pin
  
  Serial.println("Arduino Nano I2C Master initialized");
}

void loop() {
  // Read digital pins (LOW = true, HIGH = false with pull-up resistors)
  sensorValue1 = (digitalRead(DIGITAL_PIN1) == LOW);
  sensorValue2 = (digitalRead(DIGITAL_PIN2) == LOW);
  sensorValue3 = (digitalRead(DIGITAL_PIN3) == LOW);
  sensorValue4 = (digitalRead(DIGITAL_PIN4) == LOW);  // Read the new pin
  
  // Create data packet with 4 bytes
  byte dataPacket[4];
  dataPacket[0] = sensorValue1 ? 1 : 0;
  dataPacket[1] = sensorValue2 ? 1 : 0;
  dataPacket[2] = sensorValue3 ? 1 : 0;
  dataPacket[3] = sensorValue4 ? 1 : 0;  // Add the new value
  
  // Print values for debugging
  Serial.print("Sensor Values: ");
  Serial.print(sensorValue1 ? "ON" : "OFF");
  Serial.print(", ");
  Serial.print(sensorValue2 ? "ON" : "OFF");
  Serial.print(", ");
  Serial.print(sensorValue3 ? "ON" : "OFF");
  Serial.print(", ");
  Serial.println(sensorValue4 ? "ON" : "OFF");  // Print the new value
  
  // Send data to Teensy
  Wire.beginTransmission(TEENSY_I2C_ADDRESS);
  Wire.write(dataPacket, 4);  // Now sending 4 bytes
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.println("Data sent successfully");
  } else {
    Serial.print("Error sending data: ");
    Serial.println(error);
  }
  
  delay(100); // Send data every 100ms
}