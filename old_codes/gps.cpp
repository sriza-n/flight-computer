#include <Arduino.h>
#include <TinyGPS++.h>

// Create TinyGPS++ objects for both GPS modules
TinyGPSPlus gps1; // First GPS (NEO-8M)
TinyGPSPlus gps2; // Second GPS (NEO-9M)

// Connection for first GPS is on Serial1 (pins 0/1 on Teensy 4.1)
#define GPS1_SERIAL Serial1
// Connection for second GPS is on Serial2 (pins 7/8 on Teensy 4.1)
#define GPS2_SERIAL Serial2

void displayInfo(TinyGPSPlus &gps, const char* gpsName) {
  Serial.print("-------- ");
  Serial.print(gpsName);
  Serial.println(" DATA --------");
  
  if (gps.location.isValid()) {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Altitude: ");
    if (gps.altitude.isValid())
      Serial.println(gps.altitude.meters());
    else
      Serial.println("INVALID");
  } else {
    Serial.println("Location: INVALID");
  }
  
  if (gps.date.isValid()) {
    Serial.print("Date: ");
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.println(gps.date.year());
  } else {
    Serial.println("Date: INVALID");
  }
  
  if (gps.time.isValid()) {
    Serial.print("Time: ");
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(":");
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(":");
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.println(gps.time.second());
  } else {
    Serial.println("Time: INVALID");
  }
  
  if (gps.speed.isValid()) {
    Serial.print("Speed (knots): ");
    Serial.println(gps.speed.knots());
  }

  Serial.print("Satellites: ");
  if (gps.satellites.isValid())
    Serial.println(gps.satellites.value());
  else
    Serial.println("INVALID");
    
  Serial.print("HDOP: ");
  if (gps.hdop.isValid())
    Serial.println(gps.hdop.value());
  else
    Serial.println("INVALID");
    
  Serial.println();
}

void setup() {
  // Initialize serial communication with computer
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  // Initialize GPS serial connections
  GPS1_SERIAL.begin(9600); // Default baud for NEO-8M
  GPS2_SERIAL.begin(9600); // Default baud for NEO-9M
  
  Serial.println(F("Dual GPS Example"));
  Serial.println(F("NEO-8M using pins 0 (RX) and 1 (TX)"));
  Serial.println(F("NEO-9M using pins 7 (RX) and 8 (TX)"));
}

void loop() {
  bool newData1 = false;
  bool newData2 = false;
  
  // Read data from first GPS
  while (GPS1_SERIAL.available() > 0) {
    if (gps1.encode(GPS1_SERIAL.read())) {
      newData1 = true;
    }
  }

  // Read data from second GPS
  while (GPS2_SERIAL.available() > 0) {
    if (gps2.encode(GPS2_SERIAL.read())) {
      newData2 = true;
    }
  }

  // Display info if we have new data from either GPS
  if (newData1) {
    displayInfo(gps1, "GPS1 (NEO-8M)");
  }
  
  if (newData2) {
    displayInfo(gps2, "GPS2 (NEO-9M)");
  }

  // Check for connection issues
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 5000) { // Check every 5 seconds
    lastCheck = millis();
    
    if (gps1.charsProcessed() < 10) {
      Serial.println(F("No GPS1 data received. Check wiring."));
    }
    
    if (gps2.charsProcessed() < 10) {
      Serial.println(F("No GPS2 data received. Check wiring."));
    }
  }
}