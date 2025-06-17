#include <Arduino.h>
#include <TeensyThreads.h>
#include <TimeLib.h>
#include "SdFat.h"
#include <ADC.h>
#include <SPI.h>
#include <InternalTemperature.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include <RH_RF95.h>
#include <TinyGPS++.h>

// Sensor instances
Adafruit_BMP280 bmp1(&Wire), bmp2(&Wire1);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
RH_RF95 rf95(10, 8); // CS=10, INT=8

// GPS
TinyGPSPlus gps;
float gpsLat = 27.656902f, gpsLng = 85.327446f;

// Sensor data (optimized data types)
float altitude1 = 0.0f, altitude2 = 0.0f, avgAltitude = 0.0f;
float analog1 = 0.0f, analog2 = 0.0f;
float xPos = 0.0f, yPos = 0.0f;
float eulerX = 0.0f, eulerY = 0.0f, eulerZ = 0.0f;
float accelX = 0.0f, accelY = 0.0f, accelZ = 0.0f;
float internalTemp = 0.0f;

// System state
uint32_t recordSN = 1;
bool valveState = false;
bool nanoValues[5] = {false}; // [0-3]=nano values, [4]=remote state
unsigned long lastI2CTime = 0;

// Constants (optimized)
const float SEA_LEVEL_PRESSURE = 1011.3f;
const float DEG_2_RAD = 0.01745329251f;
const float ACCEL_SCALE = 500.0f;
const float POS_THRESHOLD = 0.01f;
const uint8_t SAMPLE_DELAY = 10;
const int VALVE_PIN = 28, P1_PIN = A9, P2_PIN = A8;

// Timing variables
float accelPosTransition;
ADC *adc = new ADC();

#define SD_CONFIG SdioConfig(FIFO_SDIO)
SdFat32 sd;
File32 file;

time_t getTeensy3Time() { return Teensy3Clock.get(); }

inline float analogToPsi(int val) { return (val * 100.0f) / 4095.0f; }
inline float analogToCurrent(int val) { return ((val * 3.3f / 4095.0f) - 1.65f) / 0.075f; }

bool writeData() {
  if (!file.open("data.csv", O_RDWR | O_CREAT | O_AT_END)) return false;
  
  if (file.size() == 0) {
    file.println("SN,Time,Remote,Valve1,Valve2,Reset,Ignition,ParaState,Current,Pressure,XPos,YPos,Altitude,EulerX,EulerY,EulerZ,AccelX,AccelY,AccelZ,GPSLat,GPSLng,Temp");
  }
  
  uint32_t currentTime = now();
  file.print(recordSN++); file.print(',');
  file.print(minute(currentTime)); file.print('.'); file.print(second(currentTime)); file.print(',');
  for(int i = 0; i < 5; i++) { file.print(nanoValues[i]); file.print(','); }
  file.print(valveState); file.print(',');
  file.print(analog1, 2); file.print(','); file.print(analog2, 2); file.print(',');
  file.print(xPos, 3); file.print(','); file.print(yPos, 3); file.print(',');
  file.print(avgAltitude, 2); file.print(',');
  file.print(eulerX, 2); file.print(','); file.print(eulerY, 2); file.print(','); file.print(eulerZ, 2); file.print(',');
  file.print(accelX, 3); file.print(','); file.print(accelY, 3); file.print(','); file.print(accelZ, 3); file.print(',');
  file.print(gpsLat, 6); file.print(','); file.print(gpsLng, 6); file.print(',');
  file.println(internalTemp, 1);
  
  file.flush();
  file.close();
  return true;
}

void transmitData() {
  uint8_t buffer[64];
  int idx = 0;
  
  memcpy(&buffer[idx], &recordSN, 4); idx += 4;
  
  float timeVal = minute(now()) + (second(now()) / 100.0f);
  memcpy(&buffer[idx], &timeVal, 4); idx += 4;
  
  uint8_t flags = 0;
  for(int i = 0; i < 4; i++) if(nanoValues[i]) flags |= (1 << i);
  buffer[idx++] = flags;
  buffer[idx++] = valveState;
  
  float values[] = {analog1, analog2, xPos, yPos, avgAltitude, 
                   eulerX, eulerY, eulerZ, accelX, accelY, accelZ, 
                   gpsLat, gpsLng, internalTemp};
  
  for(int i = 0; i < 14; i++) {
    memcpy(&buffer[idx], &values[i], 4);
    idx += 4;
  }
  
  rf95.send(buffer, idx);
  rf95.waitPacketSent();
}

void receiveEvent(int numBytes) {
  if (numBytes == 5) {
    for (int i = 0; i < 5; i++) {
      nanoValues[i] = Wire2.read() > 0;
    }
    lastI2CTime = millis();
  }
}

void dataTask() {
  static float initialAlt = 0.0f;
  static bool monitoring = false;
  
  while (true) {
    // Read sensors
    internalTemp = InternalTemperature.readTemperatureC();
    altitude1 = bmp1.readAltitude(SEA_LEVEL_PRESSURE);
    altitude2 = bmp2.readAltitude(SEA_LEVEL_PRESSURE);
    avgAltitude = (altitude1 + altitude2) * 0.5f;
    
    // Valve logic
    if (nanoValues[3] && !monitoring) {
      monitoring = true;
      initialAlt = avgAltitude;
    }
    
    if (monitoring && (initialAlt - avgAltitude >= 0.2f)) {
      valveState = true;
    }
    
    digitalWrite(VALVE_PIN, valveState);
    
    // GPS update
    while (Serial1.available() > 0) {
      if (gps.encode(Serial1.read()) && gps.location.isValid()) {
        gpsLat = gps.location.lat();
        gpsLng = gps.location.lng();
      }
    }
    
    // Write data and transmit
    writeData();
    transmitData();
    
    delay(50);
  }
}

void imuTask() {
  sensors_event_t orientation, linearAccel;
  
  while (true) {
    bno.getEvent(&orientation, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&linearAccel, Adafruit_BNO055::VECTOR_LINEARACCEL);
    
    eulerX = orientation.orientation.x;
    eulerY = orientation.orientation.y;
    eulerZ = orientation.orientation.z;
    
    accelX = linearAccel.acceleration.x;
    accelY = linearAccel.acceleration.y;
    accelZ = linearAccel.acceleration.z;
    
    // World frame transformation (optimized)
    float heading = eulerX * DEG_2_RAD;
    float pitch = eulerY * DEG_2_RAD;
    float roll = (360.0f - eulerZ) * DEG_2_RAD;
    
    float ch = cos(heading), sh = sin(heading);
    float cp = cos(pitch), sp = sin(pitch);
    float cr = cos(roll), sr = sin(roll);
    
    float worldX = (ch * cp) * accelX + (sh * sr - ch * sp * cr) * accelY + (ch * sp * sr + sh * cr) * accelZ;
    float worldY = sp * accelX + (cp * cr) * accelY + (-cp * sr) * accelZ;
    
    float xAdj = accelPosTransition * worldX * ACCEL_SCALE;
    float yAdj = accelPosTransition * worldY * ACCEL_SCALE;
    
    if ((xAdj * xAdj + yAdj * yAdj) > (POS_THRESHOLD * POS_THRESHOLD)) {
      xPos -= xAdj;
      yPos -= yAdj;
    }
    
    delay(SAMPLE_DELAY);
  }
}

void adcISR() {
  static bool readingP1 = true;
  uint16_t value = adc->adc0->analogReadContinuous();
  
  if (readingP1) {
    analog1 = analogToCurrent(value);
    adc->adc0->startContinuous(P2_PIN);
  } else {
    analog2 = analogToPsi(value);
    adc->adc0->startContinuous(P1_PIN);
  }
  readingP1 = !readingP1;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  
  // ADC setup
  adc->adc0->setAveraging(16);
  adc->adc0->setResolution(12);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
  adc->adc0->enableInterrupts(adcISR);
  adc->adc0->startContinuous(P1_PIN);
  
  setSyncProvider(getTeensy3Time);
  
  // Initialize peripherals
  sd.begin(SD_CONFIG);
  Wire.begin();
  Wire1.begin();
  Wire2.begin(0x42);
  Wire2.onReceive(receiveEvent);
  
  // Sensor initialization
  bmp1.begin(0x76);
  bmp2.begin(0x76);
  bno.begin();
  
  // LoRa setup
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW); delay(10); digitalWrite(9, HIGH); delay(10);
  rf95.init();
  rf95.setFrequency(467.5);
  rf95.setSignalBandwidth(125000);
  rf95.setCodingRate4(5);
  rf95.setSpreadingFactor(9);
  rf95.setTxPower(23, false);
  
  pinMode(VALVE_PIN, OUTPUT);
  
  // Calculate timing constant
  accelPosTransition = 0.5f * (SAMPLE_DELAY / 1000.0f) * (SAMPLE_DELAY / 1000.0f);
  
  // Start tasks
  threads.addThread(dataTask);
  threads.addThread(imuTask);
}

void loop() {
  // Main loop does nothing - all work in tasks
}
