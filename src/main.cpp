#include <Arduino.h>
#include <TeensyThreads.h>
#include <TimeLib.h>
#include "SdFat.h"
#include <ADC.h>
#include <AnalogBufferDMA.h>
#include <SPI.h>
#include <InternalTemperature.h>

// for HX711 load cell
#include "HX711.h"

// for bno055
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
// for bmp280
#include <Adafruit_BMP280.h>

// gps
#include <TinyGPS++.h>

// xbee pin
//  pin 2 with pin7 teensy
//  pin 3 with pin8 teensy

// BMP280 barometric pressure/altitude sensors
Adafruit_BMP280 bmp1(&Wire);  // First sensor on primary I2C bus
Adafruit_BMP280 bmp2(&Wire1); // Second sensor on secondary I2C bus
// float temperature1 = 0.0f, temperature2 = 0.0f;
// float pressure1 = 0.0f, pressure2 = 0.0f;
float altitude1 = 0.0f, altitude2 = 0.0f;
float averageAltitude = 0.0f;
float seaLevelPressure = 1011.3; // Sea level pressure in hPa (customize for your location for better accuracy)
// BNO055 variables--------------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
// Position tracking variables
double xPos = 0, yPos = 0;
double xAdjust = 0;
double yAdjust = 0;
double totalAccel = 0.0f; // Total acceleration magnitude

// global variables for sensor data
imu::Vector<3> euler(0, 0, 0);
imu::Vector<3> linearAccel(0, 0, 0);
sensors_event_t orientationData, linearAccelData;

// Constants for position calculation
double MAXMIN_THRESH = 0.01;
double MINMIN_THRESH = -0.01;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;
int ACCELERATION_SCALE_X = 500;
int ACCELERATION_SCALE_Y = 500;
// velocity = accel*dt (dt in seconds)
// position = 0.5*accel*dt^2
double ACCEL_VEL_TRANSITION;
double ACCEL_POS_TRANSITION;
double DEG_2_RAD = 0.01745329251; // trig functions require radians, BNO055 outputs degrees
float worldAccelX = 0.0f;
float worldAccelY = 0.0f;
// for analog reading------------------------
// for current and voltage reading
const int P1 = A9;
const int P2 = A8;
// for pressure transducer reading
const int P3 = A7;
const int P4 = A6;

// Global variable to analog reading
float analog1 = 0.0f;
float analog2 = 0.0f;
float psi1 = 0.0f; // Global variable to store pressure
float psi2 = 0.0f;

// for SD card---------------------------
#define SD_CONFIG SdioConfig(FIFO_SDIO)
SdFat32 sd;
File32 file;
// bool fileexists = false;

// for valve control
const int Valve_PIN = 28;
const int Camera_PIN = 29;
bool Valve_state = false;
unsigned long valveActivationTime = 0; // Track when valve was activated
bool valveTimerActive = false;         // Track if 3-second timer is running
bool Camera_state = false;
static unsigned long cameraActivatedAt = 0;
// for time
unsigned long recordSN = 1; // Serial number for data records
String timeStr = " ";
// String dateStr = " ";
// teensy internal temp
float InternalTemp = 0.0f;
int St = 0;

// for I2C with Arduino Nano pin 24(SCL) and 25(SDA)
bool remotestate = false;
bool nanoValue1 = false;
bool nanoValue2 = false;
bool nanoValue3 = false;
bool nanoValue4 = false;
unsigned long lastReceiveTime = 0;
bool dataReceived = false;
bool testmode = false;
unsigned long para_height = 5;

// RF frequency - set according to your region (915MHz for US, 868MHz for EU)
// #define RF95_FREQ 500

// float loraFreq = 500.0;          // Default frequency in MHz
// long loraBandwidth = 125000;     // Default bandwidth in Hz
// uint8_t loraCodingRate = 5;      // Default coding rate (4/5)
// uint8_t loraSpreadingFactor = 9; // Default spreading factor
// bool loraUpdatePending = false;

// GPS variables
TinyGPSPlus gps;
float gpsLat = 27.656902f, gpsLng = 85.327446f;
// int gpsSats = 0;
// float gpsHdop = 0.0f;
// bool gpsValid = false;

// for load cell
#define LOADCELL_DOUT_PIN 2
#define LOADCELL_SCK_PIN 3

HX711 scale;
float calibrationFactor = -8.85;
float tare = 0.0;
float weight = 0.0;

// DMA and ADC
ADC *adc = new ADC(); // adc object
const uint32_t initial_average_value = 2048;
const uint32_t buffer_size = 128;

DMAMEM static volatile uint16_t __attribute__((aligned(32))) buffer1[buffer_size];
DMAMEM static volatile uint16_t __attribute__((aligned(32))) buffer2[buffer_size];
AnalogBufferDMA analogBufferDMA1(buffer1, buffer_size, buffer2, buffer_size);

DMAMEM static volatile uint16_t __attribute__((aligned(32))) buffer3[buffer_size];
DMAMEM static volatile uint16_t __attribute__((aligned(32))) buffer4[buffer_size];
AnalogBufferDMA analogBufferDMA2(buffer3, buffer_size, buffer4, buffer_size);

DMAMEM static volatile uint16_t __attribute__((aligned(32))) buffer5[buffer_size];
DMAMEM static volatile uint16_t __attribute__((aligned(32))) buffer6[buffer_size];
AnalogBufferDMA analogBufferDMA3(buffer5, buffer_size, buffer6, buffer_size);

DMAMEM static volatile uint16_t __attribute__((aligned(32))) buffer7[buffer_size];
DMAMEM static volatile uint16_t __attribute__((aligned(32))) buffer8[buffer_size];
AnalogBufferDMA analogBufferDMA4(buffer7, buffer_size, buffer8, buffer_size);

// for xbee communication
#define XBEE_BAUD_RATE 115200
#define SERIAL_TIMEOUT 1000

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

bool writeCSVFile()
{

  // Open file for writing (create if doesn't exist)
  if (!file.open("data.csv", O_RDWR | O_CREAT | O_AT_END))
  {
    return false;
  }
  // Check if file is empty (new file)
  if (file.size() == 0)
  {
    // Replace the existing header line in the writeCSVFile() function
    file.println("SN,Time,remote,valv1,valv2,reast,ignst,parast,Current,Pressure,XPosition,YPosition,Altitude,EulerX,EulerY,EulerZ,AccelX,AccelY,AccelZ,GPSLat,GPSLng,TeensyTemp,p1,p2,weight,totalaccel");
  }
  // Write header and data rows
  String data = String(recordSN++) + "," +
                timeStr + "," +
                String(remotestate ? 1 : 0) + "," +
                String(nanoValue1 ? 1 : 0) + "," +
                String(nanoValue2 ? 1 : 0) + "," +
                String(nanoValue3 ? 1 : 0) + "," +
                String(nanoValue4 ? 1 : 0) + "," +
                Valve_state + "," +
                analog1 + "," +
                analog2 + "," +
                xPos + "," +
                yPos + "," +
                averageAltitude + "," +
                euler.x() + "," +
                euler.y() + "," +
                euler.z() + "," +
                linearAccel.x() + "," +
                linearAccel.y() + "," +
                linearAccel.z() + "," +
                String(gpsLat, 6) + "," +
                String(gpsLng, 6) + "," +
                InternalTemp + "," +
                String(P1) + "," +  
                String(P2) + "," +
                weight + "," +
                totalAccel;
  file.println(data);
  delayMicroseconds(150);
  file.flush();
  file.close();

  return true;
}

// Function to handle I2C receive events
void receiveEvent(int numBytes)
{
  if (numBytes == 7) // Updated from 7 to 15 bytes
  {
    byte dataPacket[7];
    unsigned long startTime = millis();
    int bytesRead = 0;

    // Increase timeout to 50ms and add retry logic
    while (bytesRead < 7 && (millis() - startTime < 60))
    {
      if (Wire2.available())
      {
        dataPacket[bytesRead] = Wire2.read();
        bytesRead++;
        startTime = millis(); // Reset timeout after each successful byte
      }
    }

    if (bytesRead < 7)
    {
      Serial.print("I2C timeout - only received ");
      Serial.print(bytesRead);
      Serial.println(" bytes");
      return;
    }

    // Convert to boolean values (first 7 bytes)
    nanoValue1 = (dataPacket[0] > 0);  // valve1.isOpen()
    nanoValue2 = (dataPacket[1] > 0);  // valve2.isOpen()
    nanoValue3 = (dataPacket[2] > 0);  // outputState1
    nanoValue4 = (dataPacket[3] > 0);  // outputState2
    remotestate = (dataPacket[4] > 0); // masterStateHigh
    testmode = (dataPacket[5] > 0);    // TestMode
    para_height = dataPacket[6];       // parsuitedeploy

    dataReceived = true;
    lastReceiveTime = millis();
  }
  else
  {
    // Clear buffer if wrong packet size
    while (Wire2.available())
    {
      Wire2.read();
    }
    Serial.print("Invalid packet size: ");
    Serial.println(numBytes);
  }
}

// Function to transmit all data in binary format

void transmitBinaryData()
{
  uint8_t buffer[72];
  int index = 0;

  // Add start byte
  buffer[index++] = 0xAA;

  // Pack record serial number (4 bytes)
  uint32_t sn = recordSN;
  memcpy(&buffer[index], &sn, 4);
  index += 4;

  // Pack time as a float to preserve decimal portion (4 bytes)
  float timeValue = minute(now()) + (second(now()) / 100.0f); // Convert to MM.SS format
  memcpy(&buffer[index], &timeValue, 4);
  index += 4;

  // Pack remotestate (1 byte)
  buffer[index++] = remotestate ? 1 : 0;

  // Pack nano values (4 bytes)
  buffer[index++] = nanoValue1 ? 1 : 0;
  buffer[index++] = nanoValue2 ? 1 : 0;
  buffer[index++] = nanoValue3 ? 1 : 0;
  buffer[index++] = nanoValue4 ? 1 : 0;

  // Pack analog values (8 bytes)
  float a1 = analog1;
  float a2 = analog2;
  memcpy(&buffer[index], &a1, 4);
  index += 4;
  memcpy(&buffer[index], &a2, 4);
  index += 4;

  // Pack P1, P2, weight (12 bytes)
  float p1Value = (float)P1;
  memcpy(&buffer[index], &p1Value, 4);
  index += 4;
  float p2Value = (float)P2;
  memcpy(&buffer[index], &p2Value, 4);
  index += 4;
  float weightValue = weight;
  memcpy(&buffer[index], &weightValue, 4);
  index += 4;

  // Pack valve state (1 byte)
  buffer[index++] = Valve_state ? 1 : 0;

  // Pack position values (8 bytes)
  float xp = xPos;
  float yp = yPos;
  memcpy(&buffer[index], &xp, 4);
  index += 4;
  memcpy(&buffer[index], &yp, 4);
  index += 4;

  // Pack altitude (4 bytes)
  float alt = averageAltitude;
  memcpy(&buffer[index], &alt, 4);
  index += 4;

  // Pack euler angles (12 bytes)
  float ex = euler.x(), ey = euler.y(), ez = euler.z();
  memcpy(&buffer[index], &ex, 4);
  index += 4;
  memcpy(&buffer[index], &ey, 4);
  index += 4;
  memcpy(&buffer[index], &ez, 4);
  index += 4;

  // Pack total acceleration (4 bytes)
  float totalAccelValue = totalAccel;
  memcpy(&buffer[index], &totalAccelValue, 4);
  index += 4;

  // Pack GPS coordinates (8 bytes)
  float lat = gpsLat;
  float lng = gpsLng;
  memcpy(&buffer[index], &lat, 4);
  index += 4;
  memcpy(&buffer[index], &lng, 4);
  index += 4;

  // Add end byte
  buffer[index++] = 0x55;

  // Transmit the binary packet through XBee (Serial2)
  Serial2.write(buffer, index);
  Serial2.flush(); // Ensure all data is sent
  Serial.print("Binary data sent: ");
  Serial.print(index);
  Serial.println(" bytes");
}

void task1()
{
  while (true)
  {
    // Serial.println("Task 1 running");
    timeStr = String(minute(now())) + "." + String(second(now()));

    // teensy cpu temperature
    InternalTemp = InternalTemperature.readTemperatureC();

    // loadcell
    if (scale.is_ready())
    {
      weight = (scale.get_units() - tare) / (calibrationFactor * 1000.0);
    }

    // bmp280
    //  Read data from first sensor
    //  temperature1 = bmp1.readTemperature();
    //  pressure1 = bmp1.readPressure() / 100.0; // Convert to hPa
    altitude1 = bmp1.readAltitude(seaLevelPressure);

    // Read data from second sensor
    // temperature2 = bmp2.readTemperature();
    // pressure2 = bmp2.readPressure() / 100.0; // Convert to hPa
    altitude2 = bmp2.readAltitude(seaLevelPressure);

    // Calculate average altitude (redundant measurement for accuracy)
    averageAltitude = (altitude1 + altitude2) / 2.0;
    // Create and write CSV file
    if (nanoValue4)
    {
      writeCSVFile();
    }
    delay(70); // Run every 100 mili second
  }
}

void task2()
{
  bool monitoringAltitude = false;
  float initialAltitude = 0.0f;
  bool valveHasBeenActivated = false;

  while (true)
  {
    // Start monitoring when nanoValue4 becomes 1
    if (nanoValue4 && !monitoringAltitude)
    {
      monitoringAltitude = true;
      initialAltitude = averageAltitude;
      valveHasBeenActivated = false;
      Serial.print("Started monitoring altitude. Initial altitude: ");
      Serial.print(initialAltitude);
      Serial.println(" meters");
    }

    // If monitoring is active and altitude has decreased by 0.2 meters or more, deploy valve
    // BUT only if valve hasn't been activated yet in this monitoring session
    if (monitoringAltitude && !valveHasBeenActivated && (initialAltitude - averageAltitude >= para_height))
    {
      Valve_state = 1;
      valveActivationTime = millis(); // Record activation time
      valveTimerActive = true;        // Start the 2.5-second timer
      valveHasBeenActivated = true;   // Mark that valve has been activated
    }

    // Check if 3 seconds have passed since valve activation
    if (valveTimerActive && Valve_state && (millis() - valveActivationTime >= 5000))
    {
      Valve_state = 0;          // Turn off valve after 5 seconds
      valveTimerActive = false; // Stop the timer
    }

    if (Valve_state)
    {
      analogWriteFrequency(Valve_PIN, 1000); // Set PWM frequency to 1kHz
      analogWrite(Valve_PIN, 192); // Activate valve with full PWM
    }
    else
    {
      analogWrite(Valve_PIN, LOW);
    }

    if (nanoValue3 && !Camera_state)
    {
      Camera_state = 1;
      cameraActivatedAt = millis();
    }

    if (Camera_state && (millis() - cameraActivatedAt >= 600000UL))
    { // 10 minutes = 600,000 ms
      Camera_state = 0;
    }
    digitalWrite(Camera_PIN, Camera_state ? HIGH : LOW);
    // Reset monitoring if nanoValue4 becomes 0
    if (!nanoValue4 && monitoringAltitude)
    {
      monitoringAltitude = false;
      valveHasBeenActivated = false; // Reset for next monitoring session
    }
    delay(100);
  }
}

void task3()
{
  while (true)
  {
    // Check for pending LoRa updates at the beginning

    unsigned long startTime = millis();
    boolean newData = false;

    // Attempt to get new data for ~1 second
    while (millis() - startTime < 100)
    {
      if (Serial1.available() > 0)
      {
        char c = Serial1.read();
        // Uncomment to see raw NMEA data
        // Serial.print(c);
        if (gps.encode(c))
          newData = true;
      }
    }
    if (newData && gps.location.isValid())
    {
      gpsLat = gps.location.lat();
      gpsLng = gps.location.lng();
    }

    // Write header and data rows with BMP280 values
    String data = String(recordSN++) + "," +
                  timeStr + "," +
                  testmode + "," +
                  String(remotestate ? 1 : 0) + "," +
                  String(nanoValue1 ? 1 : 0) + "," +
                  String(nanoValue2 ? 1 : 0) + "," +
                  String(nanoValue3 ? 1 : 0) + "," +
                  String(nanoValue4 ? 1 : 0) + "," +
                  Valve_state + "," +
                  analog1 + "," +
                  analog2 + "," +
                  xPos + "," +
                  yPos + "," +
                  averageAltitude + "," +
                  euler.x() + "," +
                  euler.y() + "," +
                  euler.z() + "," +
                  totalAccel + "," +
                  String(gpsLat, 6) + "," +
                  String(gpsLng, 6) + "," +
                  P1 + "," +
                  P2 + "," +
                  weight;

    Serial.println(data);

    // need to do binary encoding to transmit it
    transmitBinaryData();

    delay(100);
  }
}

void task4_bno055()
{
  while (true)
  {

    // Get events from sensor
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

    // Update global vectors with the data
    euler = imu::Vector<3>(orientationData.orientation.x,
                           orientationData.orientation.y,
                           orientationData.orientation.z);

    linearAccel = imu::Vector<3>(linearAccelData.acceleration.x,
                                 linearAccelData.acceleration.y,
                                 linearAccelData.acceleration.z);

    // Convert to radians and apply your orientation convention
    float heading = orientationData.orientation.x * DEG_2_RAD;
    float pitch = orientationData.orientation.y * DEG_2_RAD;
    float roll = (360.0 - orientationData.orientation.z) * DEG_2_RAD; // Convert from your 360-roll format

    // Create rotation matrix components
    float ch = cos(heading);
    float sh = sin(heading);
    float cp = cos(pitch);
    float sp = sin(pitch);
    float cr = cos(roll);
    float sr = sin(roll);

    // Only calculate the rotation matrix elements we actually use (for X and Y)
    float r00 = ch * cp;
    float r01 = sh * sr - ch * sp * cr;
    float r02 = ch * sp * sr + sh * cr;
    float r10 = sp;
    float r11 = cp * cr;
    float r12 = -cp * sr;

    // Apply rotation to raw accelerometer data
    worldAccelX = r00 * linearAccel.x() + r01 * linearAccel.y() + r02 * linearAccel.z();
    worldAccelY = r10 * linearAccel.x() + r11 * linearAccel.y() + r12 * linearAccel.z();

    // Calculate position adjustments
    xAdjust = ACCEL_POS_TRANSITION * (worldAccelX * ACCELERATION_SCALE_X);
    yAdjust = ACCEL_POS_TRANSITION * (worldAccelY * ACCELERATION_SCALE_Y);
    // Serial.println(yAdjust);
    // float yDeadBand = 0.02; // Adjust based on testing
    // if (abs(yAdjust) < yDeadBand)
    // {
    //   yAdjust = 0;
    // }
    // float xDeadBand = 0.01;
    // if (abs(xAdjust) < xDeadBand)
    // {
    //   xAdjust = 0;
    // }

    // Proper magnitude threshold check (using squared values to avoid sqrt)
    float adjustSquared = xAdjust * xAdjust + yAdjust * yAdjust;
    float threshSquared = MAXMIN_THRESH * MAXMIN_THRESH;

    // Only update position if adjustment magnitude is significant
    if (adjustSquared > threshSquared)
    {
      xPos -= xAdjust;
      yPos -= yAdjust;
    }

    // Calculate heading velocity
    // double Vel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x /
    //             cos(DEG_2_RAD * orientationData.orientation.x);

    // Calculate total acceleration
    totalAccel = sqrt(pow(linearAccel.x(), 2) +
                      pow(linearAccel.y(), 2) +
                      pow(linearAccel.z(), 2));

    // // Apply threshold
    if (totalAccel < 0.4)
    {
      totalAccel = 0;
    }
    delay(BNO055_SAMPLERATE_DELAY_MS);
  }
}

float analogToCurrent(int analogValue)
{
  // For 3.3V reference voltage on Teensy
  float voltage = (analogValue * 3.3) / 4095.0;

  // Convert to current based on ACS712 sensitivity
  // For ACS712-20A: 100mV/A, zero current at VCC/2 (1.65V for 3.3V system)
  float currentAmps = (voltage - 1.65) / 0.075; // 0.075V/A sensitivity for ACS712-20A

  return currentAmps;
}

float analogToVoltage(int analogValue)
{
  // For 3.3V reference voltage on Teensy
  float measuredVoltage = (analogValue * 3.3) / 4095.0;

  // Voltage divider scaling factor
  // If 12V input becomes 3.38V output, the scaling factor is 12/3.38 = 3.55
  float scalingFactor = 12.0 / 3.38;

  // Convert back to actual input voltage
  float actualVoltage = measuredVoltage * scalingFactor;

  return actualVoltage;
}

float analogToPsi(int analogValue)
{
  // Convert analog reading (0-4095) to voltage (0-10V)
  // float voltage = (analogValue * 10.0f) / 4095.0f;
  // Convert voltage (0-10V) to pressure in bar (0-100 bar)
  // float pressureBar = (voltage / 10.0f) * 100.0f;
  // Convert bar to PSI (1 bar = 14.5038 PSI)
  // float pressurePsi = pressureBar * 14.5038f;
  float pressureBar = (analogValue * 100.0f) / 4095.0f;
  // delay(50);
  return pressureBar;
}

void adc0_isr()
{
  static int currentPin = P1;
  uint16_t value = adc->adc0->analogReadContinuous();

  switch (currentPin)
  {
  case P1:
    analog1 = analogToCurrent(value);
    currentPin = P2;
    adc->adc0->startContinuous(P2);
    break;
  case P2:
    analog2 = analogToVoltage(value);
    currentPin = P3;
    adc->adc0->startContinuous(P3);
    break;
  case P3:
    psi1 = analogToPsi(value);
    currentPin = P4;
    adc->adc0->startContinuous(P4);
    break;
  case P4:
    psi2 = analogToPsi(value);
    currentPin = P1;
    adc->adc0->startContinuous(P1);
    break;
  }
}

// --- XBee Command Helpers ---
bool enterCommandMode()
{
  while (Serial2.available())
    Serial2.read();
  delay(1100);
  Serial2.print("+++");
  delay(1100);

  unsigned long start = millis();
  String resp = "";
  while (millis() - start < SERIAL_TIMEOUT)
  {
    if (Serial2.available())
      resp += (char)Serial2.read();
    if (resp.indexOf("OK") != -1)
      return true;
  }
  return false;
}

void sendAT(String cmd)
{
  Serial2.print(cmd + "\r");
  delay(200);
  while (Serial2.available())
    Serial2.read(); // flush
}

void configureXBee()
{
  if (!enterCommandMode())
  {
    Serial.println("XBee: Command mode failed");
    return;
  }
  // sendAT("ATHP 0");                // Disable hardware flow control
  // sendAT("ATID 1011");             // Set PAN ID
  // sendAT("ATCM FFFFFFFE00000000"); // Set coordinator address
  // sendAT("ATBD 7");                // Set baud rate to 115200
  // sendAT("ATAP 0");                // Set API mode to 0 (transparent mode)
  // sendAT("ATNH 1");                // Set node identifier (optional)
  // sendAT("ATMT 3");                // Set maximum transmission retries
  // sendAT("ATRR 3");                // Set retry count for transmission
  // sendAT("ATCE 0");                // message mode
  // // sendAT("ATKY 3FA7C21898D24F619E4A7DB1ED8F102B");
  // sendAT("ATWR");                  // Write settings to non-volatile memory
  // sendAT("ATCN");                  // Exit command mode
  sendAT("ATHP 1");                // Enable hardware flow control
  sendAT("ATID 1011");             // PAN ID
  sendAT("ATCM FFFFFFFE00000000"); // Coordinator address
  sendAT("ATBD 7");                // 115200 baud
  sendAT("ATAP 0");                // Transparent mode
  sendAT("ATNH 1");                // Max hops
  sendAT("ATMT 1");                // Transmission failure threshold
  sendAT("ATRR 1");                // Retry count
  sendAT("ATCE 0");                // Standard router
  sendAT("ATRO 0");                // No delay in serial packetization
  sendAT("ATWR");                  // Save
  sendAT("ATCN");                  // Exit

  Serial.println("XBee: Configured");
}

void setup()
{
  Serial.begin(115200);
  // Wait for USB Serial
  // while (!Serial)
  // {
  //   yield();
  // }

  // Debug prints
  Serial.println("Starting...");
  delay(100);
  // Initialize GPS on Serial1 (pins 0/1)
  Serial1.begin(9600);

  pinMode(P1, INPUT);
  pinMode(P2, INPUT);
  pinMode(P3, INPUT);
  pinMode(P4, INPUT);

  adc->adc0->setAveraging(32);
  adc->adc0->setResolution(12);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED);

  Serial.println("Initializing DMA buffers...");
  analogBufferDMA1.init(adc, ADC_0);
  analogBufferDMA1.userData(initial_average_value);

  analogBufferDMA2.init(adc, ADC_0);
  analogBufferDMA2.userData(initial_average_value);

  analogBufferDMA3.init(adc, ADC_0);
  analogBufferDMA3.userData(initial_average_value);

  analogBufferDMA4.init(adc, ADC_0);
  analogBufferDMA4.userData(initial_average_value);

  // Enable interrupts for ADC0 and pass the ISR function
  adc->adc0->enableInterrupts(adc0_isr);
  adc->adc0->enableDMA();

  adc->adc0->startContinuous(P1);
  // adc->adc0->startContinuous(P2);
  // adc->adc0->startContinuous(P3);

  // Register the ISR
  // attachInterruptVector(IRQ_ADC0, adc0_isr);
  // NVIC_ENABLE_IRQ(IRQ_ADC0);
  setSyncProvider(getTeensy3Time);

  // Initialize the SD.
  if (!sd.begin(SD_CONFIG))
  {
    // sd.initErrorHalt(&Serial);
    Serial.println("SD card initialization failed.");
    // return;
  }
  else
  {
    Serial.println("SD card initialized successfully.");
  }
  // Initialize I2C buses for BMP280 sensors
  Wire.begin();  // Primary I2C bus
  Wire1.begin(); // Secondary I2C bus

  if (!bmp1.begin(0x76))
  {
    Serial.println("Could not find BMP280 sensor #1, check wiring!");
  }
  else
  {
    // Configure settings for first sensor
    bmp1.setSampling(Adafruit_BMP280::MODE_NORMAL,
                     Adafruit_BMP280::SAMPLING_X2,
                     Adafruit_BMP280::SAMPLING_X16,
                     Adafruit_BMP280::FILTER_X16,
                     Adafruit_BMP280::STANDBY_MS_1);
    Serial.println("BMP280 sensor #1 initialized!");
  }

  if (!bmp2.begin(0x76))
  {
    Serial.println("Could not find BMP280 sensor #2, check wiring!");
  }
  else
  {
    // Configure settings for second sensor
    bmp2.setSampling(Adafruit_BMP280::MODE_NORMAL,
                     Adafruit_BMP280::SAMPLING_X2,
                     Adafruit_BMP280::SAMPLING_X16,
                     Adafruit_BMP280::FILTER_X16,
                     Adafruit_BMP280::STANDBY_MS_1);
    Serial.println("BMP280 sensor #2 initialized!");
  }

  // receive data from esp32 via I2C
  Wire2.begin(0x42);      // Initialize as slave with address 0x42
  Wire2.setClock(400000); // 100kHz instead of default 400kHz
  Wire2.onReceive(receiveEvent);
  Serial.println("Teensy I2C Slave initialized on Wire2");
  // valve control
  pinMode(Valve_PIN, OUTPUT);
  analogWriteFrequency(Valve_PIN, 1000); // Set PWM frequency to 1kHz
  pinMode(Camera_PIN, OUTPUT);
  digitalWrite(Valve_PIN, LOW);

  // load cell
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  // scale.set_scale();
  scale.tare(); // Reset to 0
  tare = scale.get_units(10);

  // Initialize BNO055
  if (!bno.begin())
  {
    Serial.println("No BNO055 detected. Check your wiring or I2C address!");
  }
  else
  {
    Serial.println("BNO055 initialized successfully!");
    // Calculate constants based on sample rate
    ACCEL_VEL_TRANSITION = (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
    ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
  }
  delay(1000);
  // Configure XBee
  Serial2.begin(XBEE_BAUD_RATE, SERIAL_8N1); // Initialize XBee serial port
  delay(1000);
  configureXBee();
  Serial.println("Xbee configured successfully!");

  // Start each task with error checking
  if (threads.addThread(task1) == -1)
  {
    Serial.println("Failed to create task 1");
  }
  if (threads.addThread(task2) == -1)
  {
    Serial.println("Failed to create task 2");
  }
  if (threads.addThread(task3) == -1)
  {
    Serial.println("Failed to create task 3");
  }
  if (threads.addThread(task4_bno055) == -1)
  {
    Serial.println("Failed to create BNO055 task");
  }
}

void loop()
{
}
