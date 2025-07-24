#include <Arduino.h>
#include <TeensyThreads.h>
#include <TimeLib.h>
#include "SdFat.h"
#include <ADC.h>
#include <AnalogBufferDMA.h>
#include <SPI.h>
#include <InternalTemperature.h>
#include "HX711.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>

// BMP280 barometric pressure/altitude sensors
Adafruit_BMP280 bmp1(&Wire);
Adafruit_BMP280 bmp2(&Wire1);
float altitude1 = 0.0f, altitude2 = 0.0f;
float averageAltitude = 0.0f;
float seaLevelPressure = 1011.3;

// BNO055 variables
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
double xPos = 0, yPos = 0;
double xAdjust = 0;
double yAdjust = 0;
double totalAccel = 0.0f;
imu::Vector<3> euler(0, 0, 0);
imu::Vector<3> linearAccel(0, 0, 0);
sensors_event_t orientationData, linearAccelData;
double MAXMIN_THRESH = 0.01;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;
int ACCELERATION_SCALE_X = 500;
int ACCELERATION_SCALE_Y = 500;
double ACCEL_VEL_TRANSITION;
double ACCEL_POS_TRANSITION;
double DEG_2_RAD = 0.01745329251;
float worldAccelX = 0.0f;
float worldAccelY = 0.0f;

// Analog reading pins
const int P1 = A9;
const int P2 = A8;
const int P3 = A7;
const int P4 = A6;
float analog1 = 0.0f;
float analog2 = 0.0f;
float psi1 = 0.0f;
float psi2 = 0.0f;

// SD card
#define SD_CONFIG SdioConfig(FIFO_SDIO)
SdFat32 sd;
File32 file;

// Valve control
const int Valve_PIN = 28;
const int Camera_PIN = 29;
bool Valve_state = false;
bool Camera_state = false;
unsigned long valveActivationTime = 0;
bool valveTimerActive = false;
static unsigned long cameraActivatedAt = 0;

// Time
unsigned long recordSN = 1;
String timeStr = " ";
float InternalTemp = 0.0f;

// I2C with Arduino Nano
bool remotestate = false;
bool nanoValue1 = false;
bool nanoValue2 = false;
bool nanoValue3 = false;
bool nanoValue4 = false;
unsigned long lastReceiveTime = 0;
bool dataReceived = false;
bool testmode = false;
unsigned long para_height = 5;

// GPS
TinyGPSPlus gps;
float gpsLat = 27.656902f, gpsLng = 85.327446f;

// Load cell
#define LOADCELL_DOUT_PIN 2
#define LOADCELL_SCK_PIN 3
HX711 scale;
float calibrationFactor = -8.85;
float tare = 0.0;
float weight = 0.0;

// DMA and ADC
ADC *adc = new ADC();
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

// XBee communication
#define XBEE_BAUD_RATE 115200
#define SERIAL_TIMEOUT 1000

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

bool writeCSVFile()
{
  if (!file.open("data.csv", O_RDWR | O_CREAT | O_AT_END))
    return false;
  if (file.size() == 0)
    file.println("SN,Time,remote,valv1,valv2,reast,ignst,parast,Current,Pressure,XPosition,YPosition,Altitude,EulerX,EulerY,EulerZ,AccelX,AccelY,AccelZ,GPSLat,GPSLng,TeensyTemp,P1,P2,Weight,TotalAccel");

  // Optimized CSV row writing using snprintf for efficiency and type safety
  char csvLine[256];
  snprintf(
      csvLine, sizeof(csvLine),
      "%lu,%s,%d,%d,%d,%d,%d,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.2f,%.2f,%.6f,%.6f\n",
      recordSN++,
      timeStr.c_str(),
      remotestate ? 1 : 0,
      nanoValue1 ? 1 : 0,
      nanoValue2 ? 1 : 0,
      nanoValue3 ? 1 : 0,
      nanoValue4 ? 1 : 0,
      Valve_state ? 1 : 0,
      analog1,
      analog2,
      xPos,
      yPos,
      averageAltitude,
      euler.x(),
      euler.y(),
      euler.z(),
      linearAccel.x(),
      linearAccel.y(),
      linearAccel.z(),
      gpsLat,
      gpsLng,
      InternalTemp,
      (float)P1,
      (float)P2,
      weight,
      totalAccel);
  file.print(csvLine);
  delayMicroseconds(150);
  file.flush();
  file.close();
  return true;
}

void receiveEvent(int numBytes)
{
  if (numBytes == 7)
  {
    byte dataPacket[7];
    unsigned long startTime = millis();
    int bytesRead = 0;
    while (bytesRead < 7 && (millis() - startTime < 60))
    {
      if (Wire2.available())
      {
        dataPacket[bytesRead] = Wire2.read();
        bytesRead++;
        startTime = millis();
      }
    }
    if (bytesRead < 7)
    {
      Serial.print("I2C timeout - only received ");
      Serial.print(bytesRead);
      Serial.println(" bytes");
      return;
    }
    nanoValue1 = (dataPacket[0] > 0);
    nanoValue2 = (dataPacket[1] > 0);
    nanoValue3 = (dataPacket[2] > 0);
    nanoValue4 = (dataPacket[3] > 0);
    remotestate = (dataPacket[4] > 0);
    testmode = (dataPacket[5] > 0);
    para_height = dataPacket[6];
    dataReceived = true;
    lastReceiveTime = millis();
  }
  else
  {
    while (Wire2.available())
      Wire2.read();
    Serial.print("Invalid packet size: ");
    Serial.println(numBytes);
  }
}

void transmitBinaryData()
{
  uint8_t buffer[64];
  int index = 0;
  buffer[index++] = 0xAA;
  uint32_t sn = recordSN;
  memcpy(&buffer[index], &sn, 4);
  index += 4;
  float timeValue = minute(now()) + (second(now()) / 100.0f);
  memcpy(&buffer[index], &timeValue, 4);
  index += 4;
  buffer[index++] = remotestate ? 1 : 0;
  buffer[index++] = nanoValue1 ? 1 : 0;
  buffer[index++] = nanoValue2 ? 1 : 0;
  buffer[index++] = nanoValue3 ? 1 : 0;
  buffer[index++] = nanoValue4 ? 1 : 0;
  float a1 = analog1, a2 = analog2;
  memcpy(&buffer[index], &a1, 4);
  index += 4;
  memcpy(&buffer[index], &a2, 4);
  index += 4;
  float p1Value = (float)P1;
  memcpy(&buffer[index], &p1Value, 4);
  index += 4;
  float p2Value = (float)P2;
  memcpy(&buffer[index], &p2Value, 4);
  index += 4;
  float weightValue = weight;
  memcpy(&buffer[index], &weightValue, 4);
  index += 4;
  buffer[index++] = Valve_state ? 1 : 0;
  float xp = xPos, yp = yPos;
  memcpy(&buffer[index], &xp, 4);
  index += 4;
  memcpy(&buffer[index], &yp, 4);
  index += 4;
  float alt = averageAltitude;
  memcpy(&buffer[index], &alt, 4);
  index += 4;
  float ex = euler.x(), ey = euler.y(), ez = euler.z();
  memcpy(&buffer[index], &ex, 4);
  index += 4;
  memcpy(&buffer[index], &ey, 4);
  index += 4;
  memcpy(&buffer[index], &ez, 4);
  index += 4;
  float totalAccelValue = totalAccel;
  memcpy(&buffer[index], &totalAccelValue, 4);
  index += 4;
  float lat = gpsLat, lng = gpsLng;
  memcpy(&buffer[index], &lat, 4);
  index += 4;
  memcpy(&buffer[index], &lng, 4);
  index += 4;
  buffer[index++] = 0x55;
  Serial2.write(buffer, index);
  Serial2.flush();
  Serial.print("Binary data sent: ");
  Serial.print(index);
  Serial.println(" bytes");
}

void task1()
{
  while (true)
  {
    timeStr = String(minute(now())) + "." + String(second(now()));
    InternalTemp = InternalTemperature.readTemperatureC();
    if (scale.is_ready())
      weight = (scale.get_units() - tare) / (calibrationFactor * 1000.0);
    altitude1 = bmp1.readAltitude(seaLevelPressure);
    altitude2 = bmp2.readAltitude(seaLevelPressure);
    averageAltitude = (altitude1 + altitude2) / 2.0;
    if (nanoValue4)
      writeCSVFile();
    delay(70);
  }
}

void task2()
{
  bool monitoringAltitude = false;
  float initialAltitude = 0.0f;
  bool valveHasBeenActivated = false;
  while (true)
  {
    if (nanoValue4 && !monitoringAltitude)
    {
      monitoringAltitude = true;
      initialAltitude = averageAltitude;
      valveHasBeenActivated = false;
      Serial.print("Started monitoring altitude. Initial altitude: ");
      Serial.print(initialAltitude);
      Serial.println(" meters");
    }
    if (monitoringAltitude && !valveHasBeenActivated && (initialAltitude - averageAltitude >= para_height))
    {
      Valve_state = 1;
      valveActivationTime = millis();
      valveTimerActive = true;
      valveHasBeenActivated = true;
    }
    if (valveTimerActive && Valve_state && (millis() - valveActivationTime >= 5000))
    {
      Valve_state = 0;
      valveTimerActive = false;
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
    digitalWrite(Valve_PIN, Valve_state ? HIGH : LOW);
    digitalWrite(Camera_PIN, Camera_state ? HIGH : LOW);
    if (!nanoValue4 && monitoringAltitude)
    {
      monitoringAltitude = false;
      valveHasBeenActivated = false;
    }
    delay(100);
  }
}

void task3()
{
  while (true)
  {
    unsigned long startTime = millis();
    boolean newData = false;
    while (millis() - startTime < 100)
    {
      if (Serial1.available() > 0)
      {
        char c = Serial1.read();
        if (gps.encode(c))
          newData = true;
      }
    }
    if (newData && gps.location.isValid())
    {
      gpsLat = gps.location.lat();
      gpsLng = gps.location.lng();
    }

    Serial.printf(
        "%lu,%s,%d,%d,%d,%d,%d,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%.6f\n",
        recordSN++,
        timeStr.c_str(),
        testmode ? 1 : 0,
        remotestate ? 1 : 0,
        nanoValue1 ? 1 : 0,
        nanoValue2 ? 1 : 0,
        nanoValue3 ? 1 : 0,
        nanoValue4 ? 1 : 0,
        Valve_state ? 1 : 0,
        analog1,
        analog2,
        xPos,
        yPos,
        averageAltitude,
        euler.x(),
        euler.y(),
        euler.z(),
        totalAccel,
        gpsLat,
        gpsLng,
        (float)P1,
        (float)P2,
        weight);
    transmitBinaryData();
    delay(100);
  }
}

void task4_bno055()
{
  while (true)
  {
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    euler = imu::Vector<3>(orientationData.orientation.x,
                           orientationData.orientation.y,
                           orientationData.orientation.z);
    linearAccel = imu::Vector<3>(linearAccelData.acceleration.x,
                                 linearAccelData.acceleration.y,
                                 linearAccelData.acceleration.z);
    float heading = orientationData.orientation.x * DEG_2_RAD;
    float pitch = orientationData.orientation.y * DEG_2_RAD;
    float roll = (360.0 - orientationData.orientation.z) * DEG_2_RAD;
    float ch = cos(heading), sh = sin(heading);
    float cp = cos(pitch), sp = sin(pitch);
    float cr = cos(roll), sr = sin(roll);
    float r00 = ch * cp;
    float r01 = sh * sr - ch * sp * cr;
    float r02 = ch * sp * sr + sh * cr;
    float r10 = sp;
    float r11 = cp * cr;
    float r12 = -cp * sr;
    worldAccelX = r00 * linearAccel.x() + r01 * linearAccel.y() + r02 * linearAccel.z();
    worldAccelY = r10 * linearAccel.x() + r11 * linearAccel.y() + r12 * linearAccel.z();
    xAdjust = ACCEL_POS_TRANSITION * (worldAccelX * ACCELERATION_SCALE_X);
    yAdjust = ACCEL_POS_TRANSITION * (worldAccelY * ACCELERATION_SCALE_Y);
    float adjustSquared = xAdjust * xAdjust + yAdjust * yAdjust;
    float threshSquared = MAXMIN_THRESH * MAXMIN_THRESH;
    if (adjustSquared > threshSquared)
    {
      xPos -= xAdjust;
      yPos -= yAdjust;
    }
    totalAccel = sqrt(pow(linearAccel.x(), 2) +
                      pow(linearAccel.y(), 2) +
                      pow(linearAccel.z(), 2));
    if (totalAccel < 0.4)
      totalAccel = 0;
    delay(BNO055_SAMPLERATE_DELAY_MS);
  }
}

float analogToCurrent(int analogValue)
{
  float voltage = (analogValue * 3.3) / 4095.0;
  float currentAmps = (voltage - 1.65) / 0.075;
  return currentAmps;
}

float analogToVoltage(int analogValue)
{
  float measuredVoltage = (analogValue * 3.3) / 4095.0;
  float scalingFactor = 12.0 / 3.38;
  float actualVoltage = measuredVoltage * scalingFactor;
  return actualVoltage;
}

float analogToPsi(int analogValue)
{
  float pressureBar = (analogValue * 100.0f) / 4095.0f;
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
    Serial2.read();
}

void configureXBee()
{
  if (!enterCommandMode())
  {
    Serial.println("XBee: Command mode failed");
    return;
  }
  sendAT("ATHP 1");
  sendAT("ATID 1011");
  sendAT("ATCM FFFFFFFE00000000");
  sendAT("ATBD 7");
  sendAT("ATAP 0");
  sendAT("ATNH 1");
  sendAT("ATMT 1");
  sendAT("ATRR 1");
  sendAT("ATCE 0");
  sendAT("ATRO 0");
  sendAT("ATWR");
  sendAT("ATCN");
  Serial.println("XBee: Configured");
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting...");
  delay(100);
  Serial1.begin(9600);
  pinMode(P1, INPUT);
  pinMode(P2, INPUT);
  pinMode(P3, INPUT);
  pinMode(P4, INPUT);
  adc->adc0->setAveraging(32);
  adc->adc0->setResolution(12);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED);
  analogBufferDMA1.init(adc, ADC_0);
  analogBufferDMA1.userData(initial_average_value);
  analogBufferDMA2.init(adc, ADC_0);
  analogBufferDMA2.userData(initial_average_value);
  analogBufferDMA3.init(adc, ADC_0);
  analogBufferDMA3.userData(initial_average_value);
  analogBufferDMA4.init(adc, ADC_0);
  analogBufferDMA4.userData(initial_average_value);
  adc->adc0->enableInterrupts(adc0_isr);
  adc->adc0->enableDMA();
  adc->adc0->startContinuous(P1);
  setSyncProvider(getTeensy3Time);
  if (!sd.begin(SD_CONFIG))
    Serial.println("SD card initialization failed.");
  else
    Serial.println("SD card initialized successfully.");
  Wire.begin();
  Wire1.begin();
  if (!bmp1.begin(0x76))
    Serial.println("Could not find BMP280 sensor #1, check wiring!");
  else
  {
    bmp1.setSampling(Adafruit_BMP280::MODE_NORMAL,
                     Adafruit_BMP280::SAMPLING_X2,
                     Adafruit_BMP280::SAMPLING_X16,
                     Adafruit_BMP280::FILTER_X16,
                     Adafruit_BMP280::STANDBY_MS_1);
    Serial.println("BMP280 sensor #1 initialized!");
  }
  if (!bmp2.begin(0x76))
    Serial.println("Could not find BMP280 sensor #2, check wiring!");
  else
  {
    bmp2.setSampling(Adafruit_BMP280::MODE_NORMAL,
                     Adafruit_BMP280::SAMPLING_X2,
                     Adafruit_BMP280::SAMPLING_X16,
                     Adafruit_BMP280::FILTER_X16,
                     Adafruit_BMP280::STANDBY_MS_1);
    Serial.println("BMP280 sensor #2 initialized!");
  }
  Wire2.begin(0x42);
  Wire2.setClock(400000);
  Wire2.onReceive(receiveEvent);
  Serial.println("Teensy I2C Slave initialized on Wire2");
  pinMode(Valve_PIN, OUTPUT);
  pinMode(Camera_PIN, INPUT_PULLUP);
  digitalWrite(Valve_PIN, LOW);
  digitalWrite(Camera_PIN, LOW);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.tare();
  tare = scale.get_units(10);
  if (!bno.begin())
    Serial.println("No BNO055 detected. Check your wiring or I2C address!");
  else
  {
    Serial.println("BNO055 initialized successfully!");
    ACCEL_VEL_TRANSITION = (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
    ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
  }
  delay(1000);
  Serial2.begin(XBEE_BAUD_RATE, SERIAL_8N1);
  delay(1000);
  configureXBee();
  Serial.println("Xbee configured successfully!");
  if (threads.addThread(task1) == -1)
    Serial.println("Failed to create task 1");
  if (threads.addThread(task2) == -1)
    Serial.println("Failed to create task 2");
  if (threads.addThread(task3) == -1)
    Serial.println("Failed to create task 3");
  if (threads.addThread(task4_bno055) == -1)
    Serial.println("Failed to create BNO055 task");
}

void loop()
{
}
