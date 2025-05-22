#include <Arduino.h>
#include <TeensyThreads.h>
#include <TimeLib.h>
#include "SdFat.h"
#include <ADC.h>
#include <AnalogBufferDMA.h>
#include <SPI.h>
#include <InternalTemperature.h>

// for bno055
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
// for bmp280
#include <Adafruit_BMP280.h>
//for lora 
#include <RH_RF95.h>

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
double xPos = 0, yPos = 0, totalAccel = 0;
double xAdjust = 0;
double yAdjust = 0;

// global variables for sensor data
imu::Vector<3> euler(0, 0, 0);
imu::Vector<3> linearAccel(0, 0, 0);
sensors_event_t orientationData, linearAccelData;

// Constants for position calculation
double MAXMIN_THRESH = 0.01;
double MINMIN_THRESH = -0.01;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;
int ACCELERATION_SCALE_X = 550;
int ACCELERATION_SCALE_Y = 525;
// velocity = accel*dt (dt in seconds)
// position = 0.5*accel*dt^2
double ACCEL_VEL_TRANSITION;
double ACCEL_POS_TRANSITION;
double DEG_2_RAD = 0.01745329251; // trig functions require radians, BNO055 outputs degrees

// for analog reading------------------------
const int P1 = A9;
const int P2 = A8;

// Global variable to analog reading
float analog1 = 0.0f;
float analog2 = 0.0f;

// for SD card---------------------------
#define SD_CONFIG SdioConfig(FIFO_SDIO)
SdFat32 sd;
File32 file;
// bool fileexists = false;

// for valve control
const int Valve_PIN = 28;
const int SWITCH_PIN = 29;
bool Valve_state = false;


// for time
unsigned long recordSN = 1; // Serial number for data records
String timeStr = " ";
// String dateStr = " ";
// teensy internal temp
float InternalTemp = 0.0f;
int St = 0;

// for I2C with Arduino Nano pin 24(SCL) and 25(SDA)
bool nanoValue1 = false;
bool nanoValue2 = false;
bool nanoValue3 = false;
bool nanoValue4 = false;
unsigned long lastReceiveTime = 0;
bool dataReceived = false;

// Pin definitions for LoRa
#define LORA_CS 10
#define LORA_RST 9 
#define LORA_INT 8

// RF frequency - set according to your region (915MHz for US, 868MHz for EU)
#define RF95_FREQ 467.5

// Initialize LoRa instance
RH_RF95 rf95(LORA_CS, LORA_INT);

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

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
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

float analogToCurrent(int analogValue) {
  // For 3.3V reference voltage on Teensy
  float voltage = (analogValue * 3.3) / 4095.0;
  
  // Convert to current based on ACS712 sensitivity
  // For ACS712-20A: 100mV/A, zero current at VCC/2 (1.65V for 3.3V system)
  float currentAmps = (voltage - 1.65) / 0.075; // 0.075V/A sensitivity for ACS712-20A
  
  return currentAmps;
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
    file.println("SN,Valvestate,Time,Pressure1,Pressure2,Thermocouple,loadcell,teensytemp");
  }
  // Write header and data rows
  // String data = String(recordSN++) + "," + Valve_state+ "," + timeStr + "," + psi1 + "," + psi2  + "," + celsius+ "," + weight+ "," + InternalTemp;
  // file.println(data);
  delayMicroseconds(150);
  file.flush();
  file.close();

  return true;
}

// Function to handle I2C receive events
void receiveEvent(int numBytes)
{
  if (numBytes == 4)
  { // Now expecting 4 bytes
    byte dataPacket[4];

    // Read all bytes into buffer
    for (int i = 0; i < 4; i++)
    {
      dataPacket[i] = Wire2.read();
    }

    // Convert to boolean values
    nanoValue1 = (dataPacket[0] > 0);
    nanoValue2 = (dataPacket[1] > 0);
    nanoValue3 = (dataPacket[2] > 0);
    nanoValue4 = (dataPacket[3] > 0); // Handle the new value

    dataReceived = true;
    lastReceiveTime = millis();
  }
}

// Function to transmit data via LoRa
void transmitLoRa(const String& data) {
  // Convert String to char array for RadioHead library
  char buffer[250];
  data.toCharArray(buffer, data.length() + 1);
  
  rf95.send((uint8_t*)buffer, strlen(buffer));
  rf95.waitPacketSent();
  
  Serial.println("Data transmitted via LoRa");
}

// Function to transmit all data in binary format
void transmitBinaryData() {
  uint8_t buffer[80]; // Larger buffer to accommodate all values
  int index = 0;
  
  // Pack record serial number (4 bytes)
  uint32_t sn = recordSN;
  memcpy(&buffer[index], &sn, 4);
  index += 4;
  
  // Pack time - minute and second (2 bytes)
  uint8_t min = minute(now());
  uint8_t sec = second(now());
  buffer[index++] = min;
  buffer[index++] = sec;
  
  // Pack nano values (all 4 booleans in 1 byte)
  uint8_t nanoFlags = 0;
  if (nanoValue1) nanoFlags |= 0x01;
  if (nanoValue2) nanoFlags |= 0x02;
  if (nanoValue3) nanoFlags |= 0x04;
  if (nanoValue4) nanoFlags |= 0x08;
  buffer[index++] = nanoFlags;
  
  // Pack valve state (1 byte)
  buffer[index++] = Valve_state ? 1 : 0;
  
  // Pack analog values (8 bytes)
  float a1 = analog1;
  float a2 = analog2;
  memcpy(&buffer[index], &a1, 4);
  index += 4;
  memcpy(&buffer[index], &a2, 4);
  index += 4;
  
  // Pack position values (8 bytes)
  float xp = -xPos;  // Converting double to float for transmission
  float yp = -yPos;
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
  
  // Pack linear acceleration (12 bytes)
  float lx = linearAccel.x(), ly = linearAccel.y(), lz = linearAccel.z();
  memcpy(&buffer[index], &lx, 4);
  index += 4;
  memcpy(&buffer[index], &ly, 4);
  index += 4;
  memcpy(&buffer[index], &lz, 4);
  index += 4;
  
  // Pack hardcoded values (12 bytes)
  float hc1 = 12.123456f;
  float hc2 = 12.123456f;
  float hc3 = 1234.12f;
  memcpy(&buffer[index], &hc1, 4);
  index += 4;
  memcpy(&buffer[index], &hc2, 4);
  index += 4;
  memcpy(&buffer[index], &hc3, 4);
  index += 4;
  
  // Pack temperature (4 bytes)
  float temp = InternalTemp;
  memcpy(&buffer[index], &temp, 4);
  index += 4;
  
  // Transmit the binary packet
  rf95.send(buffer, index);
  rf95.waitPacketSent();
  
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
    writeCSVFile();
    // if (!writeCSVFile())
    // {
    //   // Serial.println("Will retry file write on next iteration");
    //   // return;
    // }
    delay(50); // Run every 100 mili second
  }
}

void task2()
{
  bool monitoringAltitude = false;
  float initialAltitude = 0.0f;
  
  while (true)
  {
    // Start monitoring when nanoValue4 becomes 1
    if (nanoValue4 && !monitoringAltitude) {
      monitoringAltitude = true;
      initialAltitude = averageAltitude;
      Serial.print("Started monitoring altitude. Initial altitude: ");
      Serial.print(initialAltitude);
      Serial.println(" meters");
    }
    
    // If monitoring is active and altitude has decreased by 5 meters or more, deploy valve
    if (monitoringAltitude && (initialAltitude - averageAltitude >= 0.2)) {
      Valve_state = 1;
      Serial.print("Altitude decreased by 5+ meters (from ");
      Serial.print(initialAltitude);
      Serial.print(" to ");
      Serial.print(averageAltitude);
      Serial.println("). Valve deployed.");
      // monitoringAltitude = false; // Stop monitoring after deployment
    }
    
    digitalWrite(Valve_PIN, Valve_state ? HIGH : LOW);
    // // Reset monitoring if nanoValue4 becomes 0
    // if (!nanoValue4 && monitoringAltitude) {
    //   monitoringAltitude = false;
    //   Serial.println("Stopped monitoring altitude");
    // }
    
    delay(100);
  }
}

void task3()
{

  while (true)
  { // listen for incoming messages
    // Serial.println("Task 3 running");
    // String DataRf1 = "T:" + timeStr + ",p1:" + String(psi1) + ",p2:" + String(psi2);
    // // String DataRf2 = "p3:" + String(psi3) + ",p4:" + String(psi4);
    // String DataRf3 = ",ST:" + String(St) + ",T1:" + String(celsius) + ",LC:" + String(weight);
    // Write header and data rows
    // Write header and data rows with BMP280 values
    String data = String(recordSN++) + "," +
                  timeStr + "," +
                  String(nanoValue1 ? 1 : 0) + "," +
                  String(nanoValue2 ? 1 : 0) + "," +
                  String(nanoValue3 ? 1 : 0) + "," +
                  String(nanoValue4 ? 1 : 0) + "," +
                  Valve_state + "," +
                  analog1 + "," +
                  analog2 + "," +
                  -xPos + "," +
                  -yPos + "," +
                  averageAltitude + "," +
                  euler.x() + "," +
                  euler.y() + "," +
                  euler.z() + "," +
                  linearAccel.x() + "," +
                  linearAccel.y() + "," +
                  linearAccel.z() + ",12.123456,12.123456,1234.12," +
                  InternalTemp;

    Serial.println(data);

        // Transmit data
    // transmitLoRa(data);

    // need to do binary encoding to transmit it
    transmitBinaryData();
    
    // String data2 = String(nanoValue1 ? 1 : 0) + "," +
    //                String(nanoValue2 ? 1 : 0) + "," +
    //                String(nanoValue3 ? 1 : 0) + "," +
    //                String(nanoValue4 ? 1 : 0);
    // Serial.println(data2);

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

    // Calculate position adjustments
    xAdjust = ACCEL_POS_TRANSITION * (linearAccelData.acceleration.x * ACCELERATION_SCALE_X);
    yAdjust = ACCEL_POS_TRANSITION * (linearAccelData.acceleration.y * ACCELERATION_SCALE_Y);

    // Only update position if adjustment is significant
    if ((xAdjust + yAdjust > MAXMIN_THRESH) || (xAdjust + yAdjust < MINMIN_THRESH))
    {
      xPos = xPos + xAdjust; // Note: fixed the calculation to add, not negate
      yPos = yPos + yAdjust;
    }

    // Calculate heading velocity
    // double Vel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x /
    //             cos(DEG_2_RAD * orientationData.orientation.x);

    // Calculate total acceleration
    totalAccel = sqrt(pow(linearAccel.x(), 2) +
                      pow(linearAccel.y(), 2) +
                      pow(linearAccel.z(), 2));

    // Apply threshold
    if (totalAccel < 0.4)
    {
      totalAccel = 0;
    }
    delay(BNO055_SAMPLERATE_DELAY_MS);
  }
}
// Function to initialize LoRa
bool initLoRa() {
  pinMode(LORA_RST, OUTPUT);
  
  // Reset LoRa module
  digitalWrite(LORA_RST, LOW);
  delay(10);
  digitalWrite(LORA_RST, HIGH);
  delay(10);
  
  // Initialize LoRa
  if (!rf95.init()) {
    Serial.println("LoRa initialization failed");
    return false;
  }
  
  // Set frequency
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("Setting frequency failed");
    return false;
  }

  rf95.setSignalBandwidth(125000);
  rf95.setCodingRate4(5);
  rf95.setSpreadingFactor(7);

  
  
  // Set transmitter power (5 to 23 dBm)
  rf95.setTxPower(23, false);
  
  Serial.println("LoRa initialized successfully");
  return true;
}

void adc0_isr()
{
  static int currentPin = P1;
  uint16_t value = adc->adc0->analogReadContinuous();

  switch (currentPin)
  {
  case P1:
    // Serial.print("ISR ADC value (P1): ");
    // Serial.println(value);
    analog1 = analogToCurrent(value);
    currentPin = P2;
    adc->adc0->startContinuous(P2);
    break;
  case P2:
    // Serial.print("ISR ADC value (P2): ");
    // Serial.println(value);
    analog2 = analogToPsi(value);
    currentPin = P1;
    adc->adc0->startContinuous(P1);
    break;
  }
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

  pinMode(P1, INPUT);
  pinMode(P2, INPUT);

  adc->adc0->setAveraging(32);
  adc->adc0->setResolution(12);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED);

  Serial.println("Initializing DMA buffers...");
  analogBufferDMA1.init(adc, ADC_0);
  analogBufferDMA1.userData(initial_average_value);

  analogBufferDMA2.init(adc, ADC_0);
  analogBufferDMA2.userData(initial_average_value);

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

  // receive data from nano
  Wire2.begin(0x42); // Initialize as slave with address 0x42
  Wire2.onReceive(receiveEvent);
  Serial.println("Teensy I2C Slave initialized on Wire2");
  // valve control
  pinMode(Valve_PIN, OUTPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  digitalWrite(Valve_PIN, LOW);

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

  if (!initLoRa()) {
  Serial.println("LoRa failed to initialize");
  }


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
  // receiveMessage();
  // better formatting with leading zeros:
  // String time = (hour(now()) < 10 ? "0" : "") + String(hour(now())) + ":" + (minute(now()) < 10 ? "0" : "") + String(minute(now())) + ":" + (second(now()) < 10 ? "0" : "") + String(second(now()));
  // timeStr =String(hour(now())) + ":" +  String(minute(now())) + ":" + String(second(now()))+ "." +
  //        // Get milliseconds and ensure 3 digits
  //        String(millis() % 1000 + 1000).substring(1);
  // dateStr = String(day()) + "/" + String(month()) + "/" + String(year());
  // Serial.println(timeStr);
  // Serial.println(dateStr);

  // delay(100);
  // if (scale.is_ready()) {
  //   weight = (scale.get_units() - tare) / (calibrationFactor * 1000.0);
  // }
  // delay(50);
}