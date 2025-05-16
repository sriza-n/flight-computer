#include <Wire.h>
#include <Adafruit_BMP280.h>

// Create sensor objects and explicitly specify the Wire interface for each
Adafruit_BMP280 bmp1(&Wire);  // First sensor on Wire (pins 19,18)
Adafruit_BMP280 bmp2(&Wire1);  // Second sensor on Wire1 (pins 17,16)

// Sea level pressure in hPa (customize for your location for better accuracy)
float seaLevelPressure = 1011.3;

void setup() {
  Serial.begin(115200);
  
  // Wait until serial port opens
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("Dual BMP280 Altitude Sensor Test");
  
  // Initialize Wire and Wire1
  Wire.begin();
  Wire1.begin();
  
  // Initialize the sensors with their addresses
  if (!bmp1.begin(0x76)) {
    Serial.println("Could not find BMP280 sensor #1, check wiring!");
    while (1);
  }
  
  if (!bmp2.begin(0x76)) {
    Serial.println("Could not find BMP280 sensor #2, check wiring!");
    while (1);
  }
  
  // Default settings for first sensor
  bmp1.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_1);
                  
  // Default settings for second sensor
  bmp2.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_1);
}

void loop() {
  // Read data from first sensor
  float temperature1 = bmp1.readTemperature();
  float pressure1 = bmp1.readPressure();
  float altitude1 = bmp1.readAltitude(seaLevelPressure);
  
  // Read data from second sensor
  float temperature2 = bmp2.readTemperature();
  float pressure2 = bmp2.readPressure();
  float altitude2 = bmp2.readAltitude(seaLevelPressure);

  float averageAltitude = (altitude1 + altitude2) / 2.0;
  
  
  // Print readings from first sensor
  Serial.println("=== Sensor #1 (Pins 19,18) ===");
  Serial.print("Temperature: ");
  Serial.print(temperature1);
  Serial.println(" °C");
  
  Serial.print("Pressure: ");
  Serial.print(pressure1 / 100.0);
  Serial.println(" hPa");
  
  Serial.print("Altitude: ");
  Serial.print(altitude1);
  Serial.println(" m");
  
  // Print readings from second sensor
  Serial.println("\n=== Sensor #2 (Pins 17,16) ===");
  Serial.print("Temperature: ");
  Serial.print(temperature2);
  Serial.println(" °C");
  
  Serial.print("Pressure: ");
  Serial.print(pressure2 / 100.0);
  Serial.println(" hPa");
  
  Serial.print("Altitude: ");
  Serial.print(altitude2);
  Serial.println(" m");

  Serial.println("\n=== AVERAGE READINGS ===");
  Serial.print("Average Altitude: ");
  Serial.print(averageAltitude);
  Serial.println(" m");
  
  Serial.println("\n-----------------------");
  delay(100); // Longer delay for readability
}