/*
  Controls servo position from 0-180 degrees and back
  https://wokwi.com/projects/350037178957431378
  by dlloydev, December 2022.
*/

#include <Servo.h>

Servo myservo = Servo();

const int servoPin = 14;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect
  }
  
  myservo.attach(servoPin);
  Serial.println("Servo Test Starting...");
  Serial.println("Pin: " + String(servoPin));
  Serial.println("Position Range: 0-180 degrees");
  Serial.println("Channel: " + String(myservo.attached(servoPin)));
  Serial.println("------------------------");
}

void loop() {
  Serial.println("Moving servo from 0 to 180 degrees:");
  for (int pos = 0; pos <= 180; pos++) {
    myservo.write(servoPin, pos);
    Serial.print("Position: ");
    Serial.print(pos);
    Serial.println(" degrees");
    delay(25);
  }

   delay(1000);
  Serial.println("\nMoving servo from 180 to 0 degrees:");
  for (int pos = 180; pos >= 0; pos--) {
    myservo.write(servoPin, pos);
    Serial.print("Position: ");
    Serial.print(pos);
    Serial.println(" degrees");
    delay(25);
  }
  
  Serial.println("\nCycle complete. Restarting...\n");
  delay(1000); // pause between cycles
}