#include <ESP32Servo.h>

const int servoPin = 14;

Servo myServo;

void setup() {
  Serial.begin(115200);
  myServo.attach(servoPin);
}

void loop() {
  for(int posDegrees = 0; posDegrees <= 180; posDegrees++) {
    myServo.write(posDegrees);
    Serial.println(posDegrees);
    delay(20);
  }

  for(int posDegrees = 180; posDegrees >= 0; posDegrees--) {
    myServo.write(posDegrees);
    Serial.println(posDegrees);
    delay(20);
  }
}