#include <ESP32Servo.h>

Servo myServo;

// Steering Angle
#define LEFT_ANGLE 60
#define MIDDLE_ANGLE 90
#define RIGHT_ANGLE 120

void setup() {
  Serial.begin(115200);
  myServo.attach(25);  // Connect servo signal to GPIO 25
}

void loop() {
  myServo.write(LEFT_ANGLE);
  Serial.print("Left Angle : ");
  Serial.println(LEFT_ANGLE);
  delay(1000);
  myServo.write(MIDDLE_ANGLE);
  Serial.print("Middle Angle : ");
  Serial.println(MIDDLE_ANGLE);
  delay(1000);
  myServo.write(RIGHT_ANGLE);
  Serial.print("Right Angle : ");
  Serial.println(RIGHT_ANGLE);
  delay(1000);
}