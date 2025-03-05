#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// Motor Driver Pins
#define ENA 14
#define IN1 26
#define IN2 27
#define ENB 25
#define IN3 33
#define IN4 32
#define BUZZ 12
#define VOUT 18

int speed = 0;

void setup() {
    Serial.begin(115200);
    SerialBT.begin("ESP32_BT_Car");  // Bluetooth Device Name

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(BUZZ, OUTPUT);
    pinMode(VOUT, OUTPUT);
    digitalWrite(VOUT, HIGH);
}

void loop() {
    if (SerialBT.available()) {
        char command = SerialBT.read();

        if (command == 'F')
            moveForward();
        else if (command == 'B')
            moveBackward();
        else if (command == 'L')
            turnLeft();
        else if (command == 'R')
            turnRight();
        else if (command == 'S')
            stopCar();
        else if (command == 'V')
            startHorn();
        else if (command == 'v')
            stopHorn();
        else
            setSpeed(command);
    }
    delay(50);
}

void setSpeed(char command) {
    if (command == '0') {
        speed = 0;
    } else if (command == '1') {
        speed = 25;
    } else if (command == '2') {
        speed = 51;
    } else if (command == '3') {
        speed = 76;
    } else if (command == '4') {
        speed = 102;
    } else if (command == '5') {
        speed = 127;
    } else if (command == '6') {
        speed = 153;
    } else if (command == '7') {
        speed = 178;
    } else if (command == '8') {
        speed = 204;
    } else if (command == '9') {
        speed = 229;
    } else if (command == 'q') {
        speed = 255;
    }
}

void startHorn() {
    digitalWrite(BUZZ, HIGH);
    Serial.println("startHorn");
}

void stopHorn() {
    digitalWrite(BUZZ, LOW);
    Serial.println("stopHorn");
}

void moveForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
    Serial.print("moveForward, speed : ");
    Serial.println(speed);
}

void moveBackward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
    Serial.print("moveBackward, speed : ");
    Serial.println(speed);
}

void turnLeft() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
    Serial.print("turnLeft, speed : ");
    Serial.println(speed);
}

void turnRight() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
    Serial.print("turnRight, speed : ");
    Serial.println(speed);
}

void stopCar() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
    Serial.println("stopCar");
}
