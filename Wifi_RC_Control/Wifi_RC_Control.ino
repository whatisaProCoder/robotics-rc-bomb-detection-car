#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <WebSocketsServer.h>
#include <WiFi.h>

const char* ssid = "PRITAMPC-Hotspot";
const char* password = "2x8690+G";

int speed = 127;

// Motor Driver Pins
#define STEERING 13
#define ENA 18
#define IN1 26
#define IN2 27
#define ENB 19
#define IN3 33
#define IN4 32

// Steering Angle
#define LEFT_ANGLE 60
#define MIDDLE_ANGLE 90
#define RIGHT_ANGLE 120

// Metal Detector Input Pin
#define METAL_DETECTOR_PIN 36

// MQ-2 Gas Sensor
#define MQ2_ANALOG_PIN 34  // Connect A0 of MQ2 to GPIO34

WebSocketsServer webSocket(81);
Servo myServo;

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);  // Use UART1

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload,
                    size_t length) {
    String command = String((char*)payload);

    if (command == "forward") {
        Serial.println("Moving Forward");
        moveForward();
    } else if (command == "backward") {
        Serial.println("Moving Backward");
        moveBackward();
    } else if (command == "left") {
        Serial.println("Turning Left");
        steerLeft();
    } else if (command == "right") {
        Serial.println("Turning Right");
        steerRight();
    } else if (command == "stop") {
        Serial.println("Stop Movement");
        stopCar();
    } else {
        int speedSliderValue = command.toInt();
        speed = map(speedSliderValue, 0, 100, 0, 255);
        Serial.print("Speed : ");
        Serial.println(speed);
    }
}

void setup() {
    Serial.begin(115200);

    // Setup Motor Driver Pins as Output
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    myServo.attach(STEERING);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting...");
    }

    Serial.println("Connected! ESP32 IP: " + WiFi.localIP().toString());
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);

    gpsSerial.begin(9600, SERIAL_8N1, 16, 17);  // GPS RX=16, TX=17
    Serial.println("Waiting for GPS signal...");
}

void loop() {
    webSocket.loop();
    StaticJsonDocument<200> jsonDoc;

    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        gps.encode(c);

        if (gps.location.isUpdated()) {
            Serial.print("Latitude: ");
            Serial.print(gps.location.lat(), 6);
            jsonDoc["gpsLatitude"] = gps.location.lat();
            Serial.print(", Longitude: ");
            Serial.println(gps.location.lng(), 6);
            jsonDoc["gpsLongitude"] = gps.location.lng();
            jsonDoc["gpsSpeed"] = gps.speed.kmph();
            jsonDoc["gpsAltitude"] = gps.altitude.meters();
        }
    }

    jsonDoc["metalDetector"] = getMetalDetectorValue();

    jsonDoc["gasSensor"] = getGasSensorValue();

    String jsonString;
    serializeJson(jsonDoc, jsonString);

    webSocket.broadcastTXT(jsonString);
    delay(50);
}

int getMetalDetectorValue() {
    int analogValue = analogRead(METAL_DETECTOR_PIN);  // Read raw analog Input
    int mappedValue = map(analogValue, 0, 4095, 0, 1000);  // Map to 0-100 range

    // int mappedValue = random(10, 100);
    return mappedValue;  // Return mapped value
}

int getGasSensorValue() {
    int gasSensorValue = analogRead(MQ2_ANALOG_PIN);
    int mappedValue = map(gasSensorValue, 0, 4095, 0, 1000);

    //int mappedValue = random(100, 1000);
    return mappedValue;
}

void moveForward() {
    myServo.write(MIDDLE_ANGLE);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    // analogWrite(ENA, speed);
    // analogWrite(ENB, speed);
}

void moveBackward() {
    myServo.write(MIDDLE_ANGLE);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    // analogWrite(ENA, speed);
    // analogWrite(ENB, speed);
}

void stopCar() {
    myServo.write(MIDDLE_ANGLE);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    // analogWrite(ENA, speed);
    // analogWrite(ENB, speed);
}

void steerLeft() { myServo.write(LEFT_ANGLE); }

void steerRight() { myServo.write(RIGHT_ANGLE); }
