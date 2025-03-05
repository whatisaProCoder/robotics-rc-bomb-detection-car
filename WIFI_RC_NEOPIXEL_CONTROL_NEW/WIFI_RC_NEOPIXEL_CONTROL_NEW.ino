#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <WebSocketsServer.h>
#include <WiFi.h>

#define PIN 23
#define NUM_LEDS 10

Adafruit_NeoPixel strip =
    Adafruit_NeoPixel(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);

const char* ssid = "PRITAMPC-Hotspot";
const char* password = "2x8690+G";

int speed = 127;

// Motor Driver Pins
#define STEERING 4
#define ENA 12
#define IN1 26
#define IN2 27
#define ENB 14
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
    } else if (command.startsWith("speed-")) {
        // Extract speed value from "speed-0" to "speed-100"
        int speedValue = command.substring(6).toInt();
        if (speedValue >= 0 && speedValue <= 100) {
            speed = map(speedValue, 0, 100, 0, 255);
            Serial.print("Speed set to: ");
            Serial.println(speed);
        }
    } else {
        steerFront();
    }
}

void setup() {
    Serial.begin(115200);

    strip.begin();
    strip.show();

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
        ledred();
        Serial.println("Connecting...");
    }

    Serial.println("Connected! ESP32 IP: " + WiFi.localIP().toString());
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    ledblue();

    gpsSerial.begin(9600, SERIAL_8N1, 16, 17);  // GPS RX=16, TX=17
    Serial.println("Waiting for GPS signal...");
}

void ledred() {
    for (int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, strip.Color(255, 0, 0));
    }
    strip.show();
}

void ledgreen() {
    for (int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, strip.Color(0, 255, 0));
    }
    strip.show();
}

void ledblue() {
    for (int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, strip.Color(0, 0, 255));
    }
    strip.show();
}

void loop() {
    webSocket.loop();
    StaticJsonDocument<200> jsonDoc;
    bool gpsDataAvailable = false;

    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        gps.encode(c);

        if (gps.location.isUpdated()) {
            float latitude = gps.location.lat();
            float longitude = gps.location.lng();
            float speed = gps.speed.kmph();
            float altitude = gps.altitude.meters();

            Serial.print("Latitude: ");
            Serial.print(latitude, 6);
            Serial.print(", Longitude: ");
            Serial.print(longitude, 6);
            Serial.print(", Speed: ");
            Serial.print(speed);
            Serial.print(" km/h, Altitude: ");
            Serial.print(altitude);
            Serial.println(" meters");

            jsonDoc["gpsLatitude"] = latitude;
            jsonDoc["gpsLongitude"] = longitude;
            jsonDoc["gpsSpeed"] = speed;
            jsonDoc["gpsAltitude"] = altitude;

            if (gps.location.isValid()) {
                gpsDataAvailable = true;
            }
        }
    }

    jsonDoc["metalDetector"] = getMetalDetectorValue();
    jsonDoc["gasSensor"] = getGasSensorValue();

    if (gpsDataAvailable) {
        ledgreen();  // Call only if valid GPS data is available
    }

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

    // int mappedValue = random(100, 1000);
    return mappedValue;
}

void moveForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
}

void moveBackward() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
}

void stopCar() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
}

void steerLeft() { myServo.write(LEFT_ANGLE); }

void steerRight() { myServo.write(RIGHT_ANGLE); }

void steerFront() { myServo.write(MIDDLE_ANGLE); }
