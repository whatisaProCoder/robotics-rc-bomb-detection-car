#include <Servo.h>

#define MQ2pin A0  // Analog pin for MQ2 sensor
#define THRESHOLD 300 // Define a threshold value for gas detection

Servo myServo; // Create a Servo object
float sensorValue; // Variable to store sensor value

void setup() {
    Serial.begin(9600); // Set serial port to 9600
    Serial.println("MQ2 warming up!");
    delay(20000); // Allow the MQ2 to warm up

    myServo.attach(9); // Attach the servo to pin 9
    myServo.write(0); // Set initial position to 0 degrees
}

void loop() {
    sensorValue = analogRead(MQ2pin); // Read analog input from MQ2 sensor

    Serial.print("Sensor Value: ");
    Serial.println(sensorValue);
    
    if (sensorValue > THRESHOLD) {
        Serial.println("Gas detected! Rotating servo.");
        myServo.write(90); // Rotate servo to 90 degrees
        delay(1000); // Hold position for 1 second
        myServo.write(0); // Return to 0 degrees
    }

    delay(2000); // Wait 2s for the next reading
}
