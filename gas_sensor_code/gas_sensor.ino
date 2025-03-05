#define MQ135_PIN 34  // Connect A0 of MQ135 to GPIO34

void setup() {
    Serial.begin(115200);
}

void loop() {
    int sensorValue = analogRead(MQ135_PIN);  // Read analog value
    float voltage = sensorValue * (3.3 / 4095.0); // Convert to voltage (ESP32 ADC is 12-bit)
    
    Serial.print("Raw Value: ");
    Serial.print(sensorValue);
    Serial.print(" | Voltage: ");
    Serial.print(voltage, 3);
    Serial.println("V");

    delay(1000);  // Delay 1 second
}

/*
Wiring (ESP32 to MQ-135)
VCC → 3.3V (or 5V)
GND → GND
A0 → GPIO 34 (or any other analog pin)
D0: Outputs HIGH/LOW based on a preset threshold (adjustable via onboard potentiometer).*/