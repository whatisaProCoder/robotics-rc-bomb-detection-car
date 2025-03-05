#include <HardwareSerial.h>
#include <TinyGPS++.h>

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);  // Use UART1

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(9600, SERIAL_8N1, 16, 17);  // GPS RX=16, TX=17
    Serial.println("Waiting for GPS signal...");
}

void loop() {
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        gps.encode(c);

        if (gps.location.isUpdated()) {
            Serial.print("Latitude: ");
            Serial.print(gps.location.lat(), 6);
            Serial.print(", Longitude: ");
            Serial.println(gps.location.lng(), 6);
        }
    }
}

/*Explanation
->Uses HardwareSerial(1) since ESP32 has multiple hardware UARTs.
->Reads latitude, longitude.
->Uses GPIO 16 (RX2) and GPIO 17 (TX2) for GPS communication.*/