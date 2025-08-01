#include <SPI.h>
#include <LoRa.h>

// LoRa pins
#define LORA_CS   5
#define LORA_RST  14
#define LORA_IRQ  26
#define LORA_SCK  18
#define LORA_MISO 19
#define LORA_MOSI 23

// Receiver address
byte localAddress = 0xB1;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed!");
    while (1);
  }

  Serial.print("LoRa Receiver Address: 0x");
  Serial.println(localAddress, HEX);
  Serial.println();
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    byte receiver = LoRa.read();
    byte sender   = LoRa.read();

    String message = "";
    while (LoRa.available()) {
      message += (char)LoRa.read();
    }

    if (receiver == localAddress) {
      if (message.startsWith("GPS:")) {
        int sats;
        float lat, lon;
        sscanf(message.c_str(), "GPS:%d,%f,%f", &sats, &lat, &lon);

        Serial.println("---- GPS ----");
        Serial.print("Satellites: "); Serial.println(sats);
        Serial.print("Latitude: "); Serial.println(lat, 7);
        Serial.print("Longitude: "); Serial.println(lon, 7);
      }

      else if (message.startsWith("IMU:")) {
        float roll, pitch;
        int yaw;
        sscanf(message.c_str(), "IMU:%f,%f,%d", &roll, &pitch, &yaw);

        Serial.println("---- IMU ----");
        Serial.print("Roll: "); Serial.print(roll); Serial.println("°");
        Serial.print("Pitch: "); Serial.print(pitch); Serial.println("°");
        Serial.print("Yaw: "); Serial.print(yaw); Serial.println("°");
      }

      else if (message.startsWith("ALT:")) {
        float temp, pressure, alt;
        sscanf(message.c_str(), "ALT:%f,%f,%f", &temp, &pressure, &alt);

        Serial.println("---- Altimeter (BMP390) ----");
        Serial.print("Temperature: "); Serial.print(temp); Serial.println(" °C");
        Serial.print("Pressure: "); Serial.print(pressure); Serial.println(" hPa"); 
        Serial.print("Altitude: "); Serial.print(alt); Serial.println(" m");
      }

      Serial.println();  // Blank line between messages
    }
  }
}



