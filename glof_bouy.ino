#include <HardwareSerial.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>  // For BMP390

// MSP protocol commands
#define MSP_GPS 106
#define MSP_ATTITUDE 108

// Flight controller serial (UART1)
HardwareSerial FCSerial(1); // RX=GPIO16, TX=GPIO17

// LoRa pins and config
#define LORA_CS   5
#define LORA_RST  14
#define LORA_IRQ  26
#define LORA_SCK  18
#define LORA_MISO 19
#define LORA_MOSI 23

// I2C pins (optional if default SDA=21, SCL=22 are used)
#define I2C_SDA 21
#define I2C_SCL 22

// LoRa addresses
byte localAddress = 0xB0;
byte receiverAddress = 0xB1;

// Altimeter object
Adafruit_BMP3XX bmp;

void setup() {
  Serial.begin(115200);
  FCSerial.begin(115200, SERIAL_8N1, 16, 17);

  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // Initialize BMP390
  if (!bmp.begin_I2C()) {
    Serial.println("BMP390 not detected!");
    while (1);
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  Serial.println("BMP390 initialized.");

  // Setup SPI and LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed!");
    while (1);
  }

  Serial.println("LoRa initialized.");
}

void sendMSPRequest(uint8_t cmd) {
  uint8_t checksum = 0;
  FCSerial.write('$');
  FCSerial.write('M');
  FCSerial.write('<');
  FCSerial.write((uint8_t)0);
  FCSerial.write(cmd);
  checksum ^= cmd;
  FCSerial.write(checksum);
}

bool readMSPResponse(uint8_t* buffer, uint8_t expectedLength) {
  uint32_t start = millis();
  int index = 0;
  while (millis() - start < 500) {
    if (FCSerial.available()) {
      uint8_t b = FCSerial.read();
      if (index == 0 && b != '$') continue;
      buffer[index++] = b;
      if (index >= expectedLength) return true;
    }
  }
  return false;
}

void sendLoRaMessage(byte destAddress, const char* message) {
  LoRa.beginPacket();
  LoRa.write(destAddress);   // Receiver address
  LoRa.write(localAddress);  // Sender address
  LoRa.print(message);       // Payload
  LoRa.endPacket();
}

void loop() {
  char loraMessage[100];

  // ---- Request GPS ----
  sendMSPRequest(MSP_GPS);
  delay(10);

  uint8_t gpsBuffer[23];
  if (readMSPResponse(gpsBuffer, sizeof(gpsBuffer))) {
    if (gpsBuffer[0] == '$' && gpsBuffer[1] == 'M' && gpsBuffer[2] == '>') {
      uint8_t numSats = gpsBuffer[6];
      int32_t lat = gpsBuffer[7] | (gpsBuffer[8] << 8) | (gpsBuffer[9] << 16) | (gpsBuffer[10] << 24);
      int32_t lon = gpsBuffer[11] | (gpsBuffer[12] << 8) | (gpsBuffer[13] << 16) | (gpsBuffer[14] << 24);

      Serial.println("---- GPS ----");
      Serial.print("Satellites: "); Serial.println(numSats);
      Serial.print("Latitude: "); Serial.println(lat / 10000000.0, 7);
      Serial.print("Longitude: "); Serial.println(lon / 10000000.0, 7);

      snprintf(loraMessage, sizeof(loraMessage), "GPS:%d,%.7f,%.7f", numSats, lat / 10000000.0, lon / 10000000.0);
      sendLoRaMessage(receiverAddress, loraMessage);
    }
  } else {
    Serial.println("No GPS response.");
  }

  delay(50);

  // ---- Request IMU ----
  sendMSPRequest(MSP_ATTITUDE);
  delay(10);

  uint8_t imuBuffer[11];
  if (readMSPResponse(imuBuffer, sizeof(imuBuffer))) {
    if (imuBuffer[0] == '$' && imuBuffer[1] == 'M' && imuBuffer[2] == '>') {
      int16_t roll  = imuBuffer[5] | (imuBuffer[6] << 8);
      int16_t pitch = imuBuffer[7] | (imuBuffer[8] << 8);
      int16_t yaw   = imuBuffer[9] | (imuBuffer[10] << 8);
      Serial.println("---- IMU ----");
      Serial.print("Roll: "); Serial.print(roll / 10.0); Serial.println("°");
      Serial.print("Pitch: "); Serial.print(pitch / 10.0); Serial.println("°");
      Serial.print("Yaw: "); Serial.print(yaw); Serial.println("°");

      snprintf(loraMessage, sizeof(loraMessage), "IMU:%.1f,%.1f,%d", roll / 10.0, pitch / 10.0, yaw);
      sendLoRaMessage(receiverAddress, loraMessage);
    }
  } else {
    Serial.println("No IMU response.");
  }

  delay(50);

  // ---- Read BMP390 Altimeter ----
  if (bmp.performReading()) {
    float temperature = bmp.temperature; // °C
    float pressure = bmp.pressure / 100.0; // hPa
    float altitude = bmp.readAltitude(1013.25); // m
    Serial.println("---- Altimeter (BMP390) ----");
    Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" °C");
    Serial.print("Pressure: "); Serial.print(pressure); Serial.println(" hPa");
    Serial.print("Altitude: "); Serial.print(altitude); Serial.println(" m");
    snprintf(loraMessage, sizeof(loraMessage), "ALT:%.2f,%.2f,%.2f", temperature, pressure, altitude);
    sendLoRaMessage(receiverAddress, loraMessage);
  } else {
    Serial.println("BMP390 reading failed.");
  }

  delay(1000); // Loop delay
}
