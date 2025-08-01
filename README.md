                        GOLF BOUY

<img width="569" height="427" alt="image" src="https://github.com/user-attachments/assets/ba224dc0-b150-4204-ba5e-74b8f695f64a" />


Project Title: Golf Buoy – Water Level Monitoring System for Glacial Lakes

Objective:
The primary objective of this project is to monitor water level rise in glacier-fed lakes using a compact, floating buoy system. The buoy collects water depth and altitude data, determines GPS position and orientation, and sends it wirelessly via LoRa to a remote station for monitoring and analysis.

 Hardware Components
 
Component	Purpose
ESP32 (x2)	One for sensor data acquisition and transmission, one for data reception
Eagle i Flight Controller	Provides GPS and IMU heading data
GPS Module	Connected to flight controller for location data
Altimeter Sensor	Measures altitude of the buoy (for water level changes)
Echosounder	Measures water depth below the buoy
LoRa Module (SX1278 or similar) x2	Wireless data transmission
Power Module	Monitors battery voltage/current
Buck Converter (24V to 5V)	Steps down voltage to power flight controller
11.1V LiPo Battery	Primary power source
Switch	Manual ON/OFF control for the system

system Overview
1.	Sensing Unit (on buoy):
o	ESP32 collects:
	Altimeter & echosounder data via I2C/analog
	IMU & GPS data from Eagle i FC via UART
	Voltage data from the power module
o	Sends all data via LoRa
2.	Receiver Unit (on ground):
o	ESP32 + LoRa receives data
o	Displays real-time readings via Serial Monitor (Arduino IDE)

      Wiring Diagram & Explanation
      Power Supply
•	11.1V LiPo Battery → [Switch] → [Buck Converter 24V to 5V]
•	Buck Converter Output (5V) → Flight Controller (via power input port)
•	Flight Controller → 5V regulated output to power ESP32 TX unit

         Power Distribution
                                                            [11.1V LiPo Battery]
                                                                             ↓
                                                                     [Switch]
                                                                             ↓
                                                    [24V to 5V Buck Converter]
                                                                             ↓
                                                       [5V Output from Buck]
                                                                           ├──> Eagle i Flight Controller (Power Port)
                                                                            └──> ESP32 (VIN pin)

Sensor Connections to ESP32 (Transmitting Unit on Buoy)
1. Echosounder (UART)
ESP32 Pin	Echosounder Pin	Note
RX (GPIO16 or any)	TX	From sonar to ESP32
TX (GPIO17 or any)	RX	From ESP32 to sonar
GND	GND	Shared ground
5V or 3.3V	VCC	Depends on sonar model
2. Altimeter (I2C)
ESP32 Pin	Altimeter Pin
SDA (GPIO21)	SDA
SCL (GPIO22)	SCL
GND	GND
3.3V or 5V	VCC (depends on sensor)
3. Power Module (I2C)
ESP32 Pin	Power Module Pin
SDA (shared with altimeter)	SDA
SCL (shared with altimeter)	SCL
GND	GND
VCC	3.3V or 5V
 UART Data from Flight Controller to ESP32
•	Use a UART port on the Eagle i flight controller to send GPS and heading data to the ESP32.
Eagle i FC UART Port	ESP32 Pin
TX	RX
RX	TX
GND	GND
(5V is already shared from buck)	
Set the correct baud rate in the ESP32 code to match the FC telemetry output (usually 115200).

 LoRa Module Wiring (SPI)
ESP32 Pin	LoRa Module
GPIO18	SCK
GPIO19	MISO
GPIO23	MOSI
GPIO5	NSS (
GPIO14	RST
GPIO26	DIO0
GND	GND
3.3V	VCC (use 3.3V only; LoRa is not 5V tolerant)
ESP32 (RX - Ground Receiver Side)
Connection	Sensor/Module                    	Interface	Notes
                                	LoRa Module	SPI	Receives data from buoy
USB	Arduino IDE Serial Monitor	USB	Displays sensor data

 Data Format (Example)

  "altitude": 256.8,
  "depth": 3.4,
  "voltage": 11.6,
  "heading": 74.5,
  "gps": { "lat": 31.01234, "lon": 77.06543 }


 Key Notes
•	ESP32 must have proper UART baud rate configured to read data from Eagle i flight controller.
•	Altimeter and echosounder should be tested and calibrated for accuracy before deployment.
•	Use appropriate waterproof casing for buoy electronics.

                                                         Connection Diagram

<img width="940" height="576" alt="image" src="https://github.com/user-attachments/assets/6b4db816-aae1-465c-870f-af6b7763d141" />








