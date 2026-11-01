// Stift och hastigheter för Uno R4 WiFi i Open Duck-riggen.
#pragma once

// Länk till dator: USB (Serial går via ESP32-S3:ans USB-brygga på R4 WiFi). Datorn måste öppna porten
// med samma hastighet. Steg 1 kör seriellt; UDP (WiFiS3) läggs till som en andra Transport senare.
#define HOST_BAUD 460800

// Servobuss: Serial1 (stift 0 RX, 1 TX) -> Waveshare Bus Servo Adapter (A), bygel i läge B.
// Korsa TX/RX och dra gemensam jord.
#define BUS_BAUD 1000000

// Fotbrytare mot GND (INPUT_PULLUP, intryckt = 1 i feet-bitmasken). Ordning: vänster fram, vänster bak,
// höger fram, höger bak.
#define FOOT_PINS {2, 3, 4, 5}

// Antennservon (9 g, PWM)
#define ANTENNA_PINS {9, 10}

// BNO085 i kroppen. Qwiic-kontakten på R4 WiFi är Wire1 (stiften SDA/SCL vid AREF är Wire). Adress 0x4A
// (Adafruit, SA0 låg) eller 0x4B (SparkFun). Verifiera på kortet.
#define IMU_WIRE Wire1
#define IMU_ADDR 0x4A
#define IMU_INTERVAL_US 5000  // 200 Hz
// Sensorns axlar -> robotens (x framåt, y vänster, z upp): robot[i] = IMU_SIGNS[i] * sensor[IMU_AXES[i]].
// Beror på hur kortet monteras. Kontrollera med vickningstest innan fallreflexen slås på: stilla ger acc z ≈ +9,8,
// lutning framåt ger acc x < 0, lutning åt vänster acc y < 0 (samma som simulatorns site "imu").
#define IMU_AXES {0, 1, 2}
#define IMU_SIGNS {1, 1, 1}

// Utan kommandon skickas tillståndet ändå med den här perioden, så att datorn kan övervaka.
#define IDLE_STATE_MS 100
