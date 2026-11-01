// BNO085 över I2C (SHTP/SH-2), minimalt: kalibrerat gyro, accelerometer och gravitationsvektor.
// Egen implementation i stället för ett bibliotek, som feetech.cpp. Inte provad på hårdvara än.
//
// Värdena är i sensorns ram; body_uno.ino vrider dem till robotens ram med IMU_AXES/IMU_SIGNS (config.h).
// Konvention som en accelerometer: i vila visar acc och grav +9,81 m/s² uppåt (verifiera tecknet på kortet).
#pragma once
#include <Arduino.h>
#include <Wire.h>

class Bno085 {
 public:
  bool begin(TwoWire& wire, uint8_t addr, uint32_t intervalUs = 5000);
  // Läser det som finns. true om något nytt kom. Startar om sensorn om den tystnat.
  bool poll();
  bool ok() const { return ok_ && millis() - lastData_ < 200; }

  float gyro[3] = {0, 0, 0};  // rad/s
  float acc[3] = {0, 0, 0};   // m/s², inklusive gravitation
  float grav[3] = {0, 0, 0};  // m/s², bara gravitationen (sensorns filter)

 private:
  TwoWire* w_ = nullptr;
  uint8_t addr_ = 0x4A;
  uint32_t intervalUs_ = 5000;
  bool ok_ = false, pending_ = false;  // pending_: väntar på att sensorn startat efter reset
  uint32_t lastData_ = 0, lastInit_ = 0;
  uint8_t seq_[6] = {0, 0, 0, 0, 0, 0};
  uint8_t buf_[128];

  bool init();
  bool write(uint8_t channel, const uint8_t* data, uint8_t n);
  int read();  // läser ett paket till buf_ (utan header); returnerar längd, 0 = inget, -1 = fel
  uint8_t lastChannel_ = 0;
  bool enable(uint8_t reportId);
  void parseReports(const uint8_t* p, int n);
};
