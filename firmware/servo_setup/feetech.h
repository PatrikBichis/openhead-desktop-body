// Feetech STS-protokollet (STS3215) på halvduplex TTL-buss via Waveshare-adaptern i läge B.
// Paket: FF FF id len instr params... chk, där len = antal params + 2 och chk = ~(id+len+instr+params).
// Svar:  FF FF id len err params... chk.
#pragma once
#include <Arduino.h>

namespace sts {

// instruktioner
const uint8_t PING = 0x01, READ = 0x02, WRITE = 0x03, SYNC_READ = 0x82, SYNC_WRITE = 0x83;
const uint8_t BROADCAST = 0xFE;

// register (SMS_STS-tabellen)
const uint8_t REG_MODEL = 3, REG_ID = 5, REG_BAUD = 6, REG_MIN_ANGLE = 9, REG_MAX_ANGLE = 11,
              REG_P = 21, REG_D = 22, REG_I = 23, REG_OFFSET = 31, REG_MODE = 33,
              REG_TORQUE = 40, REG_ACC = 41, REG_GOAL_POS = 42, REG_GOAL_SPEED = 46, REG_LOCK = 55,
              REG_PRESENT_POS = 56, REG_PRESENT_SPEED = 58, REG_PRESENT_LOAD = 60,
              REG_VOLTAGE = 62, REG_TEMPERATURE = 63;

// returvärden (samma som ODM_DS_* i protocol.h): 0 ok, 0xFF timeout, annars servots felbyte
const int OK = 0, TIMEOUT = 0xFF;

class Bus {
 public:
  void begin(HardwareSerial& port, uint32_t baud);
  // Skickar en ping till ett ID som inte bör finnas och ser om de egna byten kommer tillbaka.
  bool detectEcho();
  bool echo() const { return echo_; }

  int ping(uint8_t id, uint16_t* model, uint32_t timeout_us = 2000);
  int read(uint8_t id, uint8_t addr, uint8_t len, uint8_t* out);
  int write(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t len);
  // data: n × len byte, i samma ordning som ids
  void syncWrite(uint8_t addr, uint8_t len, const uint8_t* ids, uint8_t n, const uint8_t* data);
  // out: n × len byte, errs: n statuskoder. Returnerar bitmask över index som svarade felfritt.
  uint16_t syncRead(uint8_t addr, uint8_t len, const uint8_t* ids, uint8_t n, uint8_t* out, uint8_t* errs);

 private:
  void send(uint8_t id, uint8_t instr, const uint8_t* params, uint8_t np);
  int reply(uint8_t id, uint8_t* params, uint8_t np, uint32_t timeout_us);
  bool readByte(uint8_t* b, uint32_t deadline_us);
  void drain();

  HardwareSerial* port_ = nullptr;
  bool echo_ = false;
};

}  // namespace sts
