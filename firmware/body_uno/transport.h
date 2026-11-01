// Länken till dator/telefon. Samma paket (protocol.h) oavsett transport: USB-seriellt nu, UDP senare.
#pragma once
#include <Arduino.h>

#include "protocol.h"

class Transport {
 public:
  virtual ~Transport() {}
  virtual void begin() = 0;
  // Läser in det som finns och returnerar ett helt paket (pekare till intern buffert) eller nullptr.
  virtual const uint8_t* poll(uint8_t* type) = 0;
  virtual void send(const void* pkt, size_t len) = 0;
};

uint16_t odmCrc16(const uint8_t* data, size_t n);

// Fyller i magic, version, typ, seq och crc. len = paketets totala storlek.
void odmFinish(void* pkt, size_t len, uint8_t type, uint16_t seq);

// Seriell länk: paketgränser hittas med magic + typ + längd, kontrolleras med CRC.
class SerialTransport : public Transport {
 public:
  SerialTransport(Stream& s, uint32_t baud) : s_(s), baud_(baud) {}
  void begin() override;
  const uint8_t* poll(uint8_t* type) override;
  void send(const void* pkt, size_t len) override { s_.write((const uint8_t*)pkt, len); }
  uint32_t dropped() const { return dropped_; }

 private:
  Stream& s_;
  uint32_t baud_;
  uint8_t buf_[128];
  size_t n_ = 0;
  uint32_t dropped_ = 0;
  void shift(size_t k);
};
