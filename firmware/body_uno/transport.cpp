#include "transport.h"

uint16_t odmCrc16(const uint8_t* data, size_t n) {  // CRC-16/CCITT-FALSE
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}

void odmFinish(void* pkt, size_t len, uint8_t type, uint16_t seq) {
  uint8_t* p = (uint8_t*)pkt;
  p[0] = ODM_MAGIC & 0xFF;
  p[1] = ODM_MAGIC >> 8;
  p[2] = ODM_VERSION;
  p[3] = type;
  p[4] = seq & 0xFF;
  p[5] = seq >> 8;
  uint16_t crc = odmCrc16(p, len - 2);
  p[len - 2] = crc & 0xFF;
  p[len - 1] = crc >> 8;
}

static size_t sizeOf(uint8_t type) {
  switch (type) {
    case ODM_TYPE_COMMAND: return ODM_SIZE_COMMANDPACKET;
    case ODM_TYPE_STATE: return ODM_SIZE_STATEPACKET;
    case ODM_TYPE_DIAG: return ODM_SIZE_DIAGPACKET;
  }
  return 0;
}

void SerialTransport::begin() {
  // Serial på R4 WiFi; för andra Stream-objekt får anroparen starta porten själv
  if (&s_ == &Serial) Serial.begin(baud_);
}

void SerialTransport::shift(size_t k) {
  memmove(buf_, buf_ + k, n_ - k);
  n_ -= k;
}

const uint8_t* SerialTransport::poll(uint8_t* type) {
  static uint8_t out[128];
  while (s_.available() && n_ < sizeof(buf_)) buf_[n_++] = (uint8_t)s_.read();
  for (;;) {
    // synka på magic (0x44 0x4F)
    size_t i = 0;
    while (i + 1 < n_ && !(buf_[i] == 0x44 && buf_[i + 1] == 0x4F)) i++;
    if (i) {
      dropped_ += i;
      shift(i);
    }
    if (n_ < 4) return nullptr;
    size_t len = sizeOf(buf_[3]);
    if (buf_[2] != ODM_VERSION || len == 0) {
      dropped_++;
      shift(1);
      continue;
    }
    if (n_ < len) return nullptr;
    uint16_t crc = buf_[len - 2] | (buf_[len - 1] << 8);
    if (odmCrc16(buf_, len - 2) != crc) {
      dropped_++;
      shift(1);
      continue;
    }
    memcpy(out, buf_, len);
    *type = buf_[3];
    shift(len);
    return out;
  }
}
