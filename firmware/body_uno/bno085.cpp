// BNO085 över I2C, se bno085.h. Protokoll: SHTP-paket med 4 byte header (längd u16 där bit 15 = fortsättning,
// kanal, sekvensnummer per kanal). Kanal 1 = exekverbar (reset), 2 = SH-2-styrning, 3 = sensorrapporter.
#include "bno085.h"

namespace {
const uint8_t CH_EXEC = 1, CH_CONTROL = 2, CH_REPORTS = 3;
const uint8_t SET_FEATURE = 0xFD;
const uint8_t RID_ACC = 0x01, RID_GYRO = 0x02, RID_GRAV = 0x06, RID_TIMEBASE = 0xFB, RID_REBASE = 0xFA;

int16_t i16(const uint8_t* p) { return (int16_t)(p[0] | (p[1] << 8)); }

// längd för rapporterna vi kan stöta på i kanal 3 (SH-2 referensmanual)
int reportLen(uint8_t id) {
  switch (id) {
    case RID_TIMEBASE:
    case RID_REBASE: return 5;
    case 0x01: case 0x02: case 0x03: case 0x04: case 0x06: case 0x07: return 10;
    case 0x08: case 0x09: return 12;
    case 0x05: return 14;
    default: return 0;
  }
}
}  // namespace

bool Bno085::begin(TwoWire& wire, uint8_t addr, uint32_t intervalUs) {
  w_ = &wire;
  addr_ = addr;
  intervalUs_ = intervalUs;
  w_->begin();
  w_->setClock(400000);
  return init();
}

// Mjuk reset; rapporterna slås på i poll() när sensorn hunnit starta. Blockerar inte, så att servoloopen
// fortsätter om sensorn startas om under drift.
bool Bno085::init() {
  lastInit_ = millis();
  ok_ = false;
  uint8_t reset = 1;
  pending_ = write(CH_EXEC, &reset, 1);
  return pending_;
}

bool Bno085::enable(uint8_t reportId) {
  uint8_t p[17] = {SET_FEATURE, reportId, 0, 0, 0};
  p[5] = intervalUs_ & 0xFF;
  p[6] = (intervalUs_ >> 8) & 0xFF;
  p[7] = (intervalUs_ >> 16) & 0xFF;
  p[8] = (intervalUs_ >> 24) & 0xFF;
  return write(CH_CONTROL, p, sizeof(p));
}

bool Bno085::write(uint8_t channel, const uint8_t* data, uint8_t n) {
  uint16_t len = n + 4;
  w_->beginTransmission(addr_);
  w_->write(len & 0xFF);
  w_->write(len >> 8);
  w_->write(channel);
  w_->write(seq_[channel]++);
  w_->write(data, n);
  return w_->endTransmission() == 0;
}

int Bno085::read() {
  // Varje läsning börjar med en header; ett långt paket läses i delar där varje del har en egen header.
  if (w_->requestFrom(addr_, (uint8_t)4) != 4) return -1;
  uint8_t h[4];
  for (uint8_t i = 0; i < 4; i++) h[i] = w_->read();
  uint16_t len = (h[0] | (h[1] << 8)) & 0x7FFF;
  if (len <= 4) return 0;
  lastChannel_ = h[2];
  int payload = len - 4, got = 0;
  const int CHUNK = 28;  // + 4 byte header = 32, säkert för Wire-bufferten
  while (got < payload) {
    int want = min(CHUNK, payload - got);
    if (w_->requestFrom(addr_, (uint8_t)(want + 4)) != (size_t)(want + 4)) return -1;
    for (uint8_t i = 0; i < 4; i++) w_->read();
    for (int i = 0; i < want; i++) {
      uint8_t b = w_->read();
      if (got + i < (int)sizeof(buf_)) buf_[got + i] = b;
    }
    got += want;
  }
  return min(payload, (int)sizeof(buf_));
}

void Bno085::parseReports(const uint8_t* p, int n) {
  int i = 0;
  while (i < n) {
    int len = reportLen(p[i]);
    if (len == 0 || i + len > n) return;
    const uint8_t* r = p + i;
    if (r[0] == RID_ACC || r[0] == RID_GYRO || r[0] == RID_GRAV) {
      float q = r[0] == RID_GYRO ? 1.0f / 512 : 1.0f / 256;  // gyro Q9, acc och grav Q8
      float* out = r[0] == RID_GYRO ? gyro : (r[0] == RID_ACC ? acc : grav);
      for (int k = 0; k < 3; k++) out[k] = i16(r + 4 + 2 * k) * q;
      lastData_ = millis();
    }
    i += len;
  }
}

bool Bno085::poll() {
  if (!w_) return false;
  bool any = false;
  for (int k = 0; k < 4; k++) {  // högst några paket per varv, så att servoloopen inte väntar
    int n = read();
    if (n <= 0) break;
    if (lastChannel_ == CH_REPORTS && ok_) {
      parseReports(buf_, n);
      any = true;
    } else if (lastChannel_ == CH_EXEC && buf_[0] == 1) {
      pending_ = true;  // sensorn har startat om (eller vår reset är klar): slå på rapporterna igen
      lastInit_ = millis();
    }
  }
  if (pending_ && millis() - lastInit_ > 300) {
    pending_ = false;
    ok_ = enable(RID_GYRO) && enable(RID_ACC) && enable(RID_GRAV);
    lastData_ = millis();
  }
  if (!pending_ && !ok() && millis() - lastInit_ > 1000) init();
  return any;
}
