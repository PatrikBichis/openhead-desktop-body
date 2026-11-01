#include "feetech.h"

namespace sts {

static const uint32_t REPLY_TIMEOUT_US = 2000;

void Bus::begin(HardwareSerial& port, uint32_t baud) {
  port_ = &port;
  port_->begin(baud);
}

void Bus::drain() {
  while (port_->available()) port_->read();
}

bool Bus::readByte(uint8_t* b, uint32_t deadline_us) {
  while ((int32_t)(micros() - deadline_us) < 0) {
    if (port_->available()) {
      *b = (uint8_t)port_->read();
      return true;
    }
  }
  return false;
}

void Bus::send(uint8_t id, uint8_t instr, const uint8_t* params, uint8_t np) {
  uint8_t buf[6 + 64];
  uint8_t n = 0, sum = id + (np + 2) + instr;
  buf[n++] = 0xFF;
  buf[n++] = 0xFF;
  buf[n++] = id;
  buf[n++] = np + 2;
  buf[n++] = instr;
  for (uint8_t i = 0; i < np; i++) {
    buf[n++] = params[i];
    sum += params[i];
  }
  buf[n++] = ~sum;
  drain();
  port_->write(buf, n);
  port_->flush();  // väntar tills allt är utskickat
  if (echo_) {     // adaptern ekar det vi skickar: läs bort samma antal byte
    uint32_t deadline = micros() + 200 + n * 12;
    uint8_t b;
    for (uint8_t i = 0; i < n && readByte(&b, deadline); i++) {
    }
  }
}

int Bus::reply(uint8_t id, uint8_t* params, uint8_t np, uint32_t timeout_us) {
  uint32_t deadline = micros() + timeout_us;
  uint8_t b, prev = 0;
  // leta upp FF FF
  for (;;) {
    if (!readByte(&b, deadline)) return TIMEOUT;
    if (prev == 0xFF && b == 0xFF) break;
    prev = b;
  }
  uint8_t rid, len, err;
  do {  // fler än två FF i rad förekommer
    if (!readByte(&rid, deadline)) return TIMEOUT;
  } while (rid == 0xFF);
  if (!readByte(&len, deadline) || !readByte(&err, deadline)) return TIMEOUT;
  if (len < 2 || (uint8_t)(len - 2) != np || rid != id) {
    // fel svar: läs bort resten och rapportera som timeout för detta ID
    for (uint8_t i = 0; i + 1 < len && readByte(&b, deadline); i++) {
    }
    return TIMEOUT;
  }
  uint8_t sum = rid + len + err;
  for (uint8_t i = 0; i < np; i++) {
    if (!readByte(&params[i], deadline)) return TIMEOUT;
    sum += params[i];
  }
  uint8_t chk;
  if (!readByte(&chk, deadline)) return TIMEOUT;
  if ((uint8_t)~sum != chk) return TIMEOUT;
  return err & 0x7F;  // 0 = ok, annars servots felbitar
}

bool Bus::detectEcho() {
  echo_ = false;
  uint8_t sent[6] = {0xFF, 0xFF, 0xFD, 0x02, PING, (uint8_t)~(0xFD + 0x02 + PING)};
  drain();
  port_->write(sent, sizeof(sent));
  port_->flush();
  uint32_t deadline = micros() + 1500;
  uint8_t b;
  for (uint8_t i = 0; i < sizeof(sent); i++) {
    if (!readByte(&b, deadline) || b != sent[i]) {
      drain();
      return false;
    }
  }
  drain();
  echo_ = true;
  return true;
}

int Bus::ping(uint8_t id, uint16_t* model, uint32_t timeout_us) {
  send(id, PING, nullptr, 0);
  int r = reply(id, nullptr, 0, timeout_us);
  if (r != TIMEOUT && model) {
    uint8_t m[2];
    *model = (read(id, REG_MODEL, 2, m) == OK) ? (uint16_t)(m[0] | (m[1] << 8)) : 0;
  }
  return r;
}

int Bus::read(uint8_t id, uint8_t addr, uint8_t len, uint8_t* out) {
  uint8_t p[2] = {addr, len};
  send(id, READ, p, 2);
  return reply(id, out, len, REPLY_TIMEOUT_US);
}

int Bus::write(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t len) {
  uint8_t p[1 + 32];
  if (len > 32) return TIMEOUT;
  p[0] = addr;
  memcpy(p + 1, data, len);
  send(id, WRITE, p, len + 1);
  if (id == BROADCAST) return OK;
  return reply(id, nullptr, 0, REPLY_TIMEOUT_US);
}

void Bus::syncWrite(uint8_t addr, uint8_t len, const uint8_t* ids, uint8_t n, const uint8_t* data) {
  uint8_t p[2 + 14 * 5];
  if (n == 0 || n * (len + 1) + 2 > (int)sizeof(p)) return;
  uint8_t k = 0;
  p[k++] = addr;
  p[k++] = len;
  for (uint8_t i = 0; i < n; i++) {
    p[k++] = ids[i];
    memcpy(p + k, data + i * len, len);
    k += len;
  }
  send(BROADCAST, SYNC_WRITE, p, k);
}

uint16_t Bus::syncRead(uint8_t addr, uint8_t len, const uint8_t* ids, uint8_t n, uint8_t* out, uint8_t* errs) {
  uint8_t p[2 + 14];
  if (n == 0 || n > 14) return 0;
  p[0] = addr;
  p[1] = len;
  memcpy(p + 2, ids, n);
  send(BROADCAST, SYNC_READ, p, n + 2);
  uint16_t ok = 0;
  for (uint8_t i = 0; i < n; i++) {
    int r = reply(ids[i], out + i * len, len, REPLY_TIMEOUT_US);
    errs[i] = (uint8_t)r;
    if (r == OK) ok |= (1u << i);
  }
  return ok;
}

}  // namespace sts
