// Open Duck Mini – Uno R4 WiFi: servobuss, fotbrytare, antenner och säkerhet.
// Paketformat: protocol.h (genererad från python/src/openhead_body/protocol.py, CLAUDE.md avsnitt 5).
//
// Uno är "dum men säker": den utför ledmål, rapporterar tillstånd och stänger av momentet om kommandon
// slutar komma (watchdog) eller vid nödstopp. Momentet slås på från servonas nuvarande position och
// målen begränsas till PARAM_MAX_STEP råenheter per kommando, så att ett felaktigt första mål inte ger
// ett ryck.
//
// Fallreflex (protocol.py, PARAM_FALL_*): Uno läser BNO085 i kroppen och reagerar själv på ett fall inom några ms
// (telefonen över Wi-Fi är för långsam, sim/fall_test.py). Målen sätts till reflexposen för fallriktningen, som
// datorn/telefonen laddat upp i råenheter (OP_SET_REFLEX), och momentet slås av efter PARAM_FALL_LIMP_MS.
#include <Arduino_LED_Matrix.h>
#include <Servo.h>

#include "bno085.h"
#include "config.h"
#include "feetech.h"
#include "protocol.h"
#include "transport.h"

static sts::Bus bus;
static SerialTransport link(Serial, HOST_BAUD);
static ArduinoLEDMatrix matrix;
static Servo antenna[2];
static const uint8_t footPins[4] = FOOT_PINS;
static const uint8_t antennaPins[2] = ANTENNA_PINS;
static Bno085 imu;
static const uint8_t imuAxes[3] = IMU_AXES;
static const int8_t imuSigns[3] = IMU_SIGNS;

// ledkonfiguration
static uint8_t jointId[ODM_N_JOINTS];  // servo-ID per led, ODM_NO_SERVO = saknas
static uint16_t activeMask = 0;        // leder som svarade vid start eller efter OP_SET_IDS
static uint8_t actIdx[ODM_N_JOINTS], actIds[ODM_N_JOINTS], nAct = 0;
static uint16_t param[ODM_N_PARAMS];

// styrtillstånd
static bool torqueOn = false, wdLatched = false, estopLatched = false;
static uint16_t cmdTarget[ODM_N_JOINTS], appliedTarget[ODM_N_JOINTS];
static uint16_t lastSeq = 0;
static uint32_t lastCmdMs = 0, lastStateMs = 0;
static uint8_t antennaVal[2] = {ODM_ANTENNA_OFF, ODM_ANTENNA_OFF};

// fallreflex
static uint16_t reflexPose[ODM_N_FALL_DIRS][ODM_N_JOINTS];
static bool fallLatched = false, fallArmed = false, fallLimp = false;
static uint32_t fallMs = 0;
static float bodyGyro[3], bodyAcc[3], bodyUp[3] = {0, 0, 1};  // robotens ram; bodyUp = uppåt (enhetsvektor)

// senaste avläsning
static StatePacket st;
static uint8_t rr = 0;  // rundgång för temperatur/spänning
static bool servoErrorThisCycle = false;

// ---------------------------------------------------------------------------------------------- LED-matris
enum Face { F_BOOT, F_IDLE, F_TORQUE, F_ALARM, F_ERROR };
static const char* const FACES[][8] = {
    {"............", "............", "............", ".##..##..##.", ".##..##..##.", "............", "............", "............"},
    {"............", "....####....", "...#....#...", "...#....#...", "...#....#...", "...#....#...", "....####....", "............"},
    {"............", ".#########..", ".#.......##.", ".#.......##.", ".#.......##.", ".#########..", "............", "............"},
    {"#..........#", ".#........#.", "..#......#..", "...#....#...", "...#....#...", "..#......#..", ".#........#.", "#..........#"},
    {".....##.....", ".....##.....", ".....##.....", ".....##.....", ".....##.....", "............", ".....##.....", ".....##....."},
};
static int shownFace = -1;

static void showFace(Face f) {
  if (shownFace == f) return;
  uint8_t frame[8][12];
  for (int r = 0; r < 8; r++)
    for (int c = 0; c < 12; c++) frame[r][c] = FACES[f][r][c] == '#';
  matrix.renderBitmap(frame, 8, 12);
  shownFace = f;
}

// ---------------------------------------------------------------------------------------------- servon
static void rebuildActive() {
  nAct = 0;
  for (uint8_t j = 0; j < ODM_N_JOINTS; j++) {
    if (activeMask & (1u << j)) {
      actIdx[nAct] = j;
      actIds[nAct] = jointId[j];
      nAct++;
    }
  }
}

static void probeJoints() {
  activeMask = 0;
  for (uint8_t j = 0; j < ODM_N_JOINTS; j++)
    if (jointId[j] != ODM_NO_SERVO && bus.ping(jointId[j], nullptr) != sts::TIMEOUT) activeMask |= (1u << j);
  rebuildActive();
}

static void setTorque(bool on) {
  uint8_t v[ODM_N_JOINTS];
  memset(v, on ? 1 : 0, sizeof(v));
  bus.syncWrite(sts::REG_TORQUE, 1, actIds, nAct, v);
  torqueOn = on;
}

static void writeGoals() {
  uint8_t d[ODM_N_JOINTS * 2];
  for (uint8_t k = 0; k < nAct; k++) {
    uint16_t t = appliedTarget[actIdx[k]];
    d[2 * k] = t & 0xFF;
    d[2 * k + 1] = t >> 8;
  }
  bus.syncWrite(sts::REG_GOAL_POS, 2, actIds, nAct, d);
}

// Läser position, hastighet och last för aktiva leder till st, samt temperatur/spänning för ett servo.
static void readState() {
  uint8_t raw[ODM_N_JOINTS * 6], errs[ODM_N_JOINTS];
  uint16_t ok = bus.syncRead(sts::REG_PRESENT_POS, 6, actIds, nAct, raw, errs);
  servoErrorThisCycle = false;
  memset(st.pos, 0, sizeof(st.pos));
  memset(st.vel, 0, sizeof(st.vel));
  memset(st.load, 0, sizeof(st.load));
  for (uint8_t k = 0; k < nAct; k++) {
    uint8_t j = actIdx[k];
    if (!(ok & (1u << k))) {
      servoErrorThisCycle = true;
      continue;
    }
    const uint8_t* p = raw + 6 * k;
    uint16_t pos = p[0] | (p[1] << 8), spd = p[2] | (p[3] << 8), load = p[4] | (p[5] << 8);
    st.pos[j] = pos & 0x0FFF;
    st.vel[j] = (spd & 0x8000) ? -(int16_t)(spd & 0x7FFF) : (int16_t)spd;          // steg/s, bit 15 = tecken
    st.load[j] = (load & 0x0400) ? -(int16_t)(load & 0x03FF) : (int16_t)(load & 0x03FF);  // 0,1 %, bit 10 = tecken
  }
  if (nAct) {
    rr = (rr + 1) % nAct;
    uint8_t vt[2];
    int r = bus.read(actIds[rr], sts::REG_VOLTAGE, 2, vt);
    st.diag[0] = actIdx[rr];
    st.diag[1] = r == sts::OK ? vt[1] : 0;  // °C
    st.diag[2] = r == sts::OK ? vt[0] : 0;  // 0,1 V
    st.diag[3] = (uint8_t)r;
    if (r == sts::OK) st.bus_mv = vt[0] * 100;
    if (r != sts::OK) servoErrorThisCycle = true;
  }
}

static void sendState(uint16_t seqEcho) {
  static uint16_t seq = 0;
  st.seq_echo = seqEcho;
  st.t_ms = millis();
  uint8_t s = 0;
  if (wdLatched) s |= ODM_ST_WATCHDOG;
  if (st.bus_mv && st.bus_mv < param[ODM_PARAM_LOW_MV]) s |= ODM_ST_LOW_VOLTAGE;
  if (servoErrorThisCycle) s |= ODM_ST_SERVO_ERROR;
  if (estopLatched) s |= ODM_ST_ESTOP;
  if (torqueOn) s |= ODM_ST_TORQUE;
  if (fallLatched) s |= ODM_ST_FALL;
  if (imu.ok()) s |= ODM_ST_IMU_OK;
  st.status = s;
  for (uint8_t i = 0; i < 3; i++) {
    st.imu[i] = imu.ok() ? (int16_t)constrain(bodyGyro[i] * ODM_IMU_GYRO_SCALE, -32767, 32767) : 0;
    st.imu[3 + i] = imu.ok() ? (int16_t)constrain(bodyAcc[i] * ODM_IMU_ACC_SCALE, -32767, 32767) : 0;
  }
  st.feet = 0;
  for (uint8_t i = 0; i < 4; i++)
    if (digitalRead(footPins[i]) == LOW) st.feet |= (1u << i);
  odmFinish(&st, sizeof(st), ODM_TYPE_STATE, seq++);
  link.send(&st, sizeof(st));
  lastStateMs = millis();
}

static void updateFace() {
  if (wdLatched || estopLatched || fallLatched) showFace(F_ALARM);
  else if (servoErrorThisCycle) showFace(F_ERROR);
  else if (torqueOn) showFace(F_TORQUE);
  else showFace(F_IDLE);
}

// ---------------------------------------------------------------------------------------------- kommandon
static void enableTorqueFromPresent() {
  // starta från där servona står, så att första målet inte ger ett ryck
  readState();
  for (uint8_t k = 0; k < nAct; k++) appliedTarget[actIdx[k]] = st.pos[actIdx[k]];
  writeGoals();
  setTorque(true);
  fallArmed = false;  // reflexen blir beredd först när roboten står upprätt
}

// ---------------------------------------------------------------------------------------------- fallreflex
static void updateImu() {
  if (!imu.poll()) return;
  float n = 0;
  for (uint8_t i = 0; i < 3; i++) {
    bodyGyro[i] = imuSigns[i] * imu.gyro[imuAxes[i]];
    bodyAcc[i] = imuSigns[i] * imu.acc[imuAxes[i]];
    bodyUp[i] = imuSigns[i] * imu.grav[imuAxes[i]];
    n += bodyUp[i] * bodyUp[i];
  }
  n = sqrtf(n);
  if (n > 1.0f)
    for (uint8_t i = 0; i < 3; i++) bodyUp[i] /= n;
}

static void triggerFall(uint8_t dir) {
  fallLatched = true;
  fallLimp = false;
  fallMs = millis();
  for (uint8_t k = 0; k < nAct; k++) {
    uint8_t j = actIdx[k];
    uint16_t v = reflexPose[dir][j];
    if (v != ODM_REFLEX_KEEP) appliedTarget[j] = cmdTarget[j] = min(v, (uint16_t)4095);  // utan max_step
  }
  writeGoals();
}

// Lutning (grader) och lutningshastighet ger en prognos; samma regel som sim/fall_test.py --detect prognos.
static void checkFall() {
  uint16_t limit = param[ODM_PARAM_FALL_DEG];
  if (!imu.ok() || !torqueOn || fallLatched || limit == 0) return;
  float tilt = degrees(acosf(constrain(bodyUp[2], -1.0f, 1.0f)));
  if (!fallArmed) {
    fallArmed = tilt < 15;
    return;
  }
  float rate = degrees(sqrtf(bodyGyro[0] * bodyGyro[0] + bodyGyro[1] * bodyGyro[1]));
  if (tilt < 8 || tilt + rate * param[ODM_PARAM_FALL_HORIZON_MS] / 1000.0f < limit) return;
  // riktning: uppåtvektorns x < 0 = faller framåt, y < 0 = faller åt vänster
  uint8_t dir = fabsf(bodyUp[0]) >= fabsf(bodyUp[1]) ? (bodyUp[0] < 0 ? ODM_FALL_FWD : ODM_FALL_BACK)
                                                     : (bodyUp[1] < 0 ? ODM_FALL_LEFT : ODM_FALL_RIGHT);
  triggerFall(dir);
}

static void fallTimers() {
  uint16_t limp = param[ODM_PARAM_FALL_LIMP_MS];
  if (fallLatched && !fallLimp && limp && millis() - fallMs >= limp) {
    if (torqueOn) setTorque(false);  // skonar servona när roboten ligger
    fallLimp = true;
  }
}

static void handleCommand(const CommandPacket* c) {
  lastCmdMs = millis();
  lastSeq = c->seq;
  if (c->flags & ODM_FLAG_RESET) {
    wdLatched = estopLatched = false;
    fallLatched = fallLimp = fallArmed = false;
  }
  if (c->flags & ODM_FLAG_ESTOP) {
    estopLatched = true;
    if (torqueOn) setTorque(false);
  }
  bool want = (c->flags & ODM_FLAG_TORQUE) && !wdLatched && !estopLatched && !fallLimp;
  for (uint8_t j = 0; j < ODM_N_JOINTS; j++) cmdTarget[j] = c->targets[j] > 4095 ? 4095 : c->targets[j];
  if (want && !torqueOn) enableTorqueFromPresent();
  else if (!want && torqueOn) setTorque(false);
  if (torqueOn && !fallLatched) {  // under fallreflexen gäller reflexposen, inte kommandots mål
    int maxStep = param[ODM_PARAM_MAX_STEP];
    for (uint8_t k = 0; k < nAct; k++) {
      uint8_t j = actIdx[k];
      int d = (int)cmdTarget[j] - (int)appliedTarget[j];
      if (maxStep > 0) d = constrain(d, -maxStep, maxStep);
      appliedTarget[j] += d;
    }
    writeGoals();
  }
  for (uint8_t i = 0; i < 2; i++) {
    if (c->antennas[i] == antennaVal[i]) continue;
    antennaVal[i] = c->antennas[i];
    if (antennaVal[i] == ODM_ANTENNA_OFF) antenna[i].detach();
    else {
      if (!antenna[i].attached()) antenna[i].attach(antennaPins[i]);
      antenna[i].write(min(antennaVal[i], (uint8_t)180));
    }
  }
  readState();
  sendState(c->seq);
}

static void handleDiag(const DiagPacket* q) {
  DiagPacket r;
  memset(&r, 0, sizeof(r));
  r.op = q->op;
  r.id = q->id;
  r.addr = q->addr;
  r.len = q->len;
  r.status = ODM_DS_OK;
  switch (q->op) {
    case ODM_OP_PING: {
      uint16_t model = 0;
      r.status = bus.ping(q->id, &model);
      r.data[0] = model & 0xFF;
      r.data[1] = model >> 8;
      break;
    }
    case ODM_OP_READ:
      if (q->len == 0 || q->len > sizeof(r.data)) r.status = ODM_DS_BAD_ARG;
      else r.status = bus.read(q->id, q->addr, q->len, r.data);
      break;
    case ODM_OP_WRITE:
      if (torqueOn) r.status = ODM_DS_BUSY;
      else if (q->len == 0 || q->len > sizeof(q->data)) r.status = ODM_DS_BAD_ARG;
      else r.status = bus.write(q->id, q->addr, q->data, q->len);
      break;
    case ODM_OP_SCAN:
      if (torqueOn) {
        r.status = ODM_DS_BUSY;
        break;
      }
      for (uint16_t id = 0; id < 254; id++)
        if (bus.ping(id, nullptr, 800) != sts::TIMEOUT) r.data[id >> 3] |= (1u << (id & 7));
      break;
    case ODM_OP_SET_IDS:
      if (torqueOn) {
        r.status = ODM_DS_BUSY;
        break;
      }
      memcpy(jointId, q->data, ODM_N_JOINTS);
      probeJoints();
      [[fallthrough]];  // svara med tabellen
    case ODM_OP_GET_IDS:
      memcpy(r.data, jointId, ODM_N_JOINTS);
      r.data[14] = activeMask & 0xFF;
      r.data[15] = activeMask >> 8;
      break;
    case ODM_OP_SET_PARAM:
      if (q->addr >= ODM_N_PARAMS) r.status = ODM_DS_BAD_ARG;
      else param[q->addr] = q->data[0] | (q->data[1] << 8);
      break;
    case ODM_OP_SET_REFLEX:
    case ODM_OP_GET_REFLEX:
      if (q->addr >= ODM_N_FALL_DIRS) {
        r.status = ODM_DS_BAD_ARG;
        break;
      }
      if (q->op == ODM_OP_SET_REFLEX)
        for (uint8_t j = 0; j < ODM_N_JOINTS; j++) reflexPose[q->addr][j] = q->data[2 * j] | (q->data[2 * j + 1] << 8);
      for (uint8_t j = 0; j < ODM_N_JOINTS; j++) {
        r.data[2 * j] = reflexPose[q->addr][j] & 0xFF;
        r.data[2 * j + 1] = reflexPose[q->addr][j] >> 8;
      }
      break;
    case ODM_OP_GET_INFO:
      r.data[0] = ODM_FW_VERSION & 0xFF;
      r.data[1] = ODM_FW_VERSION >> 8;
      for (uint8_t i = 0; i < ODM_N_PARAMS; i++) {
        r.data[2 + 2 * i] = param[i] & 0xFF;
        r.data[3 + 2 * i] = param[i] >> 8;
      }
      r.data[2 + 2 * ODM_N_PARAMS] = bus.echo();
      break;
    default:
      r.status = ODM_DS_BAD_ARG;
  }
  odmFinish(&r, sizeof(r), ODM_TYPE_DIAG, q->seq);
  link.send(&r, sizeof(r));
}

// ---------------------------------------------------------------------------------------------- start och loop
void setup() {
  matrix.begin();
  showFace(F_BOOT);
  link.begin();
  for (uint8_t i = 0; i < 4; i++) pinMode(footPins[i], INPUT_PULLUP);
  memcpy(param, ODM_PARAM_DEFAULTS, sizeof(param));
  memcpy(jointId, ODM_DEFAULT_IDS, sizeof(jointId));
  memset(&st, 0, sizeof(st));
  for (uint8_t d = 0; d < ODM_N_FALL_DIRS; d++)
    for (uint8_t j = 0; j < ODM_N_JOINTS; j++) reflexPose[d][j] = ODM_REFLEX_KEEP;
  imu.begin(IMU_WIRE, IMU_ADDR, IMU_INTERVAL_US);  // utan sensor: ST_IMU_OK saknas, reflexen gör inget
  bus.begin(Serial1, BUS_BAUD);
  delay(300);  // servona startar
  bus.detectEcho();
  probeJoints();
  showFace(F_IDLE);
}

void loop() {
  uint8_t type;
  const uint8_t* pkt;
  while ((pkt = link.poll(&type)) != nullptr) {
    if (type == ODM_TYPE_COMMAND) handleCommand((const CommandPacket*)pkt);
    else if (type == ODM_TYPE_DIAG) handleDiag((const DiagPacket*)pkt);
  }
  updateImu();
  checkFall();
  fallTimers();
  uint32_t now = millis();
  if (torqueOn && now - lastCmdMs > param[ODM_PARAM_WATCHDOG_MS]) {
    setTorque(false);
    wdLatched = true;
  }
  if (now - lastStateMs >= IDLE_STATE_MS && now - lastCmdMs >= IDLE_STATE_MS) {
    readState();
    sendState(lastSeq);
  }
  updateFace();
}
