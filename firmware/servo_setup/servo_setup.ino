// Open Duck Mini – inställning av servon via seriell terminal (PuTTY, Arduino IDE:s seriella monitor m.fl.).
//
// Hittar alla STS3215 på bussen, visar vilken plats (led) varje ID hör till enligt Open Ducks numrering,
// byter ID ett servo i taget, nollställer inställningar från tidigare användning (t.ex. LeRobot/SO-101)
// och kör servona till nolläget (rå 2048) så att horn och delar kan monteras i rätt läge.
//
// Koppling som för huvudfirmware (firmware/body_uno): Serial1 (stift 0/1) -> Waveshare-adapter i läge B.
// Terminal: 115200 baud, 8N1, UTF-8. Skriv "hjalp" för kommandon.
#include "feetech.h"
#include "protocol.h"

#define TERM_BAUD 115200
#define BUS_BAUD 1000000
#define ZERO_POS 2048
#define SAFE_SPEED 400   // steg/s vid noll/flytta (ca 35 °/s)
#define SAFE_ACC 20

static sts::Bus bus;

// ---------------------------------------------------------------------------------------------- terminal
static char line[64];

// Läser en rad med eko och backsteg. Returnerar false om ingen rad finns än (anropas i loop).
static bool readLine() {
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (c == '\n' && n == 0) continue;  // CR LF
      Serial.print("\r\n");
      line[n] = 0;
      n = 0;
      return true;
    }
    if ((c == 8 || c == 127) && n) {
      n--;
      Serial.print("\b \b");
    } else if (c >= 32 && n < sizeof(line) - 1) {
      line[n++] = c;
      Serial.print(c);
    }
  }
  return false;
}

// Väntar på en hel rad (blockerande).
static const char* ask(const char* prompt) {
  Serial.print(prompt);
  while (!readLine()) {
  }
  return line;
}

static bool confirm(const char* what) {
  Serial.print(what);
  const char* a = ask(" Skriv JA för att fortsätta: ");
  return strcmp(a, "JA") == 0 || strcmp(a, "ja") == 0;
}

static void prompt() { Serial.print("\r\nservo> "); }

// ---------------------------------------------------------------------------------------------- tabellen
static int jointForId(int id) {
  for (int j = 0; j < ODM_N_JOINTS; j++)
    if (ODM_DEFAULT_IDS[j] == id) return j;
  return -1;
}

// Lednamn ("left_knee") eller, om places är sant, platsnummer 1–14. Returnerar led-index eller -1.
static int parseJoint(const char* s, bool places) {
  for (int j = 0; j < ODM_N_JOINTS; j++)
    if (strcmp(s, ODM_JOINT_NAMES[j]) == 0) return j;
  int v = atoi(s);
  if (places && v >= 1 && v <= ODM_N_JOINTS) return v - 1;
  return -1;
}

static void printJoint(int j) {
  Serial.print(ODM_JOINT_LABELS[j]);
  Serial.print(" (");
  Serial.print(ODM_JOINT_NAMES[j]);
  Serial.print(")");
}

static void printLabel(int id) {
  int j = jointForId(id);
  Serial.print("  +--------------------------------+\r\n  | ETIKETT: ID ");
  Serial.print(id);
  Serial.print(id < 10 ? "  " : " ");
  if (j >= 0) {
    Serial.print(" ");
    Serial.print(ODM_JOINT_LABELS[j]);
  }
  Serial.print("\r\n  | ");
  Serial.print(j >= 0 ? ODM_JOINT_NAMES[j] : "(ingen plats i Open Duck)");
  Serial.print("\r\n  +--------------------------------+\r\n");
  Serial.print("  Skriv också utväxlingen från servots dekal (C001 = 1/345, C044 = 1/191, C046 = 1/147).\r\n");
}

// ---------------------------------------------------------------------------------------------- servon
static uint8_t found[254];
static int nFound = 0;

static void scanBus(bool quiet) {
  nFound = 0;
  for (int id = 0; id < 254; id++)
    if (bus.ping(id, nullptr, 800) != sts::TIMEOUT) found[nFound++] = id;
  if (!quiet) {
    Serial.print("Hittade ");
    Serial.print(nFound);
    Serial.print(" servon.\r\n");
  }
}

static int readU16(int id, uint8_t addr) {
  uint8_t b[2];
  return bus.read(id, addr, 2, b) == sts::OK ? (b[0] | (b[1] << 8)) : -1;
}

static int readU8(int id, uint8_t addr) {
  uint8_t b;
  return bus.read(id, addr, 1, &b) == sts::OK ? b : -1;
}

static void printServoLine(int id) {
  uint16_t model = 0;
  bus.ping(id, &model);
  uint8_t vt[2] = {0, 0};
  bus.read(id, sts::REG_VOLTAGE, 2, vt);
  char buf[96];
  snprintf(buf, sizeof(buf), "  ID %3d  modell %4u  pos %4d  %2d.%d V  %2d C  ", id, model,
           readU16(id, sts::REG_PRESENT_POS), vt[0] / 10, vt[0] % 10, vt[1]);
  Serial.print(buf);
  int j = jointForId(id);
  if (j >= 0) printJoint(j);
  else Serial.print("-> inte i Open Ducks tabell, byt ID");
  Serial.print("\r\n");
}

static bool writeEeprom(int id, uint8_t addr, const uint8_t* data, uint8_t len) {
  uint8_t z = 0, one = 1;
  if (bus.write(id, sts::REG_LOCK, &z, 1) != sts::OK) return false;
  bool ok = bus.write(id, addr, data, len) == sts::OK;
  int newId = (addr == sts::REG_ID) ? data[0] : id;
  delay(5);
  bus.write(newId, sts::REG_LOCK, &one, 1);
  return ok;
}

static bool changeId(int oldId, int newId) {
  if (bus.ping(oldId, nullptr) == sts::TIMEOUT) {
    Serial.print("Inget servo svarar på ID ");
    Serial.print(oldId);
    Serial.print(".\r\n");
    return false;
  }
  if (oldId != newId && bus.ping(newId, nullptr) != sts::TIMEOUT) {
    Serial.print("ID ");
    Serial.print(newId);
    Serial.print(" är redan upptaget på bussen.\r\n");
    return false;
  }
  uint8_t v = newId;
  writeEeprom(oldId, sts::REG_ID, &v, 1);
  delay(20);
  bool ok = bus.ping(newId, nullptr) != sts::TIMEOUT;
  Serial.print(ok ? "ID ändrat: " : "MISSLYCKADES: ");
  Serial.print(oldId);
  Serial.print(" -> ");
  Serial.print(newId);
  Serial.print("\r\n");
  return ok;
}

// Positionsläge, P32/I0/D0 (som Open Ducks configure_motor), vinkelgränser 0..4095 och offset 0.
// SO-101-servon har ofta LeRobots gränser och offset kvar; utan detta stämmer inte 2048 = mitten.
static bool configure(int id) {
  uint8_t mode = 0, pid[3] = {32, 0, 0}, lim[4] = {0, 0, 0xFF, 0x0F}, ofs[2] = {0, 0};
  bool ok = writeEeprom(id, sts::REG_MODE, &mode, 1) && writeEeprom(id, sts::REG_P, pid, 3) &&
            writeEeprom(id, sts::REG_MIN_ANGLE, lim, 4) && writeEeprom(id, sts::REG_OFFSET, ofs, 2);
  Serial.print("  ID ");
  Serial.print(id);
  Serial.print(ok ? ": positionsläge, P 32, gränser 0..4095, offset 0\r\n" : ": KONFIGURATION MISSLYCKADES\r\n");
  return ok;
}

static void release(int id) {
  uint8_t off = 0;
  bus.write(id, sts::REG_TORQUE, &off, 1);
  // acceleration och hastighet tillbaka till 0 (= max), som huvudfirmware förutsätter; målet = där servot står
  int p = max(readU16(id, sts::REG_PRESENT_POS), 0);
  uint8_t fast[7] = {0, (uint8_t)(p & 0xFF), (uint8_t)(p >> 8), 0, 0, 0, 0};
  bus.write(id, sts::REG_ACC, fast, 7);
}

// Långsam rörelse till pos för en lista av servon, väntar tills de är framme (högst 8 s).
static void moveSlow(const uint8_t* ids, int n, int pos) {
  uint8_t d[7] = {SAFE_ACC, (uint8_t)(pos & 0xFF), (uint8_t)(pos >> 8), 0, 0, SAFE_SPEED & 0xFF, SAFE_SPEED >> 8};
  uint8_t on = 1;
  for (int i = 0; i < n; i++) {
    bus.write(ids[i], sts::REG_ACC, d, 7);
    bus.write(ids[i], sts::REG_TORQUE, &on, 1);
  }
  uint32_t t0 = millis();
  for (;;) {
    int worst = 0, worstId = -1;
    for (int i = 0; i < n; i++) {
      int p = readU16(ids[i], sts::REG_PRESENT_POS);
      int e = p < 0 ? 9999 : abs(p - pos);
      if (e > worst) worst = e, worstId = ids[i];
    }
    if (worst <= 8) {
      Serial.print("  framme (största fel ");
      Serial.print(worst);
      Serial.print(" steg)\r\n");
      return;
    }
    if (millis() - t0 > 8000) {
      Serial.print("  INTE FRAMME efter 8 s: ID ");
      Serial.print(worstId);
      Serial.print(" fel ");
      Serial.print(worst);
      Serial.print(" steg (fastnat, för låg spänning eller vinkelgräns?)\r\n");
      return;
    }
    delay(50);
  }
}

// "alla" eller ett ID -> lista. Returnerar antal.
static int targetList(const char* arg, uint8_t* ids) {
  if (!arg || !*arg || strcmp(arg, "alla") == 0) {
    scanBus(true);
    memcpy(ids, found, nFound);
    return nFound;
  }
  ids[0] = atoi(arg);
  if (bus.ping(ids[0], nullptr) == sts::TIMEOUT) {
    Serial.print("Inget servo svarar på ID ");
    Serial.print(ids[0]);
    Serial.print(".\r\n");
    return 0;
  }
  return 1;
}

// Servots egen offset (register 31–32): belopp i bit 0–10, bit 11 = negativt.
static int offsetSteps(int id) {
  int r = readU16(id, sts::REG_OFFSET);
  if (r < 0) return 0;
  return (r & 0x800) ? -(r & 0x7FF) : (r & 0x7FF);
}

// Utväxlingen syns inte i positionen (givaren sitter på utgående axeln), men i tomgångsfarten.
// Nominellt vid 7,4 V: 1/345 ca 3070 steg/s (0,222 s/60°), 1/191 och 1/147 skalade med utväxlingen (verifiera).
// Samma tabell som python/src/openhead_body/servo.py.
static const char* GEAR_NAMES[3] = {"1/345 (C001)", "1/191 (C044)", "1/147 (C046)"};
static const float GEAR_SPEED[3] = {3070.0f, 3070.0f * 345 / 191, 3070.0f * 345 / 147};

static void cmdGear(int id) {
  if (bus.ping(id, nullptr) == sts::TIMEOUT) {
    Serial.print("Inget svar.\r\n");
    return;
  }
  if (!confirm("Servot kör ett halvt varv med full fart. Det måste sitta LÖST, utan horn eller delar.")) return;
  uint8_t vt[2] = {0, 0};
  bus.read(id, sts::REG_VOLTAGE, 2, vt);
  int p0 = readU16(id, sts::REG_PRESENT_POS);
  if (p0 < 0) return;
  const int travel = 1536;  // 135°
  int goal = p0 < 2048 ? p0 + travel : p0 - travel;
  int a = p0 + (goal - p0) / 5, b = p0 + (goal - p0) * 4 / 5;  // mät mellan 20 % och 80 %
  uint8_t d[7] = {0, (uint8_t)(goal & 0xFF), (uint8_t)(goal >> 8), 0, 0, 0, 0};  // acc 0, fart 0 = max
  uint8_t on = 1;
  bus.write(id, sts::REG_ACC, d, 7);
  bus.write(id, sts::REG_TORQUE, &on, 1);
  uint32_t t0 = micros(), ta = 0, tb = 0;
  while (micros() - t0 < 2000000UL && !tb) {
    int p = readU16(id, sts::REG_PRESENT_POS);
    if (p < 0) continue;
    bool pastA = goal > p0 ? p >= a : p <= a, pastB = goal > p0 ? p >= b : p <= b;
    if (pastA && !ta) ta = micros();
    if (pastB && ta) tb = micros();
  }
  uint8_t one[1] = {(uint8_t)id};
  moveSlow(one, 1, p0);
  release(id);
  if (!ta || !tb || tb <= ta) {
    Serial.print("  Rörelsen nådde inte fram. Fastnat, för låg spänning eller vinkelgräns? Kör konfig.\r\n");
    return;
  }
  float v = abs(b - a) * 1e6f / (tb - ta);
  float v74 = vt[0] ? v * 74.0f / vt[0] : v;
  int best = 0;
  for (int k = 1; k < 3; k++)
    if (fabsf(v74 / GEAR_SPEED[k] - 1) < fabsf(v74 / GEAR_SPEED[best] - 1)) best = k;
  char buf[140];
  snprintf(buf, sizeof(buf), "  %d steg/s vid %d.%d V (%d steg/s vid 7,4 V) -> närmast %s, %d %% av nominellt\r\n",
           (int)v, vt[0] / 10, vt[0] % 10, (int)v74, GEAR_NAMES[best], (int)(100 * v74 / GEAR_SPEED[best]));
  Serial.print(buf);
  float r = v74 / GEAR_SPEED[best];
  if (r < 0.8f || r > 1.25f) Serial.print("  Osäkert: långt från alla nominella värden.\r\n");
  Serial.print("  Dekalen är förstahandskällan; testet är en kontroll. Skriv utväxlingen på etiketten.\r\n");
}

// ---------------------------------------------------------------------------------------------- kommandon
static void cmdHelp() {
  Serial.print(
      "\r\nKommandon (skriv och tryck Enter):\r\n"
      "  scan                 hitta alla servon, visa ID, position, spänning och plats\r\n"
      "  tabell               Open Ducks platser och ID, vilka som hittats och vilka som saknas\r\n"
      "  guide                märk servon ett i taget: välj plats, sätt ID, konfigurera, kör till noll\r\n"
      "  id <gammalt> <nytt>  byt ID. Nytt = ID-nummer eller lednamn (t.ex. id 1 left_knee ger ID 23)\r\n"
      "  konfig <id|alla>     positionsläge, P32, vinkelgränser 0..4095, offset 0\r\n"
      "  noll <id|alla>       kör långsamt till nolläge (2048) och håll\r\n"
      "  flytta <id> <pos>    kör långsamt till rå position 0..4095\r\n"
      "  av [id|alla]         släpp momentet\r\n"
      "  visa                 positioner löpande, Enter stoppar\r\n"
      "  info <id>            servots register\r\n"
      "  vaxel <id>           hastighetstest av utväxlingen (servot löst, kör ett halvt varv)\r\n"
      "  etikett <id>         skriv ut etiketten för ett ID\r\n"
      "  hjalp                den här listan\r\n");
}

static void cmdScan() {
  scanBus(false);
  for (int i = 0; i < nFound; i++) printServoLine(found[i]);
  int ones = 0;
  for (int i = 0; i < nFound; i++)
    if (jointForId(found[i]) < 0) ones++;
  if (ones) Serial.print("Servon utan plats behöver nytt ID: använd guide eller id.\r\n");
  Serial.print("Obs: två servon med samma ID (t.ex. nya med ID 1) syns som ett eller inte alls. Byt ID med ett servo i taget.\r\n");
}

// Antal tecken (inte byte) i en UTF-8-sträng, för kolumnjustering
static int utf8Len(const char* s) {
  int n = 0;
  for (; *s; s++) n += (*s & 0xC0) != 0x80;
  return n;
}

static void cmdTable() {
  scanBus(true);
  Serial.print("  nr  plats                           ID   status\r\n");
  int missing = 0;
  for (int j = 0; j < ODM_N_JOINTS; j++) {
    bool ok = false;
    for (int i = 0; i < nFound; i++) ok |= found[i] == ODM_DEFAULT_IDS[j];
    char buf[24];
    snprintf(buf, sizeof(buf), "  %2d  ", j + 1);
    Serial.print(buf);
    printJoint(j);
    int len = utf8Len(ODM_JOINT_LABELS[j]) + strlen(ODM_JOINT_NAMES[j]) + 3;
    for (int k = len; k < 31; k++) Serial.print(' ');
    snprintf(buf, sizeof(buf), "%3d   ", ODM_DEFAULT_IDS[j]);
    Serial.print(buf);
    Serial.print(ok ? "hittad\r\n" : "saknas\r\n");
    missing += !ok;
  }
  Serial.print("Okända ID på bussen:");
  int unk = 0;
  for (int i = 0; i < nFound; i++)
    if (jointForId(found[i]) < 0) {
      Serial.print(' ');
      Serial.print(found[i]);
      unk++;
    }
  Serial.print(unk ? "\r\n" : " inga\r\n");
  Serial.print(missing ? "" : "Alla 14 platser har sitt servo.\r\n");
}

static void cmdGuide() {
  Serial.print("\r\nGUIDE: koppla in ETT servo i taget på bussen. q avbryter.\r\n");
  for (;;) {
    const char* a = ask("\r\nKoppla in nästa servo och tryck Enter (q = klar): ");
    if (a[0] == 'q') return;
    scanBus(true);
    if (nFound == 0) {
      Serial.print("Inget servo svarar. Kontrollera kabeln och matningen (>= 6 V).\r\n");
      continue;
    }
    if (nFound > 1) {
      Serial.print("Flera servon svarar (");
      for (int i = 0; i < nFound; i++) {
        Serial.print(found[i]);
        Serial.print(i + 1 < nFound ? ", " : "");
      }
      Serial.print("). Koppla bara in ett.\r\n");
      continue;
    }
    int id = found[0];
    printServoLine(id);
    int j = jointForId(id);
    if (j >= 0) {
      Serial.print("Servot har redan plats: ");
      printJoint(j);
      Serial.print("\r\n");
    }
    Serial.print("Platser:\r\n");
    for (int k = 0; k < ODM_N_JOINTS; k++) {
      char buf[16];
      snprintf(buf, sizeof(buf), "  %2d  ID %2d  ", k + 1, ODM_DEFAULT_IDS[k]);
      Serial.print(buf);
      printJoint(k);
      Serial.print("\r\n");
    }
    a = ask("Vilken plats (1-14, Enter = behåll ID): ");
    int target = id;
    if (*a) {
      int k = parseJoint(a, true);
      if (k < 0) {
        Serial.print("Okänd plats.\r\n");
        continue;
      }
      target = ODM_DEFAULT_IDS[k];
    }
    if (target != id && !changeId(id, target)) continue;
    configure(target);
    a = ask("Hastighetstest av utväxlingen? Bara med löst servo (j/N): ");
    if (a[0] == 'j' || a[0] == 'J') cmdGear(target);
    Serial.print("Kör till nolläge (2048)...\r\n");
    uint8_t one[1] = {(uint8_t)target};
    moveSlow(one, 1, ZERO_POS);
    printLabel(target);
    ask("Montera hornet/delen i nolläget nu om du vill. Enter släpper momentet: ");
    release(target);
  }
}

static void cmdId(char* args) {
  char* a = strtok(args, " ");
  char* b = strtok(nullptr, " ");
  if (!a || !b) {
    Serial.print("Användning: id <gammalt> <nytt ID | lednamn>\r\n");
    return;
  }
  int oldId = atoi(a), newId;
  int k = parseJoint(b, false);  // tal = nytt ID, annars lednamn
  newId = k >= 0 ? ODM_DEFAULT_IDS[k] : atoi(b);
  if (newId < 0 || newId > 253) {
    Serial.print("Ogiltigt ID.\r\n");
    return;
  }
  if (changeId(oldId, newId)) printLabel(newId);
}

static void cmdInfo(int id) {
  if (bus.ping(id, nullptr) == sts::TIMEOUT) {
    Serial.print("Inget svar.\r\n");
    return;
  }
  printServoLine(id);
  char buf[128];
  snprintf(buf, sizeof(buf),
           "  läge %d, P %d I %d D %d, vinkelgränser %d..%d, offset %d, moment %d, acc %d, målhastighet %d\r\n",
           readU8(id, sts::REG_MODE), readU8(id, sts::REG_P), readU8(id, sts::REG_I), readU8(id, sts::REG_D),
           readU16(id, sts::REG_MIN_ANGLE), readU16(id, sts::REG_MAX_ANGLE), offsetSteps(id),
           readU8(id, sts::REG_TORQUE), readU8(id, sts::REG_ACC), readU16(id, sts::REG_GOAL_SPEED));
  Serial.print(buf);
  int lim0 = readU16(id, sts::REG_MIN_ANGLE), lim1 = readU16(id, sts::REG_MAX_ANGLE);
  if (lim0 != 0 || lim1 != 4095 || offsetSteps(id) != 0)
    Serial.print("  Gränser/offset avviker från standard: kör konfig.\r\n");
}

static void cmdWatch() {
  scanBus(true);
  Serial.print("Positioner (Enter stoppar):\r\n");
  while (!Serial.available()) {
    for (int i = 0; i < nFound; i++) {
      char buf[20];
      snprintf(buf, sizeof(buf), "%d:%4d  ", found[i], readU16(found[i], sts::REG_PRESENT_POS));
      Serial.print(buf);
    }
    Serial.print("\r\n");
    delay(250);
  }
  while (Serial.available()) Serial.read();
}

static void handle(char* l) {
  char* cmd = strtok(l, " ");
  if (!cmd) return;
  char* rest = strtok(nullptr, "");
  uint8_t ids[254];
  if (!strcmp(cmd, "hjalp") || !strcmp(cmd, "help") || !strcmp(cmd, "?")) cmdHelp();
  else if (!strcmp(cmd, "scan")) cmdScan();
  else if (!strcmp(cmd, "tabell")) cmdTable();
  else if (!strcmp(cmd, "guide")) cmdGuide();
  else if (!strcmp(cmd, "id")) cmdId(rest ? rest : (char*)"");
  else if (!strcmp(cmd, "info") && rest) cmdInfo(atoi(rest));
  else if (!strcmp(cmd, "etikett") && rest) printLabel(atoi(rest));
  else if (!strcmp(cmd, "vaxel") && rest) cmdGear(atoi(rest));
  else if (!strcmp(cmd, "visa")) cmdWatch();
  else if (!strcmp(cmd, "konfig")) {
    int n = targetList(rest, ids);
    for (int i = 0; i < n; i++) configure(ids[i]);
  } else if (!strcmp(cmd, "noll") || !strcmp(cmd, "flytta")) {
    int pos = ZERO_POS;
    char* idArg = rest;
    if (!strcmp(cmd, "flytta")) {
      idArg = rest ? strtok(rest, " ") : nullptr;
      char* p = idArg ? strtok(nullptr, " ") : nullptr;
      if (!idArg || !p || !strcmp(idArg, "alla")) {
        Serial.print("Användning: flytta <id> <pos 0..4095>\r\n");
        return;
      }
      pos = constrain(atoi(p), 0, 4095);
    }
    int n = targetList(idArg, ids);
    if (!n) return;
    if (n > 1 && !confirm("Alla servon kör till nolläge. Sitter de i roboten kan delar slå i varandra.")) return;
    Serial.print("Kör ");
    Serial.print(n);
    Serial.print(" servon till ");
    Serial.print(pos);
    Serial.print("...\r\n");
    moveSlow(ids, n, pos);
    Serial.print("Momentet är på. Skriv av för att släppa.\r\n");
  } else if (!strcmp(cmd, "av")) {
    int n = targetList(rest, ids);
    for (int i = 0; i < n; i++) release(ids[i]);
    Serial.print("Momentet av.\r\n");
  } else {
    Serial.print("Okänt kommando. Skriv hjalp.\r\n");
  }
}

void setup() {
  Serial.begin(TERM_BAUD);
  bus.begin(Serial1, BUS_BAUD);
  delay(300);
  bus.detectEcho();
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
  }
  Serial.print("\r\nOpen Duck Mini – servoinställning. Buss 1 Mbps");
  Serial.print(bus.echo() ? " (adaptern ekar, hanteras)" : "");
  Serial.print(".\r\n");
  cmdHelp();
  prompt();
}

void loop() {
  if (readLine()) {
    handle(line);
    prompt();
  }
}
