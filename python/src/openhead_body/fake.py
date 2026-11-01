"""
Simulerad Uno med servon, för test av datorverktyget utan hårdvara (--port fake).

Följer firmwarens regler (firmware/body_uno/body_uno.ino): moment slås på från nuvarande position,
målen begränsas till max_step per kommando, watchdog stänger av momentet och låser tills reset,
nödstopp låser, WRITE/SCAN/SET_IDS nekas med momentet på, fallreflexen (imu_up/imu_gyro sätts av testet). Servona rör sig mot målet med begränsad
hastighet. Tidsstyrt av perf_counter, så watchdog och dynamik går i verklig tid.
"""
import math
import struct
import time

from . import protocol as P
from .link import REG_ID, REG_LOCK, REG_MAX_ANGLE, REG_MODEL, REG_PRESENT_POS, REG_TEMPERATURE, REG_VOLTAGE

STS3215_MODEL = 777
MAX_SPEED = 3000.0      # steg/s (ca 4,6 rad/s)


class FakeServo:
    def __init__(self, sid, pos=2048):
        self.regs = bytearray(80)
        self.regs[REG_ID] = sid
        struct.pack_into("<H", self.regs, REG_MODEL, STS3215_MODEL)
        struct.pack_into("<H", self.regs, REG_MAX_ANGLE, 4095)   # fabriksvärde, min = 0
        self.regs[21] = 32                                       # P (fabriksvärde)
        struct.pack_into("<HH", self.regs, 16, 1000, 0)          # maxmoment (EEPROM)
        struct.pack_into("<H", self.regs, 48, 1000)              # momentgräns (RAM)
        self.regs[REG_LOCK] = 1
        self.regs[REG_VOLTAGE] = 74
        self.regs[REG_TEMPERATURE] = 28
        self.pos = float(pos)
        self.goal = float(pos)
        self.torque = False
        self.speed = 0.0

    @property
    def id(self):
        return self.regs[REG_ID]

    def step(self, dt):
        if self.torque:
            d = self.goal - self.pos
            v = max(-MAX_SPEED, min(MAX_SPEED, d / max(dt, 1e-3) * 0.5))
            self.pos += v * dt
            self.speed = v
        else:
            self.speed = 0.0


class FakeUno:
    """Ser ut som en pyserial-port: write(), read(n), in_waiting, close()."""

    def __init__(self, ids=(20, 21), watchdog_ms=150, max_step=40, echo=False):
        self.servos = {i: FakeServo(i, 2048 + 50 * k) for k, i in enumerate(ids)}
        self.param = list(P.PARAM_DEFAULTS)
        self.param[P.PARAM_WATCHDOG_MS] = watchdog_ms
        self.param[P.PARAM_MAX_STEP] = max_step
        self.joint_ids = list(P.DEFAULT_IDS)
        self.out = bytearray()
        self.parser = P.StreamParser()
        self.torque = self.wd = self.estop = False
        self.applied = [0] * P.N_JOINTS
        self.last_cmd = self.last_state = self.t = time.perf_counter()
        self.last_seq = 0
        self.state_seq = 0
        self.rr = 0
        self.t0 = self.t
        self.echo = echo
        # BNO085 i kroppen (robotens ram). Tester ändrar imu_up/imu_gyro för att simulera ett fall.
        self.imu_ok = True
        self.imu_up = [0.0, 0.0, 1.0]      # uppåt i robotens ram (enhetsvektor)
        self.imu_gyro = [0.0, 0.0, 0.0]    # rad/s
        self.reflex = [[P.REFLEX_KEEP] * P.N_JOINTS for _ in range(P.N_FALL_DIRS)]
        self.fall = self.fall_armed = self.fall_limp = False
        self.fall_t = 0.0
        self.probe()

    # ---------------------------------------------------------------- "port"
    @property
    def in_waiting(self):
        self.tick()
        return len(self.out)

    def read(self, n):
        self.tick()
        data, self.out = bytes(self.out[:n]), self.out[n:]
        return data

    def write(self, data):
        self.tick()
        for pkt in self.parser.feed(data):
            if pkt["type"] == P.TYPE_COMMAND:
                self.on_command(pkt)
            elif pkt["type"] == P.TYPE_DIAG:
                self.on_diag(pkt)
        return len(data)

    def close(self):
        pass

    # ---------------------------------------------------------------- firmwarelogik
    def probe(self):
        self.active = [j for j, i in enumerate(self.joint_ids) if i in self.servos]

    def tick(self):
        now = time.perf_counter()
        dt, self.t = now - self.t, now
        for s in self.servos.values():
            s.step(dt)
        if self.torque and (now - self.last_cmd) * 1000 > self.param[P.PARAM_WATCHDOG_MS]:
            self.set_torque(False)
            self.wd = True
        self.check_fall()
        limp = self.param[P.PARAM_FALL_LIMP_MS]
        if self.fall and not self.fall_limp and limp and (now - self.fall_t) * 1000 >= limp:
            if self.torque:
                self.set_torque(False)
            self.fall_limp = True
        if now - self.last_state >= 0.1 and now - self.last_cmd >= 0.1:
            self.send_state(self.last_seq)

    def check_fall(self):
        """Samma regel som checkFall() i firmware (prognos av lutningen, beredd först efter upprätt läge)."""
        limit = self.param[P.PARAM_FALL_DEG]
        if not self.imu_ok or not self.torque or self.fall or limit == 0:
            return
        up = self.imu_up
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, up[2]))))
        if not self.fall_armed:
            self.fall_armed = tilt < 15
            return
        rate = math.degrees(math.hypot(self.imu_gyro[0], self.imu_gyro[1]))
        if tilt < 8 or tilt + rate * self.param[P.PARAM_FALL_HORIZON_MS] / 1000 < limit:
            return
        if abs(up[0]) >= abs(up[1]):
            d = P.FALL_FWD if up[0] < 0 else P.FALL_BACK
        else:
            d = P.FALL_LEFT if up[1] < 0 else P.FALL_RIGHT
        self.fall, self.fall_limp, self.fall_t = True, False, time.perf_counter()
        self.fall_dir = d
        for j in self.active:
            v = self.reflex[d][j]
            if v != P.REFLEX_KEEP:
                self.applied[j] = min(v, 4095)
                self.servo(j).goal = self.applied[j]

    def servo(self, j):
        return self.servos[self.joint_ids[j]]

    def set_torque(self, on):
        for j in self.active:
            self.servo(j).torque = on
        self.torque = on

    def on_command(self, c):
        self.last_cmd = time.perf_counter()
        self.last_seq = c["seq"]
        if c["flags"] & P.FLAG_RESET:
            self.wd = self.estop = False
            self.fall = self.fall_limp = self.fall_armed = False
        if c["flags"] & P.FLAG_ESTOP:
            self.estop = True
            if self.torque:
                self.set_torque(False)
        want = bool(c["flags"] & P.FLAG_TORQUE) and not self.wd and not self.estop and not self.fall_limp
        if want and not self.torque:
            for j in self.active:
                self.applied[j] = int(round(self.servo(j).pos))
                self.servo(j).goal = self.applied[j]
            self.set_torque(True)
            self.fall_armed = False
        elif not want and self.torque:
            self.set_torque(False)
        self.check_fall()
        if self.torque and not self.fall:     # under fallreflexen gäller reflexposen
            m = self.param[P.PARAM_MAX_STEP]
            for j in self.active:
                d = min(c["targets"][j], 4095) - self.applied[j]
                if m:
                    d = max(-m, min(m, d))
                self.applied[j] += d
                self.servo(j).goal = self.applied[j]
        self.send_state(c["seq"])

    def send_state(self, seq_echo):
        pos, vel, load = [0] * P.N_JOINTS, [0] * P.N_JOINTS, [0] * P.N_JOINTS
        for j in self.active:
            s = self.servo(j)
            pos[j] = int(round(s.pos)) & 0x0FFF
            vel[j] = int(s.speed)
        diag, bus_mv = [0, 0, 0, 0], 0
        if self.active:
            self.rr = (self.rr + 1) % len(self.active)
            s = self.servo(self.active[self.rr])
            diag = [self.active[self.rr], s.regs[REG_TEMPERATURE], s.regs[REG_VOLTAGE], 0]
            bus_mv = s.regs[REG_VOLTAGE] * 100
        status = (P.ST_WATCHDOG if self.wd else 0) | (P.ST_ESTOP if self.estop else 0) | (P.ST_TORQUE if self.torque else 0)
        status |= (P.ST_FALL if self.fall else 0) | (P.ST_IMU_OK if self.imu_ok else 0)
        if bus_mv and bus_mv < self.param[P.PARAM_LOW_MV]:
            status |= P.ST_LOW_VOLTAGE
        imu = [0] * 6
        if self.imu_ok:
            imu = [round(g * P.IMU_GYRO_SCALE) for g in self.imu_gyro] + \
                  [round(u * 9.81 * P.IMU_ACC_SCALE) for u in self.imu_up]
        self.out += P.pack(P.TYPE_STATE, self.state_seq, seq_echo=seq_echo,
                           t_ms=int((time.perf_counter() - self.t0) * 1000) & 0xFFFFFFFF, status=status,
                           pos=pos, vel=vel, load=load, bus_mv=bus_mv, diag=diag, imu=imu)
        self.state_seq += 1
        self.last_state = time.perf_counter()

    def on_diag(self, q):
        op, sid, addr, n = q["op"], q["id"], q["addr"], q["len"]
        data, status = bytearray(32), P.DS_OK
        s = self.servos.get(sid)
        if op == P.OP_PING:
            if s:
                struct.pack_into("<H", data, 0, STS3215_MODEL)
            else:
                status = P.DS_TIMEOUT
        elif op == P.OP_READ:
            if not 0 < n <= 32:
                status = P.DS_BAD_ARG
            elif not s:
                status = P.DS_TIMEOUT
            else:
                struct.pack_into("<H", s.regs, REG_PRESENT_POS, int(round(s.pos)) & 0x0FFF)
                data[:n] = s.regs[addr:addr + n]
        elif op == P.OP_WRITE:
            if self.torque:
                status = P.DS_BUSY
            elif not s:
                status = P.DS_TIMEOUT
            else:
                s.regs[addr:addr + n] = q["data"][:n]
                if addr == REG_ID and q["data"][0] != sid:
                    self.servos[q["data"][0]] = self.servos.pop(sid)
        elif op == P.OP_SCAN:
            if self.torque:
                status = P.DS_BUSY
            for i in self.servos:
                data[i >> 3] |= 1 << (i & 7)
        elif op in (P.OP_SET_IDS, P.OP_GET_IDS):
            if op == P.OP_SET_IDS:
                if self.torque:
                    status = P.DS_BUSY
                else:
                    self.joint_ids = list(q["data"][:P.N_JOINTS])
                    self.probe()
            mask = sum(1 << j for j in self.active)
            data[:P.N_JOINTS] = bytes(self.joint_ids)
            struct.pack_into("<H", data, 14, mask)
        elif op == P.OP_SET_PARAM:
            if addr >= P.N_PARAMS:
                status = P.DS_BAD_ARG
            else:
                self.param[addr] = struct.unpack_from("<H", q["data"])[0]
        elif op in (P.OP_SET_REFLEX, P.OP_GET_REFLEX):
            if addr >= P.N_FALL_DIRS:
                status = P.DS_BAD_ARG
            else:
                if op == P.OP_SET_REFLEX:
                    self.reflex[addr] = list(struct.unpack_from(f"<{P.N_JOINTS}H", q["data"]))
                struct.pack_into(f"<{P.N_JOINTS}H", data, 0, *self.reflex[addr])
        elif op == P.OP_GET_INFO:
            struct.pack_into(f"<H{P.N_PARAMS}HB", data, 0, P.FW_VERSION, *self.param, int(self.echo))
        else:
            status = P.DS_BAD_ARG
        self.out += P.pack(P.TYPE_DIAG, q["seq"], op=op, id=sid, addr=addr, len=n, status=status, data=bytes(data))
