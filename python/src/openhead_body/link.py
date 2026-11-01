"""
Länken till Uno: skickar kommandon och diagnostik, tar emot tillstånd. Seriellt (USB) nu, UDP senare.

    with Uno.open("COM5") as uno:       # "fake" = simulerad Uno utan hårdvara (openhead_body/fake.py)
        print(uno.info())
        st = uno.command(targets, torque=True)
"""
import struct
import time

from . import protocol as P

# Feetech STS3215-register (samma som firmware/body_uno/feetech.h)
REG_MODEL, REG_ID, REG_BAUD, REG_MIN_ANGLE, REG_MAX_ANGLE = 3, 5, 6, 9, 11
REG_P, REG_D, REG_I, REG_OFFSET, REG_MODE = 21, 22, 23, 31, 33
REG_TORQUE, REG_ACC, REG_GOAL_POS, REG_GOAL_SPEED, REG_LOCK = 40, 41, 42, 46, 55
REG_PRESENT_POS, REG_PRESENT_SPEED, REG_PRESENT_LOAD, REG_VOLTAGE, REG_TEMPERATURE = 56, 58, 60, 62, 63

ARDUINO_VID = 0x2341
DS_NAMES = {P.DS_OK: "ok", P.DS_TIMEOUT: "svarar inte", P.DS_BUSY: "inte tillåtet med momentet på",
            P.DS_BAD_ARG: "felaktigt argument"}
SERVO_ERR_BITS = {0: "spänning", 1: "sensor", 2: "temperatur", 3: "ström", 5: "överlast"}


class LinkError(RuntimeError):
    pass


def describe_status(code):
    if code in DS_NAMES:
        return DS_NAMES[code]
    bits = [n for b, n in SERVO_ERR_BITS.items() if code & (1 << b)]
    return "servofel: " + (", ".join(bits) or hex(code))


def find_port():
    """COM-porten för första Arduino-kortet (VID 0x2341), annars None."""
    from serial.tools import list_ports
    for p in list_ports.comports():
        if p.vid == ARDUINO_VID:
            return p.device
    return None


class Uno:
    def __init__(self, stream):
        self.s = stream
        self.parser = P.StreamParser()
        self.seq = 0
        self.state = None           # senaste tillståndet
        self.states = 0
        self._diag = []

    @classmethod
    def open(cls, port=None, baud=460800, wait=3.0):
        if port == "fake":
            from .fake import FakeUno
            stream = FakeUno()
        else:
            import serial
            port = port or find_port()
            if not port:
                raise LinkError("hittar inget Arduino-kort. Ange --port COMx (lista: openhead-body-bench ports)")
            stream = serial.Serial(port, baud, timeout=0)
        uno = cls(stream)
        # Uno skickar tillstånd med 10 Hz när inga kommandon kommer; vänta på det första
        if not uno.wait_for(lambda: uno.state is not None, wait):
            uno.close()
            raise LinkError(f"inget svar från Uno på {port} (fel port, fel baud eller annan firmware?)")
        return uno

    def close(self):
        self.s.close()

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()

    # ------------------------------------------------------------------ låg nivå
    def _send(self, ptype, **fields):
        self.seq = (self.seq + 1) & 0xFFFF
        self.s.write(P.pack(ptype, self.seq, **fields))
        return self.seq

    def poll(self):
        """Läser in det som kommit. Returnerar nya tillstånd."""
        n = self.s.in_waiting
        new = []
        if n:
            for pkt in self.parser.feed(self.s.read(n)):
                if pkt["type"] == P.TYPE_STATE:
                    pkt["rx_time"] = time.perf_counter()
                    self.state = pkt
                    self.states += 1
                    new.append(pkt)
                elif pkt["type"] == P.TYPE_DIAG:
                    self._diag.append(pkt)
        return new

    def wait_for(self, cond, timeout):
        end = time.perf_counter() + timeout
        while True:
            self.poll()
            if cond():
                return True
            if time.perf_counter() > end:
                return False
            time.sleep(0.0005)

    # ------------------------------------------------------------------ styrning
    def command(self, targets, torque=False, estop=False, reset=False, antennas=(P.ANTENNA_OFF, P.ANTENNA_OFF),
                wait=0.1):
        """Skickar ett kommando (14 råmål) och väntar på tillståndet som svarar på det."""
        flags = (P.FLAG_TORQUE if torque else 0) | (P.FLAG_ESTOP if estop else 0) | (P.FLAG_RESET if reset else 0)
        seq = self._send(P.TYPE_COMMAND, flags=flags, targets=list(targets), antennas=list(antennas))
        if wait and not self.wait_for(lambda: self.state and self.state["seq_echo"] == seq, wait):
            raise LinkError(f"inget tillstånd för kommando {seq} inom {wait * 1000:.0f} ms")
        return self.state

    def torque_off(self):
        targets = self.state["pos"] if self.state else [2048] * P.N_JOINTS
        return self.command(targets, torque=False)

    # ------------------------------------------------------------------ diagnostik
    def diag(self, op, id=0, addr=0, length=0, data=b"", timeout=1.0, check=True):
        seq = self._send(P.TYPE_DIAG, op=op, id=id, addr=addr, len=length, data=bytes(data))
        found = []

        def got():
            for d in self._diag:
                if d["seq"] == seq and d["op"] == op:
                    found.append(d)
            self._diag = [d for d in self._diag if d["seq"] != seq]
            return bool(found)

        if not self.wait_for(got, timeout):
            raise LinkError(f"inget diagnostiksvar (op {op})")
        r = found[0]
        if check and r["status"] != P.DS_OK:
            raise LinkError(f"op {op} id {id}: {describe_status(r['status'])}")
        return r

    def info(self):
        d = self.diag(P.OP_GET_INFO)["data"]
        params = struct.unpack_from(f"<{P.N_PARAMS}H", d, 2)
        return {"fw": struct.unpack_from("<H", d)[0], "watchdog_ms": params[0], "max_step": params[1],
                "low_mv": params[2], "fall_deg": params[3], "fall_horizon_ms": params[4], "fall_limp_ms": params[5],
                "bus_echo": bool(d[2 + 2 * P.N_PARAMS])}

    def set_param(self, param, value):
        self.diag(P.OP_SET_PARAM, addr=param, data=struct.pack("<H", value))

    def set_reflex(self, direction, targets):
        """Reflexpose för en fallriktning (P.FALL_*): 14 råmål, P.REFLEX_KEEP = behåll leden."""
        d = self.diag(P.OP_SET_REFLEX, addr=direction, data=struct.pack(f"<{P.N_JOINTS}H", *targets))["data"]
        return list(struct.unpack_from(f"<{P.N_JOINTS}H", d))

    def get_reflex(self, direction):
        d = self.diag(P.OP_GET_REFLEX, addr=direction)["data"]
        return list(struct.unpack_from(f"<{P.N_JOINTS}H", d))

    def get_ids(self):
        d = self.diag(P.OP_GET_IDS)["data"]
        return list(d[:P.N_JOINTS]), struct.unpack_from("<H", d, 14)[0]

    def set_ids(self, ids):
        d = self.diag(P.OP_SET_IDS, data=bytes(ids), timeout=3.0)["data"]
        return list(d[:P.N_JOINTS]), struct.unpack_from("<H", d, 14)[0]

    def scan(self):
        d = self.diag(P.OP_SCAN, timeout=5.0)["data"]
        return [i for i in range(254) if d[i >> 3] & (1 << (i & 7))]

    def ping(self, sid):
        r = self.diag(P.OP_PING, id=sid, check=False)
        if r["status"] == P.DS_TIMEOUT:
            return None
        return struct.unpack_from("<H", r["data"])[0]

    def read(self, sid, addr, n):
        return self.diag(P.OP_READ, id=sid, addr=addr, length=n)["data"][:n]

    def write(self, sid, addr, data):
        self.diag(P.OP_WRITE, id=sid, addr=addr, length=len(data), data=data)

    def write_eeprom(self, sid, addr, data):
        """Skriver EEPROM-register: lås upp, skriv, lås igen (på servots ID efter skrivningen)."""
        self.write(sid, REG_LOCK, b"\x00")
        self.write(sid, addr, data)
        new_id = data[0] if addr == REG_ID else sid
        self.write(new_id, REG_LOCK, b"\x01")

    def read_u16(self, sid, addr):
        return struct.unpack("<H", self.read(sid, addr, 2))[0]
