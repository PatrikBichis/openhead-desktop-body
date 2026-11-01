"""
Paketformatet mellan dator/telefon och Uno (CLAUDE.md avsnitt 5). Enda källan: firmware-headern
firmware/body_uno/protocol.h genereras härifrån (cd python && uv run python -m openhead_body.protocol --header) och ett test
kontrollerar att den är i synk.

Little-endian, packade strukturer, fast storlek per typ. Samma paket går över USB-seriellt (steg 1)
och UDP (senare). På seriell länk letas början upp med magic och paketet kontrolleras med CRC.
"""
import struct
import sys

MAGIC = 0x4F44            # "OD", på tråden 0x44 0x4F
VERSION = 1
TYPE_COMMAND = 1          # dator/telefon -> Uno
TYPE_STATE = 2            # Uno -> dator/telefon
TYPE_DIAG = 3             # båda håll: konfiguration och diagnostik

N_JOINTS = 14
# Aktionsordningen i policyn (Open Duck Playground). targets[i] gäller JOINT_NAMES[i].
JOINT_NAMES = [
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "neck_pitch", "head_pitch", "head_yaw", "head_roll",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
]
# Etikett att märka servot med (vilken plats det ska monteras på)
JOINT_LABELS = [
    "V höft yaw", "V höft roll", "V höft pitch", "V knä", "V fotled",
    "Nacke pitch", "Huvud pitch", "Huvud yaw", "Huvud roll",
    "H höft yaw", "H höft roll", "H höft pitch", "H knä", "H fotled",
]
# Open Ducks servo-ID: höger ben 10–14, vänster ben 20–24, hals 30–33
DEFAULT_IDS = [20, 21, 22, 23, 24, 30, 31, 32, 33, 10, 11, 12, 13, 14]
NO_SERVO = 0xFF           # i ID-tabellen: leden saknas

# flags i kommandot
FLAG_TORQUE = 1 << 0      # moment på
FLAG_ESTOP = 1 << 1       # nödstopp (låser tills FLAG_RESET)
FLAG_RESET = 1 << 2       # nollställ watchdog- och nödstoppslarm

# status i tillståndet
ST_WATCHDOG = 1 << 0      # watchdog utlöst (låst tills FLAG_RESET)
ST_LOW_VOLTAGE = 1 << 1
ST_SERVO_ERROR = 1 << 2   # något aktivt servo svarade inte eller rapporterade fel denna cykel
ST_ESTOP = 1 << 3         # nödstopp låst
ST_TORQUE = 1 << 4        # momentet är på
ST_FALL = 1 << 5          # fallreflexen har löst ut (låst tills FLAG_RESET; kommandomål ignoreras under tiden)
ST_IMU_OK = 1 << 6        # BNO085 levererar data (imu-fältet är giltigt)

ANTENNA_OFF = 0xFF        # ingen puls till antennservot

# imu i tillståndet: kroppens BNO085 i robotens ram (x framåt, y vänster, z upp), som simulatorns site "imu":
# gyro (rad/s) × IMU_GYRO_SCALE, accelerometer (m/s², +9,81 på z i vila) × IMU_ACC_SCALE
IMU_GYRO_SCALE = 1000     # mrad/s
IMU_ACC_SCALE = 100       # cm/s²

# Fallreflex (sim/fall_test.py, strategin "skydda"): Uno läser BNO085 själv och reagerar inom några ms, utan att
# vänta på telefonen. Upptäckt: lutning över 8° och lutning + lutningshastighet × FALL_HORIZON_MS över FALL_DEG.
# Då sätts målen till reflexposen för fallriktningen (utan max_step), och efter FALL_LIMP_MS slås momentet av.
# Reflexen är bara beredd när roboten har stått upprätt (lutning under 15°) sedan momentet slogs på eller reset.
FALL_FWD, FALL_BACK, FALL_LEFT, FALL_RIGHT = 0, 1, 2, 3
N_FALL_DIRS = 4
REFLEX_KEEP = 0xFFFF      # i en reflexpose: behåll leden där den är

# diagnostikoperationer (op). Svaret har samma op och status nedan.
OP_PING = 1               # id -> data[0:2] modellnummer
OP_READ = 2               # id, addr, len (<= 32) -> data
OP_WRITE = 3              # id, addr, len, data (bara med momentet av)
OP_SCAN = 4               # -> data = bitkarta över ID 0..253 som svarade
OP_SET_IDS = 5            # data[0:14] = servo-ID per led (NO_SERVO = saknas)
OP_GET_IDS = 6            # -> data[0:14], data[14:16] aktiv-mask (u16)
OP_SET_PARAM = 7          # addr = parameter, data[0:2] = värde (u16)
OP_GET_INFO = 8           # -> data: fw-version u16, parametrar u16 × N_PARAMS, eko-läge u8
OP_SET_REFLEX = 9         # addr = fallriktning (FALL_*), data[0:28] = 14 råmål u16 (REFLEX_KEEP = behåll)
OP_GET_REFLEX = 10        # addr = fallriktning -> data[0:28]

DS_OK = 0
DS_TIMEOUT = 0xFF         # servot svarade inte
DS_BUSY = 0xFE            # inte tillåtet med momentet på
DS_BAD_ARG = 0xFD
# 1..0x7F: servots egen felbyte

# parametrar för OP_SET_PARAM
PARAM_WATCHDOG_MS = 0     # standard 150
PARAM_MAX_STEP = 1        # största målsteg per kommando i råenheter (0 = ingen gräns), standard 40
PARAM_LOW_MV = 2          # lågspänningsgräns, standard 6000
PARAM_FALL_DEG = 3        # fallreflexens gräns i grader, 0 = av (standard: av tills den provats på roboten)
PARAM_FALL_HORIZON_MS = 4  # prognosens tidshorisont, standard 150
PARAM_FALL_LIMP_MS = 5    # momentet av så här länge efter reflexen, 0 = håll kvar reflexposen; standard 1500
N_PARAMS = 6
PARAM_DEFAULTS = [150, 40, 6000, 0, 150, 1500]
FW_VERSION = 2

# Fält: (namn, typ, antal). Typer: u8 u16 u32 i16
_T = {"u8": ("B", "uint8_t"), "u16": ("H", "uint16_t"), "u32": ("I", "uint32_t"), "i16": ("h", "int16_t")}
HEADER = [("magic", "u16", 1), ("version", "u8", 1), ("type", "u8", 1), ("seq", "u16", 1)]
COMMAND = HEADER + [("flags", "u8", 1), ("targets", "u16", N_JOINTS), ("antennas", "u8", 2), ("crc", "u16", 1)]
STATE = HEADER + [
    ("seq_echo", "u16", 1), ("t_ms", "u32", 1), ("status", "u8", 1),
    ("pos", "u16", N_JOINTS), ("vel", "i16", N_JOINTS), ("load", "i16", N_JOINTS),
    ("feet", "u8", 1), ("bus_mv", "u16", 1),
    ("diag", "u8", 4),        # led-index, temperatur °C, spänning 0,1 V, servots felbyte (rundgång)
    ("imu", "i16", 6),        # BNO085 i kroppen: gyro × IMU_GYRO_SCALE, acc × IMU_ACC_SCALE (nollor utan ST_IMU_OK)
    ("crc", "u16", 1),
]
DIAG = HEADER + [("op", "u8", 1), ("id", "u8", 1), ("addr", "u8", 1), ("len", "u8", 1), ("status", "u8", 1),
                 ("data", "u8", 32), ("crc", "u16", 1)]
LAYOUTS = {TYPE_COMMAND: ("CommandPacket", COMMAND), TYPE_STATE: ("StatePacket", STATE), TYPE_DIAG: ("DiagPacket", DIAG)}


def _fmt(fields):
    return "<" + "".join(f"{n}{_T[t][0]}" if n > 1 else _T[t][0] for _, t, n in fields)


FMT = {k: _fmt(v) for k, (_, v) in LAYOUTS.items()}
SIZE = {k: struct.calcsize(f) for k, f in FMT.items()}


def crc16(data, crc=0xFFFF):
    """CRC-16/CCITT-FALSE (poly 0x1021, start 0xFFFF, ingen reflektion)."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def pack(ptype, seq, **values):
    """Packa ett paket. Fält som saknas blir noll, crc räknas fram."""
    vals = []
    for name, t, n in LAYOUTS[ptype][1]:
        if name == "magic":
            v = MAGIC
        elif name == "version":
            v = VERSION
        elif name == "type":
            v = ptype
        elif name == "seq":
            v = seq & 0xFFFF
        elif name == "crc":
            v = 0
        else:
            v = values.get(name, 0 if n == 1 else [0] * n)
        if n == 1:
            vals.append(int(v))
        elif t == "u8" and isinstance(v, (bytes, bytearray)):
            vals.extend(list(v.ljust(n, b"\0")[:n]))
        else:
            v = list(v)
            assert len(v) <= n, (name, len(v), n)
            vals.extend(int(x) for x in v + [0] * (n - len(v)))
    raw = bytearray(struct.pack(FMT[ptype], *vals))
    struct.pack_into("<H", raw, len(raw) - 2, crc16(raw[:-2]))
    return bytes(raw)


def unpack(raw):
    """dict med fälten, eller ValueError om paketet är fel."""
    if len(raw) < 6:
        raise ValueError("för kort")
    magic, ver, ptype, _ = struct.unpack_from("<HBBH", raw)
    if magic != MAGIC or ver != VERSION or ptype not in FMT:
        raise ValueError(f"fel header {magic:#x} v{ver} typ {ptype}")
    if len(raw) != SIZE[ptype]:
        raise ValueError(f"fel storlek {len(raw)} != {SIZE[ptype]}")
    if crc16(raw[:-2]) != struct.unpack_from("<H", raw, len(raw) - 2)[0]:
        raise ValueError("fel CRC")
    flat = struct.unpack(FMT[ptype], raw)
    out, i = {}, 0
    for name, t, n in LAYOUTS[ptype][1]:
        out[name] = flat[i] if n == 1 else list(flat[i:i + n])
        i += n
    if ptype == TYPE_DIAG:
        out["data"] = bytes(out["data"])
    return out


class StreamParser:
    """Plockar hela paket ur en byteström (seriell länk). Skräp före magic och paket med fel CRC hoppas över."""

    def __init__(self):
        self.buf = bytearray()
        self.dropped = 0

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(b"\x44\x4f")
            if i < 0:
                self.dropped += max(0, len(self.buf) - 1)
                del self.buf[:max(0, len(self.buf) - 1)]
                return out
            if i:
                self.dropped += i
                del self.buf[:i]
            if len(self.buf) < 4:
                return out
            ptype = self.buf[3]
            if self.buf[2] != VERSION or ptype not in SIZE:
                self.dropped += 1
                del self.buf[:1]
                continue
            n = SIZE[ptype]
            if len(self.buf) < n:
                return out
            try:
                out.append(unpack(bytes(self.buf[:n])))
                del self.buf[:n]
            except ValueError:
                self.dropped += 1
                del self.buf[:1]


def c_header():
    """C-header för firmware (genererad, redigera inte för hand)."""
    L = ["// GENERERAD av python/src/openhead_body/protocol.py (cd python && uv run python -m openhead_body.protocol --header). Redigera inte för hand.",
         "#pragma once", "#include <stdint.h>", ""]
    consts = {k: v for k, v in globals().items()
              if k.isupper() and isinstance(v, int) and not k.startswith("_")}
    for k, v in consts.items():
        L.append(f"#define ODM_{k} {v}")
    L.append(f"static const uint8_t ODM_DEFAULT_IDS[{N_JOINTS}] = {{{', '.join(map(str, DEFAULT_IDS))}}};")
    q = lambda xs: ", ".join(f'"{x}"' for x in xs)
    L.append(f"static const char* const ODM_JOINT_NAMES[{N_JOINTS}] = {{{q(JOINT_NAMES)}}};")
    L.append(f"static const char* const ODM_JOINT_LABELS[{N_JOINTS}] = {{{q(JOINT_LABELS)}}};  // UTF-8")
    L.append(f"static const uint16_t ODM_PARAM_DEFAULTS[{N_PARAMS}] = {{{', '.join(map(str, PARAM_DEFAULTS))}}};")
    for ptype, (cname, _) in LAYOUTS.items():
        L.append(f"#define ODM_SIZE_{cname.upper()} {SIZE[ptype]}")
    L.append("")
    for ptype, (cname, fields) in LAYOUTS.items():
        L.append("typedef struct __attribute__((packed)) {")
        for name, t, n in fields:
            L.append(f"  {_T[t][1]} {name}{f'[{n}]' if n > 1 else ''};")
        L.append(f"}} {cname};")
        L.append(f'static_assert(sizeof({cname}) == {SIZE[ptype]}, "{cname}");')
        L.append("")
    return "\n".join(L)


FIRMWARE = __import__("pathlib").Path(__file__).resolve().parents[3] / "firmware"
SKETCHES = ["body_uno", "servo_setup"]          # båda får samma protocol.h
HEADER_PATHS = [FIRMWARE / s / "protocol.h" for s in SKETCHES]

if __name__ == "__main__":
    if "--header" in sys.argv:
        for hp in HEADER_PATHS:
            hp.write_text(c_header(), encoding="utf-8", newline="\n")
            print(f"skrev {hp}")
    else:
        for k, (cname, _) in LAYOUTS.items():
            print(f"{cname}: {SIZE[k]} byte")
