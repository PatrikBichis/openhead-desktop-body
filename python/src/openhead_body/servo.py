"""
STS3215-detaljer som inte hör till paketprotokollet: registerkontroll, servots egen offset (register 31)
och identifiering av utväxling med ett hastighetstest.

Positionen mäts av en magnetgivare på utgående axeln: 4096 steg per varv oavsett utväxling. Utväxlingen
ändrar alltså inte rad <-> rå, bara moment, hastighet och glapp. Därför syns den inte i positionen och
modellnumret är detsamma (777) för C001, C044 och C046; dekalen är förstahandskällan och hastighetstestet
en kontroll.
"""
from .link import (
    REG_ACC,
    REG_D,
    REG_I,
    REG_MAX_ANGLE,
    REG_MIN_ANGLE,
    REG_MODE,
    REG_MODEL,
    REG_OFFSET,
    REG_P,
    REG_VOLTAGE,
)

REG_MAX_TORQUE = 16          # EEPROM, 0–1000 (promille av fullt moment)
REG_TORQUE_LIMIT = 48        # RAM, 0–1000, sätts till REG_MAX_TORQUE vid uppstart

STS3215_MODEL = 777
STANDARD = {"mode": 0, "p": 32, "i": 0, "d": 0, "min": 0, "max": 4095, "offset": 0}

# Utväxlingar på SO-101-servona (CLAUDE.md §3) och nominell tomgångshastighet vid 7,4 V i steg/s.
# 1/345: 0,222 s/60° enligt Feetech (≈ 3070 steg/s). De andra skalade med utväxlingen. (verifiera på bänken)
GEARS = {
    "1/345": {"code": "C001", "steps_per_s": 3070},
    "1/191": {"code": "C044", "steps_per_s": 3070 * 345 / 191},
    "1/147": {"code": "C046", "steps_per_s": 3070 * 345 / 147},
}
NOMINAL_V = 7.4


def decode_offset(raw):
    """Register 31–32: tecken och belopp, bit 11 = negativt. Returnerar steg (±2047)."""
    mag = raw & 0x7FF
    return -mag if raw & 0x800 else mag


def encode_offset(steps):
    if not -2047 <= steps <= 2047:
        raise ValueError("offset utanför ±2047 steg")
    return (abs(steps) | 0x800) if steps < 0 else steps


def classify_gear(steps_per_s, volt):
    """Närmaste utväxling för uppmätt tomgångshastighet, normerad till 7,4 V. -> (namn, kvot mot nominellt)."""
    v = abs(steps_per_s) * NOMINAL_V / max(volt, 1.0)
    name = min(GEARS, key=lambda g: abs(v / GEARS[g]["steps_per_s"] - 1))
    return name, v / GEARS[name]["steps_per_s"]


def read_settings(uno, sid):
    """Inställningar som avgör om rå 2048 är servots mitt och om det rör sig som Open Duck förväntar sig."""
    u16 = lambda a: uno.read_u16(sid, a)
    p, d, i = uno.read(sid, REG_P, 3)
    return {"model": u16(REG_MODEL), "mode": uno.read(sid, REG_MODE, 1)[0], "p": p, "i": i, "d": d,
            "min": u16(REG_MIN_ANGLE), "max": u16(REG_MAX_ANGLE), "offset": decode_offset(u16(REG_OFFSET)),
            "max_torque": u16(REG_MAX_TORQUE), "torque_limit": u16(REG_TORQUE_LIMIT),
            "acc": uno.read(sid, REG_ACC, 1)[0], "volt": uno.read(sid, REG_VOLTAGE, 1)[0] / 10}


def problems(s):
    """Avvikelser från standardinställningen, som text."""
    out = []
    if s["model"] != STS3215_MODEL:
        out.append(f"modell {s['model']} (STS3215 = {STS3215_MODEL}; ST3020/12 V-modeller hör inte hemma i benen)")
    if s["mode"] != 0:
        out.append(f"läge {s['mode']} (ska vara 0 = position)")
    if (s["min"], s["max"]) != (0, 4095):
        out.append(f"vinkelgränser {s['min']}..{s['max']} (ska vara 0..4095, LeRobot-rester?)")
    if s["offset"] != 0:
        out.append(f"egen offset {s['offset']:+d} steg (ska vara 0: all offset ligger i joints.json)")
    if (s["p"], s["i"], s["d"]) != (32, 0, 0):
        out.append(f"P/I/D {s['p']}/{s['i']}/{s['d']} (Open Duck: 32/0/0)")
    if s["max_torque"] < 1000:
        out.append(f"maxmoment {s['max_torque']}/1000 i EEPROM")
    if s["acc"] != 0:
        out.append(f"acceleration {s['acc']} (body_uno förutsätter 0; kvar från servo_setup?)")
    if not 6.0 <= s["volt"] <= 8.4:
        out.append(f"spänning {s['volt']:.1f} V (7,4 V-buss väntad)")
    return out


def write_standard(uno, sid):
    """Positionsläge, P32/I0/D0, gränser 0..4095, offset 0, acceleration 0. Momentet måste vara av."""
    uno.write_eeprom(sid, REG_MODE, b"\x00")
    uno.write_eeprom(sid, REG_P, bytes([32, 0, 0]))
    uno.write_eeprom(sid, REG_MIN_ANGLE, bytes([0, 0, 0xFF, 0x0F]))
    uno.write_eeprom(sid, REG_OFFSET, b"\x00\x00")
    uno.write(sid, REG_ACC, b"\x00")


assert (REG_P, REG_D, REG_I) == (21, 22, 23)
