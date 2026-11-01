"""
Leder: namn, gränser och omvandling mellan radianer (simulatorns ledvinklar) och servots råposition.

Rå position 0–4095 per varv, 2048 = 0 rad (verifiera per led). Tecken och offset per led ligger i
host/joints.json och fylls i vid bänktestet (openhead-body-bench offsets / sign). Gränserna kommer från MJCF
(sim/xmls/open_duck_mini_v2*.xml) och används för att klippa alla mål.
"""
import json
import math
import os
from pathlib import Path

from .protocol import DEFAULT_IDS, JOINT_NAMES, N_JOINTS

RAW_PER_REV = 4096
RAW_CENTER = 2048
RAW_PER_RAD = RAW_PER_REV / (2 * math.pi)

# (min, max) rad ur MJCF
LIMITS = {
    "left_hip_yaw": (-0.5236, 0.5236), "left_hip_roll": (-0.4363, 0.4363), "left_hip_pitch": (-1.2217, 0.5236),
    "left_knee": (-1.5708, 1.5708), "left_ankle": (-1.5708, 1.5708),
    "neck_pitch": (-0.3491, 1.1345), "head_pitch": (-0.7854, 0.7854), "head_yaw": (-2.7925, 2.7925),
    "head_roll": (-0.5236, 0.5236),
    "right_hip_yaw": (-0.5236, 0.5236), "right_hip_roll": (-0.4363, 0.4363), "right_hip_pitch": (-0.5236, 1.2217),
    "right_knee": (-1.5708, 1.5708), "right_ankle": (-1.5708, 1.5708),
}
# Standardpose (= keyframe "home" i MJCF), rad
DEFAULT_POSE = dict(zip(JOINT_NAMES, [0.002, 0.053, -0.63, 1.368, -0.784, 0.0, 0.0, 0.0, 0.0,
                                      -0.003, -0.065, 0.635, 1.379, -0.796]))

# Per rig, so it lives outside the package: $OPENHEAD_BODY_JOINTS or ~/.openhead-body/joints.json
CONFIG_PATH = Path(os.environ.get("OPENHEAD_BODY_JOINTS") or Path.home() / ".openhead-body" / "joints.json")


class Joints:
    """Ledkonfiguration: servo-ID, tecken (+1/-1) och offset (rad) per led."""

    def __init__(self, ids=None, sign=None, offset=None):
        self.ids = dict(zip(JOINT_NAMES, ids or DEFAULT_IDS))
        self.sign = sign or {n: 1 for n in JOINT_NAMES}
        self.offset = offset or {n: 0.0 for n in JOINT_NAMES}
        self.extra = {n: {} for n in JOINT_NAMES}    # övriga fält per led (utväxling, kalibrering), bevaras

    @classmethod
    def load(cls, path=CONFIG_PATH):
        path = Path(path)
        if not path.exists():
            return cls()
        d = json.loads(path.read_text(encoding="utf-8"))
        j = cls()
        for n in JOINT_NAMES:
            e = d.get(n, {})
            j.ids[n] = e.get("id", j.ids[n])
            j.sign[n] = e.get("sign", 1)
            j.offset[n] = e.get("offset_rad", 0.0)
            j.extra[n] = {k: v for k, v in e.items() if k not in ("id", "sign", "offset_rad")}
        return j

    def save(self, path=CONFIG_PATH):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        d = {n: {"id": self.ids[n], "sign": self.sign[n], "offset_rad": round(self.offset[n], 5), **self.extra[n]}
             for n in JOINT_NAMES}
        Path(path).write_text(json.dumps(d, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    def id_list(self):
        return [self.ids[n] for n in JOINT_NAMES]

    def clamp(self, name, q):
        lo, hi = LIMITS[name]
        return min(max(q, lo), hi)

    def to_raw(self, name, q):
        """rad -> rå. Klipper mot ledgränserna."""
        q = self.clamp(name, q)
        r = RAW_CENTER + self.sign[name] * (q + self.offset[name]) * RAW_PER_RAD
        return int(min(max(round(r), 0), RAW_PER_REV - 1))

    def to_rad(self, name, raw):
        return (raw - RAW_CENTER) / RAW_PER_RAD * self.sign[name] - self.offset[name]

    def vel_to_rad(self, name, steps_per_s):
        return steps_per_s / RAW_PER_RAD * self.sign[name]


def index(name):
    if name not in JOINT_NAMES:
        raise KeyError(f"okänd led {name!r}. Leder: {', '.join(JOINT_NAMES)}")
    return JOINT_NAMES.index(name)


assert len(LIMITS) == len(DEFAULT_POSE) == N_JOINTS
