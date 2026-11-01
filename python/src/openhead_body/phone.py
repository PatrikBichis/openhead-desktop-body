"""
Telefonens rörelsedata över UDP till datorn, för kalibrering av huvudet (openhead-body-calib head-*).

Appen (planerad i app/) skickar ett UDP-paket per Core Motion-uppdatering (50–100 Hz) till datorns port
5005, en rad text per paket, antingen JSON

    {"t": 12.345, "g": [gx, gy, gz], "w": [wx, wy, wz]}

eller kommaseparerat  t,gx,gy,gz,wx,wy,wz.  t = CMDeviceMotion.timestamp (s), g = gravity (enhet g,
telefonens axlar: x höger, y upp, z ut ur skärmen), w = rotationRate (rad/s, används för att se att
huvudet står still). Andra fält ignoreras.

--phone fake ger en simulerad telefon som räknar gravitationen ur den simulerade Uno:ns servon med en
känd kalibrering (FAKE_TRUE), så att hela kedjan mätning -> anpassning går att prova utan hårdvara.
"""
import json
import math
import random
import socket
import time

from . import headcal as H
from .protocol import JOINT_NAMES

PORT = 5005

# "Sann" kalibrering för den simulerade telefonen: tecken, offset (rad) och stativlutning
FAKE_TRUE = {"signs": {"neck_pitch": 1, "head_pitch": -1, "head_yaw": 1, "head_roll": -1},
             "offset": {"neck_pitch": 0.0, "head_pitch": 0.06, "head_yaw": -0.09, "head_roll": 0.04},
             "base": (0.012, -0.008)}


def parse(line):
    line = line.strip()
    if not line:
        return None
    if line.startswith("{"):
        d = json.loads(line)
        return {"t": float(d.get("t", time.perf_counter())), "g": tuple(map(float, d["g"])),
                "w": tuple(map(float, d.get("w", (0.0, 0.0, 0.0))))}
    v = [float(x) for x in line.split(",")]
    return {"t": v[0], "g": tuple(v[1:4]), "w": tuple(v[4:7]) if len(v) >= 7 else (0.0, 0.0, 0.0)}


class PhoneUDP:
    def __init__(self, port=PORT, host="0.0.0.0"):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.setblocking(False)
        self.bad = 0

    def poll(self):
        out = []
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
            except (BlockingIOError, OSError):
                return out
            for line in data.decode("utf-8", "replace").splitlines():
                try:
                    s = parse(line)
                except (ValueError, KeyError, IndexError):
                    self.bad += 1
                    continue
                if s:
                    out.append(s)

    def close(self):
        self.sock.close()


class FakePhone:
    """Gravitation ur den simulerade Uno:ns råpositioner (state()["pos"]) och FAKE_TRUE, med brus."""

    def __init__(self, state, true=FAKE_TRUE, noise_g=0.002, rate=100.0, seed=1):
        self.state, self.true, self.noise, self.dt = state, true, noise_g, 1.0 / rate
        self.model = H.HeadModel()
        self.rnd = random.Random(seed)
        self.t_last = time.perf_counter()

    def sample(self, raw_by_joint):
        q = {n: H.joint_angle(raw_by_joint[n], self.true["signs"][n], self.true["offset"][n]) for n in H.NECK}
        g = self.model.gravity(q, self.true["base"])
        return tuple(c + self.rnd.gauss(0, self.noise) for c in g)

    def poll(self):
        now = time.perf_counter()
        n = int((now - self.t_last) / self.dt)
        if n <= 0:
            return []
        self.t_last += n * self.dt
        st = self.state()
        if st is None:
            return []
        raw = {name: st["pos"][JOINT_NAMES.index(name)] for name in H.NECK}
        return [{"t": now, "g": self.sample(raw), "w": (0.0, 0.0, 0.0)} for _ in range(min(n, 10))]

    def close(self):
        pass


def summarize(samples):
    """Medelvärde av gravitationen, dess spridning (g) och rotationshastighetens RMS (rad/s)."""
    n = len(samples)
    if not n:
        return None
    g = [sum(s["g"][i] for s in samples) / n for i in range(3)]
    sd = math.sqrt(sum(sum((s["g"][i] - g[i]) ** 2 for i in range(3)) for s in samples) / n)
    w = math.sqrt(sum(sum(c * c for c in s["w"]) for s in samples) / n)
    return {"g": g, "g_std": sd, "w_rms": w, "n": n}
