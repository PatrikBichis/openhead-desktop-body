"""
Kalibrering av halsen och huvudet på bänkriggen med telefonens gravitationsvektor (Core Motion).

Modell: telefonens orientering i riggen är en produkt av rotationer kring halsledernas axlar (riggens
koordinater i nolläget, cad/benchhead/reference/frames.json) gånger telefonens montering i huvudet
(edits/placering.json, "iphone12"):

    R(q) = Rot(a_np, q_np) · Rot(a_hp, q_hp) · Rot(a_hy, q_hy) · Rot(a_hr, q_hr) · M
    g_telefon = R(q)ᵀ · R_basᵀ · (0, 0, −1)

där R_bas är stativets lutning och ledvinkeln kommer från servots uppmätta råposition som i joints.json:
q = tecken·(rå − 2048)·skala/k − offset (+ glapp·riktning). Minsta kvadrat (Levenberg–Marquardt) ger
offset, tecken, skala och glapp per led.

Vad gravitationen inte kan se (därför mäts det på annat sätt, se host/KALIBRERING.md):
- neck_pitch och head_pitch har parallella axlar: bara summan av deras offset syns. neck_pitch sätts med
  mall, head_pitch anpassas.
- Stativets lutning kring y är parallell med pitch-axlarna: mäts med telefonen liggande på basplattan.
- head_yaw syns bara när yaw-axeln lutar, alltså när neck_pitch + head_pitch ≠ 0.
- Telefonens rotation i hållaren kring roll-axeln går inte att skilja från head_roll-offset.

Ren Python (inga beroenden), så att verktyget går i host-venv:en och logiken kan portas till Swift.
"""
import itertools
import json
import math
import random
from pathlib import Path

NECK = ("neck_pitch", "head_pitch", "head_yaw", "head_roll")
DATA = Path(__file__).resolve().parent / "data"
FRAMES = DATA / "frames.json"          # joint axes in the rig at rest (from the CAD)
PLACEMENT = DATA / "placement.json"    # where the phone sits in the holder
RAW_PER_RAD = 4096 / (2 * math.pi)

# Telefonens axlar (Core Motion: x höger, y upp mot frontkameran, z ut ur skärmen) i riggen, kolumnvis.
# Läge A i hållaren: överkanten åt +y, skärmen framåt (+x).
PHONE_IN_HEAD = ((0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0))
# Telefonen liggande på basplattan, skärmen upp, överkanten framåt
PHONE_ON_BASE = ((0.0, 1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, 1.0))


# ------------------------------------------------------------------------------------------- 3×3-algebra
def mul(A, B):
    return tuple(tuple(sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def T(A):
    return tuple(tuple(A[j][i] for j in range(3)) for i in range(3))


def mv(A, v):
    return tuple(sum(A[i][k] * v[k] for k in range(3)) for i in range(3))


def rot(axis, a):
    """Rodrigues: rotation a (rad) kring enhetsvektorn axis."""
    x, y, z = axis
    c, s, C = math.cos(a), math.sin(a), 1 - math.cos(a)
    return ((c + x * x * C, x * y * C - z * s, x * z * C + y * s),
            (y * x * C + z * s, c + y * y * C, y * z * C - x * s),
            (z * x * C - y * s, z * y * C + x * s, c + z * z * C))


def unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


def angle_between(u, v):
    u, v = unit(u), unit(v)
    return math.atan2(math.sqrt(sum(c * c for c in cross(u, v))), sum(a * b for a, b in zip(u, v)))


def cross(u, v):
    return (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])


def solve(A, b):
    """Gausselimination med pivotering (liten tät matris)."""
    n = len(b)
    M = [list(A[i]) + [b[i]] for i in range(n)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(M[r][c]))
        if abs(M[p][c]) < 1e-15:
            raise ZeroDivisionError("singulär")
        M[c], M[p] = M[p], M[c]
        for r in range(n):
            if r != c:
                f = M[r][c] / M[c][c]
                for k in range(c, n + 1):
                    M[r][k] -= f * M[c][k]
    return [M[i][n] / M[i][i] for i in range(n)]


# ------------------------------------------------------------------------------------------- kinematik
class HeadModel:
    def __init__(self, axes=None, mount=PHONE_IN_HEAD):
        self.axes = axes or load_axes()
        self.mount = mount

    def phone_R(self, q, base=(0.0, 0.0)):
        """Telefonens axlar i världen (lodrätt z) för ledvinklar q {led: rad} och stativlutning (kring x, y)."""
        R = mul(rot((1, 0, 0), base[0]), rot((0, 1, 0), base[1]))
        for n in NECK:
            R = mul(R, rot(self.axes[n], q.get(n, 0.0)))
        return mul(R, self.mount)

    def gravity(self, q, base=(0.0, 0.0)):
        """Core Motions gravity (enhet g, telefonens axlar) för ledvinklar q."""
        return mv(T(self.phone_R(q, base)), (0.0, 0.0, -1.0))

    def positive_direction(self, name, deg=10.0):
        """Hur telefonens skärmnormal och överkant flyttar sig vid positiv ledvinkel (för teckenkontroll)."""
        R0, R1 = self.phone_R({}), self.phone_R({name: math.radians(deg)})
        return {"skarm": tuple(R1[i][2] - R0[i][2] for i in range(3)),
                "overkant": tuple(R1[i][1] - R0[i][1] for i in range(3))}


def load_axes(path=FRAMES):
    d = json.loads(Path(path).read_text(encoding="utf-8"))["joints"]
    return {n: unit(d[n]["axis"]) for n in NECK}


def load_mount(path=PLACEMENT):
    """Telefonens montering ur Shapr3D-placeringen om den finns, annars läge A."""
    try:
        R = json.loads(Path(path).read_text(encoding="utf-8"))["iphone12"]["R"]
        return tuple(tuple(float(x) for x in r) for r in R)
    except (OSError, KeyError, ValueError):
        return PHONE_IN_HEAD


def base_gravity(base):
    """Gravitationens riktning i riggens axlar för stativlutningen base = (kring x, kring y)."""
    return mv(T(mul(rot((1, 0, 0), base[0]), rot((0, 1, 0), base[1]))), (0.0, 0.0, -1.0))


def base_tilt(g_phone_flat):
    """Stativets lutning (kring x, y) ur gravitationen med telefonen liggande på basplattan."""
    g = mv(PHONE_ON_BASE, unit(g_phone_flat))          # i riggens axlar
    # g_rig = Ry(-by)·Rx(-bx)·(0,0,-1) = (sin(by)cos(bx), -sin(bx), -cos(by)cos(bx))
    bx = math.asin(max(-1.0, min(1.0, -g[1])))
    by = math.atan2(g[0], -g[2])
    return bx, by


# ------------------------------------------------------------------------------------------- mätmodell
def joint_angle(raw, sign, offset, scale=1.0, backlash=0.0, direction=0):
    return sign * (raw - 2048) / RAW_PER_RAD * scale - offset + backlash * direction


class Fit:
    """Resultatet av en anpassning."""

    def __init__(self, **kw):
        self.__dict__.update(kw)

    def as_dict(self):
        return dict(self.__dict__)


def fit(samples, model, signs, offsets, fit_offsets=("head_pitch", "head_yaw", "head_roll"), fit_scale=(),
        fit_backlash=(), fit_base_roll=True, base=(0.0, 0.0), try_signs=NECK, max_offset=math.radians(25)):
    """
    samples: [{"raw": {led: rå}, "dir_raw": {led: ±1/0}, "g": (gx, gy, gz)}], där dir_raw är riktningen
    (i råenheter) leden senast rörde sig åt innan mätningen; används bara när glapp anpassas.
    signs/offsets: startvärden (joints.json). Leder i try_signs prövas med båda tecknen; bästa kombinationen
    väljs. Returnerar Fit med offset, tecken, skala, glapp, stativlutning och residualer (rad).

    Lösningar med |offset| > max_offset räknas som orimliga: hornets delning ger högst ±7,2°, och med
    omvända pitch-tecken plus yaw-offset 180° passar gravitationen lika bra (spegling kring lodlinjen).
    Skalan ska ligga nära 1 (givaren sitter på utgående axeln); negativ skala vore ett dolt teckenbyte.
    """
    if "neck_pitch" in fit_offsets and "head_pitch" in fit_offsets:
        raise ValueError("neck_pitch och head_pitch har parallella axlar; bara summan av offset syns. "
                         "Lås neck_pitch (mall) och anpassa head_pitch.")
    # steg 1: alla teckenkombinationer med bara offset och stativ (snabbt); steg 2: full modell för de två bästa
    results = []
    for combo in itertools.product((1, -1), repeat=len(try_signs)):
        sg = dict(signs)
        sg.update(dict(zip(try_signs, combo)))
        # offset som inte anpassas (neck_pitch från mallen) är mätt med det gamla tecknet: rå0 = 2048 + tecken·offset·k
        of = {n: v * sg.get(n, 1) * signs.get(n, 1) for n, v in offsets.items()}
        results.append(_fit_signs(samples, model, sg, of, fit_offsets, (), (), fit_base_roll, base))

    def plausible(r):
        return (all(abs(r.offset.get(n, 0.0)) <= max_offset for n in fit_offsets)
                and all(abs(r.scale[n] - 1) < 0.2 for n in fit_scale))

    ok = sorted(filter(plausible, results), key=lambda r: r.rms)
    if fit_scale or fit_backlash:
        ok = sorted(filter(plausible, [_fit_signs(samples, model, r.signs, r.offset, fit_offsets, fit_scale,
                                                  fit_backlash, fit_base_roll, (r.base[0], base[1]))
                                       for r in ok[:2]]), key=lambda r: r.rms)
    if not ok:
        raise ValueError("ingen teckenkombination ger rimliga offset; kontrollera montering och mätdata")
    best = ok[0]
    best.second_rms = ok[1].rms if len(ok) > 1 else None
    return best


def _fit_signs(samples, model, signs, offsets, fit_offsets, fit_scale, fit_backlash, fit_base_roll, base):
    names = ([("offset", n) for n in fit_offsets] + [("scale", n) for n in fit_scale]
             + [("backlash", n) for n in fit_backlash] + ([("base", 0)] if fit_base_roll else []))
    x0 = []
    for kind, n in names:
        x0.append({"offset": offsets.get(n, 0.0), "scale": 1.0, "backlash": 0.0, "base": base[0]}[kind])

    def unpack(x):
        p = {"offset": dict(offsets), "scale": {n: 1.0 for n in NECK}, "backlash": {n: 0.0 for n in NECK},
             "base": list(base)}
        for (kind, n), v in zip(names, x):
            if kind == "base":
                p["base"][0] = v
            else:
                p[kind][n] = v
        return p

    def residuals(x):
        p = unpack(x)
        out = []
        for s in samples:
            q = {n: joint_angle(s["raw"][n], signs[n], p["offset"].get(n, 0.0), p["scale"][n], p["backlash"][n],
                                signs[n] * s.get("dir_raw", {}).get(n, 0)) for n in NECK}
            pred = model.gravity(q, tuple(p["base"]))
            out.extend(m - g for m, g in zip(unit(s["g"]), pred))
        return out

    x, lam = list(x0), 1e-3
    r = residuals(x)
    cost = sum(v * v for v in r)
    for _ in range(100):
        J = []
        for k in range(len(x)):
            h = 1e-6
            xp = list(x)
            xp[k] += h
            J.append([(a - b) / h for a, b in zip(residuals(xp), r)])
        JTJ = [[sum(a * b for a, b in zip(J[i], J[j])) for j in range(len(x))] for i in range(len(x))]
        JTr = [sum(a * b for a, b in zip(J[i], r)) for i in range(len(x))]
        improved = False
        while lam < 1e8:
            A = [[JTJ[i][j] + (lam * JTJ[i][i] + 1e-12 if i == j else 0.0) for j in range(len(x))]
                 for i in range(len(x))]
            try:
                dx = solve(A, [-v for v in JTr])
            except ZeroDivisionError:
                lam *= 10
                continue
            xn = [a + b for a, b in zip(x, dx)]
            rn = residuals(xn)
            cn = sum(v * v for v in rn)
            if cn < cost:
                x, r, cost, lam, improved = xn, rn, cn, lam / 10, True
                break
            lam *= 10
        if not improved or max((abs(v) for v in dx), default=0) < 1e-9:
            break
    p = unpack(x)
    per = [math.sqrt(sum(v * v for v in r[3 * i:3 * i + 3])) for i in range(len(samples))]
    # parameterns standardavvikelse ur (JᵀJ)⁻¹·σ²: diagonal via lösning mot enhetsvektorer
    std = {}
    if x and len(r) > len(x):
        s2 = cost / (len(r) - len(x))
        for k, key in enumerate(names):
            e = [1.0 if i == k else 0.0 for i in range(len(x))]
            try:
                std[f"{key[0]}:{key[1]}"] = math.sqrt(max(solve(JTJ, e)[k], 0.0) * s2)
            except ZeroDivisionError:
                std[f"{key[0]}:{key[1]}"] = float("inf")
    return Fit(signs=dict(signs), offset=p["offset"], scale=p["scale"], backlash=p["backlash"], base=tuple(p["base"]),
               rms=math.sqrt(sum(v * v for v in per) / len(per)), max_err=max(per), per_sample=per, std=std)


# ------------------------------------------------------------------------------------------- poser
def plan_poses():
    """
    Mätposer (grader, simulatorns ledvinklar). Pitch varieras för pitch och roll; yaw mäts med huvudet
    framåt- eller bakåtlutat (annars syns yaw inte i gravitationen); några neck_pitch-poser kontrollerar
    tecken och skala för nacken. 24 poser, alla inom MJCF-gränserna med marginal.
    """
    P = []
    for hp in (-25, 0, 25):
        for roll in (-20, 0, 20):
            P.append({"neck_pitch": 0, "head_pitch": hp, "head_yaw": 0, "head_roll": roll})
    for hp in (25, -25):
        for yaw in (-60, -30, 30, 60):
            P.append({"neck_pitch": 0, "head_pitch": hp, "head_yaw": yaw, "head_roll": 0})
    for yaw in (-45, 45):
        for roll in (-15, 15):
            P.append({"neck_pitch": 0, "head_pitch": 25, "head_yaw": yaw, "head_roll": roll})
    P += [{"neck_pitch": 20, "head_pitch": 0, "head_yaw": 0, "head_roll": 0},
          {"neck_pitch": 40, "head_pitch": -25, "head_yaw": 30, "head_roll": 0},
          {"neck_pitch": -15, "head_pitch": 0, "head_yaw": 0, "head_roll": 10}]
    return [{n: math.radians(v) for n, v in p.items()} for p in P]


def synthetic(model, true, poses, noise_deg=0.2, raw_noise=1, seed=1, approach=None):
    """
    Konstgjorda mätningar från en känd kalibrering `true` = {"signs", "offset", "backlash", "base"},
    för test av anpassningen. approach: {led: ±1} (riktning leden senast rörde sig åt), annars +1.
    """
    rnd = random.Random(seed)
    out = []
    for q in poses:
        raw, d, d_raw = {}, {}, {}
        for n in NECK:
            d[n] = (approach or {}).get(n, 1)
            d_raw[n] = d[n] * true["signs"][n]
            # servot står där den uppmätta råpositionen säger; den verkliga vinkeln innehåller glappet
            raw[n] = round(2048 + true["signs"][n] * (q[n] + true["offset"][n]) * RAW_PER_RAD) + rnd.randint(
                -raw_noise, raw_noise)
        q_true = {n: joint_angle(raw[n], true["signs"][n], true["offset"][n], 1.0, true.get("backlash", {}).get(n, 0.0),
                                 d[n]) for n in NECK}
        g = model.gravity(q_true, true.get("base", (0.0, 0.0)))
        tilt = rot(unit((rnd.gauss(0, 1), rnd.gauss(0, 1), rnd.gauss(0, 1))), math.radians(rnd.gauss(0, noise_deg)))
        out.append({"raw": raw, "dir_raw": d_raw, "g": mv(tilt, g)})
    return out
