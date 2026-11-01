"""
Servoidentifiering, nolläge och kalibrering av huvudet. Arbetsgången står i host/KALIBRERING.md.

    python openhead-body-calib audit                     # register per led: modell, gränser, egen offset, P, moment
    python openhead-body-calib fix neck_pitch            # standardinställning (gränser 0..4095, offset 0, P32 ...)
    python openhead-body-calib gear head_yaw --decal 1/191   # hastighetstest av utväxlingen (servot löst!)
    python openhead-body-calib zero neck_pitch head_pitch    # långsamt, med momentgräns, till 0 rad i simulatorn
    python openhead-body-calib zero --raw head_yaw       # till rå 2048 (för att sätta hornet)
    python openhead-body-calib head-base                 # stativets lutning: telefonen på basplattan
    python openhead-body-calib head-capture              # kör mätposerna, logga telefonens gravitation
    python openhead-body-calib head-fit kalibrering/huvud_*.json --save   # offset och tecken till joints.json

Kräver firmware/body_uno. --port fake kör mot en simulerad Uno med servon på halsens ID (30–33) och
--phone fake mot en simulerad telefon (openhead_body/phone.py).
"""
import argparse
import datetime
import glob
import json
import math
import sys
import time
from pathlib import Path

from openhead_body import headcal as H
from openhead_body import protocol as P
from openhead_body import servo as S
from openhead_body.bench import active_names, minjerk, release, run
from openhead_body.joints import CONFIG_PATH, LIMITS, RAW_PER_RAD, Joints, index
from openhead_body.link import LinkError, Uno
from openhead_body.phone import FAKE_TRUE, PORT, FakePhone, PhoneUDP, summarize

# Measurements are per rig: next to joints.json
CAL_DIR = CONFIG_PATH.parent / "kalibrering"
BASE_FILE = CAL_DIR / "stativ.json"
FAKE_IDS = (30, 31, 32, 33)


class Abort(RuntimeError):
    pass


# --------------------------------------------------------------------------------------------- hjälp
def open_uno(a):
    if a.port == "fake":
        from openhead_body.fake import FakeUno
        uno = Uno(FakeUno(ids=FAKE_IDS))
        if not uno.wait_for(lambda: uno.state is not None, 1.0):
            raise LinkError("den simulerade Uno:n svarar inte")
        return uno
    return Uno.open(a.port, a.baud)


def connect(a):
    uno = open_uno(a)
    joints = Joints.load(a.config)
    release(uno)
    ids, mask = uno.get_ids()
    if ids != joints.id_list():
        ids, mask = uno.set_ids(joints.id_list())
    return uno, joints, mask


def pick(names, mask, default=None):
    names = names or default or active_names(mask)
    for n in names:
        if not mask & (1 << index(n)):
            raise Abort(f"{n} svarar inte (ID enligt joints.json). Kör openhead-body-bench scan / config --id.")
    return names


def ask_yes(a, text):
    if getattr(a, "yes", False):
        return True
    return input(f"{text} Skriv JA: ").strip().lower() == "ja"


def set_torque_limit(uno, joints, names, value):
    for n in names:
        uno.write(joints.ids[n], S.REG_TORQUE_LIMIT, bytes([value & 0xFF, value >> 8]))


def guard(limits_load):
    """Avbryter rörelsen om ett servo tar för mycket last eller Uno rapporterar servofel."""

    def check(st, names):
        if st["status"] & P.ST_SERVO_ERROR:
            raise Abort("servofel rapporterat av Uno")
        for n in names:
            if abs(st["load"][index(n)]) > limits_load:
                raise Abort(f"{n}: last {st['load'][index(n)]} > {limits_load} (slår i något?)")

    return check


def move_plan(start, goal, speed):
    """Min-jerk-tid så att största ledens toppfart (1,875 × medel) håller sig under speed rad/s."""
    d = max((abs(goal[n] - start[n]) for n in goal), default=0.0)
    return max(1.0, 1.875 * d / speed)


# --------------------------------------------------------------------------------------------- servon
def cmd_audit(a):
    uno, joints, mask = connect(a)
    with uno:
        names = a.joints or [n for n in P.JOINT_NAMES if mask & (1 << index(n))]
        bad = 0
        for n in names:
            sid = joints.ids[n]
            if not mask & (1 << index(n)):
                print(f"{n:16s} ID {sid:3d}  svarar inte")
                bad += 1
                continue
            s = S.read_settings(uno, sid)
            gear = joints.extra[n].get("gear", "?")
            pr = S.problems(s)
            if n not in ("neck_pitch", "head_pitch", "head_yaw", "head_roll") and gear not in ("?", "1/345"):
                pr.append(f"utväxling {gear} i benet (ska vara 1/345, samma som ställdonsmodellen)")
            bad += bool(pr)
            print(f"{n:16s} ID {sid:3d}  modell {s['model']}  utväxling {gear:5s}  "
                  + ("ok" if not pr else "; ".join(pr)))
        print("alla ok" if not bad else f"{bad} led(er) avviker. Rätta med: openhead-body-calib fix <led>")


def cmd_fix(a):
    uno, joints, mask = connect(a)
    with uno:
        for n in pick(a.joints, mask):
            S.write_standard(uno, joints.ids[n])
            pr = S.problems(S.read_settings(uno, joints.ids[n]))
            print(f"{n}: standardinställning skriven" + (f", kvar: {'; '.join(pr)}" if pr else ""))


def cmd_gear(a):
    """
    Utväxlingen syns inte i positionen (givaren sitter på utgående axeln). Den syns i tomgångsfarten:
    servot får ett stort steg utan Unos stegbegränsning och farten mäts i Unos tillståndspaket.
    """
    uno, joints, mask = connect(a)
    with uno:
        (n,) = pick([a.joint], mask)
        if not ask_yes(a, f"{n} (ID {joints.ids[n]}) kör ett halvt varv med full fart. Servot måste sitta LÖST, "
                          "utan horn eller delar."):
            sys.exit("avbrutet")
        j = index(n)
        info = uno.info()
        volt = uno.read(joints.ids[n], S.REG_VOLTAGE, 1)[0] / 10
        uno.set_param(P.PARAM_MAX_STEP, 0)
        log = []
        try:
            st = uno.command(uno.state["pos"], torque=True, reset=True)
            start = st["pos"][j]
            goal = start + (a.steps if start < 2048 else -a.steps)
            targets = list(st["pos"])
            for k in range(int(a.seconds * 50)):
                targets[j] = goal if k >= 10 else start
                t = time.perf_counter()
                st = uno.command(targets, torque=True)
                log.append((st["t_ms"], st["pos"][j]))
                time.sleep(max(0.0, 0.02 - (time.perf_counter() - t)))
            for k in range(int(a.seconds * 50)):   # tillbaka, långsamt via stegbegränsningen nedan
                if k == 0:
                    uno.set_param(P.PARAM_MAX_STEP, info["max_step"])
                targets[j] = start
                st = uno.command(targets, torque=True)
                time.sleep(0.02)
        finally:
            release(uno)
            uno.set_param(P.PARAM_MAX_STEP, info["max_step"])
        lo, hi = min(start, goal), max(start, goal)
        a_, b_ = lo + 0.2 * (hi - lo), lo + 0.8 * (hi - lo)
        mid = [(t, p) for t, p in log if a_ <= p <= b_]
        if len(mid) < 3:
            sys.exit(f"för få mätpunkter i mitten av rörelsen ({len(mid)}); nådde servot målet? pos {log[-1][1]}")
        (t0, p0), (t1, p1) = mid[0], mid[-1]
        speed = (p1 - p0) / max(t1 - t0, 1) * 1000
        gear, ratio = S.classify_gear(speed, volt)
        print(f"{n}: {abs(speed):.0f} steg/s vid {volt:.1f} V ({math.degrees(abs(speed) / RAW_PER_RAD):.0f} °/s) "
              f"-> närmast {gear} ({S.GEARS[gear]['code']}), {ratio:.2f} × nominellt")
        if not 0.8 < ratio < 1.25:
            print("  osäkert: långt från alla nominella värden. Last på axeln, låg spänning eller annan modell?")
        joints.extra[n]["gear_test"] = {"steg_per_s": round(abs(speed)), "volt": volt, "gissning": gear}
        if a.decal:
            joints.extra[n]["gear"] = a.decal
            if a.decal != gear:
                print(f"  VARNING: dekalen säger {a.decal}, testet {gear}. Läs dekalen igen.")
        elif "gear" not in joints.extra[n]:
            print("  ange utväxlingen från dekalen med --decal så sparas den")
        joints.save(a.config)


def cmd_zero(a):
    """
    Säker körning till nolläget: momentgräns (register 48, RAM), begränsad fart, lastvakt. Utan --raw är
    målet 0 rad i simulatorn (efter tecken och offset i joints.json), med --raw rå 2048 (servots mitt,
    där hornet sätts).
    """
    uno, joints, mask = connect(a)
    with uno:
        names = pick(a.joints, mask)
        if len(names) > 1 and not ask_yes(a, f"{len(names)} leder kör till nolläget samtidigt."):
            sys.exit("avbrutet")
        start = {n: joints.to_rad(n, uno.state["pos"][index(n)]) for n in names}
        goal = {n: joints.to_rad(n, 2048) if a.raw else 0.0 for n in names}
        for n in names:
            lo, hi = LIMITS[n]
            if not lo <= goal[n] <= hi:
                print(f"OBS {n}: målet {math.degrees(goal[n]):.1f}° ligger utanför ledgränsen och klipps")
        T = move_plan(start, goal, math.radians(a.speed))
        set_torque_limit(uno, joints, names, a.torque)
        check = guard(a.max_load)
        print(f"till {'rå 2048' if a.raw else '0 rad'} på {T:.1f} s, momentgräns {a.torque / 10:.0f} %: {', '.join(names)}")

        def fn(t, st):
            check(st, names)
            s = minjerk(t / T)
            return {n: start[n] + (goal[n] - start[n]) * s for n in names}

        try:
            st, _ = run(uno, joints, mask, fn, T + a.hold)
            for n in names:
                raw = st["pos"][index(n)]
                want = joints.to_raw(n, goal[n])
                print(f"  {n:16s} rå {raw:4d} (mål {want:4d}, fel {raw - want:+d} steg = "
                      f"{math.degrees((raw - want) / RAW_PER_RAD):+.1f}°), last {st['load'][index(n)]}")
        except Abort as e:
            print(f"AVBRUTET: {e}")
        finally:
            release(uno)
            set_torque_limit(uno, joints, names, 1000)


# --------------------------------------------------------------------------------------------- huvudet
def open_phone(a, uno=None):
    if a.phone == "fake":
        return FakePhone(lambda: uno.state if uno else None)
    return PhoneUDP(a.udp)


def collect(phone, seconds):
    out, end = [], time.perf_counter() + seconds
    while time.perf_counter() < end:
        out += phone.poll()
        time.sleep(0.005)
    return out


def cmd_head_base(a):
    if a.phone == "fake":
        g = H.mv(H.T(H.PHONE_ON_BASE), H.base_gravity(FAKE_TRUE["base"]))
        samples = [{"g": g, "w": (0, 0, 0)}] * 50
    else:
        phone = open_phone(a)
        input("Lägg telefonen på basplattan, skärmen upp, överkanten framåt (+x). Enter när den ligger still... ")
        samples = collect(phone, a.seconds)
        phone.close()
    s = summarize(samples)
    if not s:
        sys.exit(f"inga data från telefonen på UDP-port {a.udp}")
    bx, by = H.base_tilt(s["g"])
    print(f"stativets lutning: kring x {math.degrees(bx):+.2f}°, kring y {math.degrees(by):+.2f}° "
          f"({s['n']} prover, spridning {s['g_std'] * 1000:.1f} mg)")
    CAL_DIR.mkdir(parents=True, exist_ok=True)
    Path(a.out).write_text(json.dumps({"datum": datetime.date.today().isoformat(), "bas_rad": [bx, by],
                                       "g": s["g"]}, indent=2) + "\n", encoding="utf-8")
    print(f"sparat: {a.out}")


def cmd_head_plan(a):
    model = H.HeadModel(mount=H.load_mount())
    print("positiv riktning per led (från CAD-axlarna; kontrollera tecknet med openhead-body-bench move <led> 10):")
    words = {(0, 0, 1): "uppåt", (0, 0, -1): "nedåt", (0, 1, 0): "åt vänster", (0, -1, 0): "åt höger",
             (1, 0, 0): "framåt", (-1, 0, 0): "bakåt"}

    def dirword(v):
        k = max(range(3), key=lambda i: abs(v[i]))
        e = [0, 0, 0]
        e[k] = 1 if v[k] > 0 else -1
        return words[tuple(e)]

    for n in H.NECK:
        d = model.positive_direction(n)
        part = "skärmen vrids " + dirword(d["skarm"]) if max(map(abs, d["skarm"])) > 1e-6 else \
            "överkanten (telefonens +y) går " + dirword(d["overkant"])
        print(f"  {n:11s} +10°: {part}")
    print(f"\nmätposer (grader), {len(H.plan_poses())} st:")
    for k, p in enumerate(H.plan_poses()):
        print(f"  {k + 1:2d}  " + "  ".join(f"{n} {math.degrees(v):+5.0f}" for n, v in p.items()))


def cmd_head_capture(a):
    uno, joints, mask = connect(a)
    phone = open_phone(a, uno)
    names = list(H.NECK)
    samples = []
    with uno:
        pick(names, mask)
        plan = H.plan_poses()
        poses = [(i, plan[i - 1]) for i in (a.only or range(1, len(plan) + 1))]
        print(f"{len(poses)} poser, närmar sig varje pose från {'båda hållen' if a.both else 'samma håll'}. "
              f"Telefonen ska skicka till UDP-port {a.udp}. Ctrl+C avbryter.")
        directions = (1, -1) if a.both else (1,)
        set_torque_limit(uno, joints, names, a.torque)
        check = guard(a.max_load)
        try:
            for k, pose in poses:
                for d in directions:
                    s = capture_pose(uno, joints, mask, phone, pose, d, a, check)
                    s["pose"] = k
                    samples.append(s)
                    q = s["summary"]
                    flag = "" if q["w_rms"] < 0.02 and q["g_std"] < 0.005 else "  RÖRDE SIG?"
                    print(f"  pose {k:2d}{'+' if d > 0 else '-'}: g = ({q['g'][0]:+.3f}, {q['g'][1]:+.3f}, "
                          f"{q['g'][2]:+.3f}), {q['n']} prover, rotation {q['w_rms']:.3f} rad/s{flag}")
        except (Abort, KeyboardInterrupt) as e:
            print(f"AVBRUTET: {e}")
        finally:
            release(uno)
            set_torque_limit(uno, joints, names, 1000)
            phone.close()
    if not samples:
        sys.exit("inga mätningar")
    CAL_DIR.mkdir(parents=True, exist_ok=True)
    out = a.out or str(CAL_DIR / f"huvud_{datetime.datetime.now():%Y%m%d_%H%M}.json")
    meta = {"datum": datetime.datetime.now().isoformat(timespec="seconds"), "telefon": a.phone or "udp",
            "joints": {n: {"id": joints.ids[n], "sign": joints.sign[n], "offset_rad": joints.offset[n]} for n in names}}
    Path(out).write_text(json.dumps({"meta": meta, "samples": samples}, indent=1) + "\n", encoding="utf-8")
    print(f"sparat: {out}  ->  openhead-body-calib head-fit {out}")


def capture_pose(uno, joints, mask, phone, pose, d, a, check):
    """
    Kör till posen via en förpunkt `approach` grader på ena sidan (riktning d i råenheter), väntar `settle`
    s och mäter i `measure` s. Råpositionen är servots uppmätta (inte det kommenderade målet).
    """
    names = list(pose)
    st0 = uno.state
    start = {n: joints.to_rad(n, st0["pos"][index(n)]) for n in names}
    # förpunkten ligger d·approach råenheter före målet, uttryckt i rad med nuvarande tecken
    pre = {n: joints.clamp(n, pose[n] - d * joints.sign[n] * math.radians(a.approach)) for n in names}
    T1 = move_plan(start, pre, math.radians(a.speed))
    T2 = move_plan(pre, pose, math.radians(a.speed))
    t_meas = T1 + T2 + a.settle
    got, raws = [], []

    def fn(t, st):
        check(st, names)
        new = phone.poll()
        if t >= t_meas:
            got.extend(new)
            raws.append([st["pos"][index(n)] for n in names])
        if t < T1:
            s = minjerk(t / T1)
            return {n: start[n] + (pre[n] - start[n]) * s for n in names}
        s = minjerk((t - T1) / T2)
        return {n: pre[n] + (pose[n] - pre[n]) * s for n in names}

    run(uno, joints, mask, fn, t_meas + a.measure)
    summ = summarize(got) or {"g": [0, 0, 0], "g_std": 1, "w_rms": 1, "n": 0}
    raw = {n: round(sum(r[i] for r in raws) / len(raws)) for i, n in enumerate(names)} if raws else {}
    dir_raw = {n: (1 if joints.to_raw(n, pose[n]) > joints.to_raw(n, pre[n]) else
                   -1 if joints.to_raw(n, pose[n]) < joints.to_raw(n, pre[n]) else 0) for n in names}
    return {"target_rad": pose, "raw": raw, "dir_raw": dir_raw, "g": summ["g"], "summary": summ}


def cmd_head_fit(a):
    files = sorted(f for p in a.files for f in glob.glob(p))
    if not files:
        sys.exit("inga mätfiler")
    samples = []
    for f in files:
        d = json.loads(Path(f).read_text(encoding="utf-8"))
        samples += [s for s in d["samples"] if s["summary"]["n"] > 0]
    joints = Joints.load(a.config)
    base = (0.0, 0.0)
    if Path(a.base).exists():
        base = tuple(json.loads(Path(a.base).read_text(encoding="utf-8"))["bas_rad"])
    else:
        print(f"OBS: {a.base} saknas – stativets lutning kring y antas 0 (hamnar i head_pitch-offset)")
    both = any(s["dir_raw"].get("head_pitch", 1) < 0 for s in samples) and \
        any(s["dir_raw"].get("head_pitch", 1) > 0 for s in samples)
    fit_offsets = [n for n in H.NECK if n != "neck_pitch"] if not a.neck else [n for n in H.NECK if n != "head_pitch"]
    model = H.HeadModel(mount=H.load_mount())
    f = H.fit(samples, model, signs=dict(joints.sign), offsets=dict(joints.offset), fit_offsets=fit_offsets,
              fit_scale=H.NECK[1:] if a.scale else (), fit_backlash=H.NECK if (a.backlash and both) else (),
              base=base, max_offset=math.radians(a.max_offset))
    print(f"{len(samples)} mätningar ur {len(files)} fil(er). Residual RMS {math.degrees(f.rms):.2f}°, "
          f"störst {math.degrees(f.max_err):.2f}°"
          + (f", näst bästa teckenkombination {math.degrees(f.second_rms):.1f}°" if f.second_rms else ""))
    print(f"stativ: kring x {math.degrees(f.base[0]):+.2f}° (anpassad), kring y {math.degrees(f.base[1]):+.2f}° (mätt)")
    print(f"  {'led':11s} {'tecken':>6s} {'offset':>8s} {'±':>5s}   {'skala':>6s} {'glapp':>6s}")
    for n in H.NECK:
        sd = f.std.get(f"offset:{n}")
        print(f"  {n:11s} {f.signs[n]:+6d} {math.degrees(f.offset[n]):+7.2f}° "
              f"{'' if sd is None else f'{math.degrees(sd):4.2f}°':>5s}   {f.scale[n]:6.3f} "
              f"{math.degrees(f.backlash[n]):5.2f}°" + ("   (från mallen)" if n not in fit_offsets else "")
              + ("   TECKEN ÄNDRAT" if f.signs[n] != joints.sign[n] else ""))
    worst = sorted(range(len(samples)), key=lambda i: -f.per_sample[i])[:3]
    print("största residualer: " + ", ".join(f"pose {samples[i].get('pose', i + 1)} {math.degrees(f.per_sample[i]):.2f}°"
                                             for i in worst))
    if f.second_rms and f.second_rms < 3 * f.rms:
        print("VARNING: tecknen är inte entydiga. Lägg till poser eller kontrollera tecknen med openhead-body-bench move.")
    if a.scale and any(abs(f.scale[n] - 1) > 0.02 for n in H.NECK[1:]):
        print("VARNING: skala > 2 % från 1. Givaren sitter på utgående axeln; slirar hornet eller böjer delen?")
    if f.rms > math.radians(1.0):
        print("VARNING: residual > 1°. Glapp, flex, telefonen lös i hållaren eller fel montering (placering.json)?")
    if a.save:
        stamp = datetime.date.today().isoformat()
        for n in H.NECK:
            joints.sign[n] = f.signs[n]
            joints.offset[n] = f.offset[n]
            if n in fit_offsets:
                joints.extra[n]["kalibrering"] = {
                    "metod": "telefonens gravitation", "datum": stamp, "rms_grader": round(math.degrees(f.rms), 2),
                    "glapp_grader": round(math.degrees(f.backlash[n]), 2), "skala": round(f.scale[n], 4)}
        joints.save(a.config)
        print(f"sparat: {a.config}")
    else:
        print("(torrkörning: --save skriver tecken och offset till joints.json)")


# --------------------------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="COM-port, eller 'fake' (simulerad Uno med servon på ID 30–33)")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--config", default=str(CONFIG_PATH), help="ledtabell (ID, tecken, offset)")
    ap.add_argument("--yes", action="store_true", help="fråga inte före rörelser")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("audit", help="kontrollera servonas register mot standardinställningen")
    p.add_argument("joints", nargs="*")
    p = sub.add_parser("fix", help="skriv standardinställningen (momentet av)")
    p.add_argument("joints", nargs="+")
    p = sub.add_parser("gear", help="hastighetstest av utväxlingen (servot löst)")
    p.add_argument("joint")
    p.add_argument("--decal", choices=sorted(S.GEARS), help="utväxling enligt dekalen, sparas i joints.json")
    p.add_argument("--steps", type=int, default=1536, help="stegets längd i råenheter (1536 = 135°)")
    p.add_argument("--seconds", type=float, default=1.5)
    p = sub.add_parser("zero", help="kör leder långsamt och med momentgräns till nolläget")
    p.add_argument("joints", nargs="*")
    p.add_argument("--raw", action="store_true", help="till rå 2048 i stället för 0 rad")
    p.add_argument("--speed", type=float, default=20.0, help="toppfart °/s")
    p.add_argument("--torque", type=int, default=300, help="momentgräns 0–1000 (promille)")
    p.add_argument("--max-load", type=int, default=600, help="avbryt över denna last (promille)")
    p.add_argument("--hold", type=float, default=3.0, help="håll i s innan momentet släpps (Ctrl+C släpper)")
    for name, hlp in (("head-base", "stativets lutning med telefonen på basplattan"),
                      ("head-capture", "kör mätposerna och logga telefonens gravitation"),
                      ("head-plan", "visa mätposerna och ledernas positiva riktning")):
        p = sub.add_parser(name, help=hlp)
        if name == "head-plan":
            continue
        p.add_argument("--phone", choices=["fake"], help="simulerad telefon")
        p.add_argument("--udp", type=int, default=PORT, help="UDP-port telefonen skickar till")
        if name == "head-base":
            p.add_argument("--seconds", type=float, default=3.0)
            p.add_argument("--out", default=str(BASE_FILE))
        else:
            p.add_argument("--out")
            p.add_argument("--only", type=int, nargs="*", help="bara dessa poser (1-baserat)")
            p.add_argument("--both", action="store_true", help="närma sig varje pose från båda hållen (glapp)")
            p.add_argument("--approach", type=float, default=4.0, help="förpunkt, grader före posen")
            p.add_argument("--speed", type=float, default=30.0, help="toppfart °/s")
            p.add_argument("--settle", type=float, default=1.0)
            p.add_argument("--measure", type=float, default=1.0)
            p.add_argument("--torque", type=int, default=500, help="momentgräns 0–1000")
            p.add_argument("--max-load", type=int, default=800)
    p = sub.add_parser("head-fit", help="anpassa tecken, offset (och glapp/skala) ur mätfilerna")
    p.add_argument("files", nargs="+")
    p.add_argument("--base", default=str(BASE_FILE), help="stativets lutning från head-base")
    p.add_argument("--neck", action="store_true", help="anpassa neck_pitch och lås head_pitch i stället")
    p.add_argument("--scale", action="store_true", help="anpassa också skalan per led (kontroll, sparas inte)")
    p.add_argument("--backlash", action="store_true", help="anpassa glapp (kräver head-capture --both)")
    p.add_argument("--max-offset", type=float, default=25.0, help="största rimliga offset, grader")
    p.add_argument("--save", action="store_true")
    a = ap.parse_args()
    try:
        globals()["cmd_" + a.cmd.replace("-", "_")](a)
    except (LinkError, Abort) as e:
        sys.exit(f"fel: {e}")


if __name__ == "__main__":
    main()
