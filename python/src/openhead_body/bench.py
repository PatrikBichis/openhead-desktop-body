"""
Bänkverktyg: dator -> USB -> Uno R4 WiFi -> Waveshare-adapter (läge B) -> STS3215-servon.

    python openhead-body-bench ports                       # hitta kortet
    python openhead-body-bench info                        # firmware, parametrar, leder, bussspänning
    python openhead-body-bench scan                        # vilka servo-ID svarar
    python openhead-body-bench set-id 1 22                 # byt ID (ett servo i taget på bussen)
    python openhead-body-bench move left_knee 20           # led till 20° (mjuk rörelse), släpp sedan
    python openhead-body-bench sweep left_knee --amp 10    # sinus, mäter fördröjning och följfel
    python openhead-body-bench watchdog                    # sluta skicka -> momentet ska släppa inom 150 ms
    python openhead-body-bench imu                         # kroppens BNO085: vickningstest av axlar och tecken
    python openhead-body-bench reflex on                   # ladda upp fallreflexens poser och slå på den (35°)

--port fake kör mot en simulerad Uno (openhead_body/fake.py). Vinklar anges i grader i simulatorns
konvention (efter tecken och offset i joints.json). Se host/README.md för hela bänktestet.
"""
import argparse
import csv
import math
import statistics
import sys
import time

from openhead_body import fall_reflex
from openhead_body import protocol as P
from openhead_body.joints import CONFIG_PATH, DEFAULT_POSE, LIMITS, Joints, index
from openhead_body.link import (
    REG_ACC,
    REG_GOAL_SPEED,
    REG_ID,
    REG_MAX_ANGLE,
    REG_MIN_ANGLE,
    REG_MODE,
    REG_OFFSET,
    REG_P,
    REG_PRESENT_POS,
    REG_TORQUE,
    REG_VOLTAGE,
    LinkError,
    Uno,
    find_port,
)

RATE = 50.0


# --------------------------------------------------------------------------------------------- hjälp
def connect(a, push_ids=True):
    uno = Uno.open(a.port, a.baud)
    joints = Joints.load(a.config)
    ids, mask = uno.get_ids()
    if push_ids and ids != joints.id_list():
        ids, mask = uno.set_ids(joints.id_list())
    return uno, joints, mask


def active_names(mask):
    return [n for j, n in enumerate(P.JOINT_NAMES) if mask & (1 << j)]


def require_active(name, mask):
    j = index(name)
    if not mask & (1 << j):
        sys.exit(f"{name} (servo-ID enligt joints.json) svarar inte. Kör scan och config --id {name}=<ID>.")
    return j


def minjerk(s):
    s = min(max(s, 0.0), 1.0)
    return s ** 3 * (10 - 15 * s + 6 * s * s)


def run(uno, joints, mask, fn, seconds, log=None):
    """50 Hz-loop med momentet på. fn(t, state) -> {led: rad}. Leder som inte anges håller sin position."""
    st = uno.state
    hold = {n: joints.to_rad(n, st["pos"][index(n)]) for n in active_names(mask)}
    t0 = time.perf_counter()
    k = 0
    worst = 0.0
    try:
        while True:
            t = time.perf_counter() - t0
            if t > seconds:
                break
            want = {**hold, **fn(t, st)}
            targets = [joints.to_raw(n, want[n]) if n in want else 2048 for n in P.JOINT_NAMES]
            st = uno.command(targets, torque=True)
            if st["status"] & (P.ST_WATCHDOG | P.ST_ESTOP):
                raise LinkError("Uno har stängt av momentet (watchdog/nödstopp). Kör torque-off --reset.")
            if log is not None:
                log.append((t, want, st))
            k += 1
            nxt = t0 + k / RATE
            worst = max(worst, time.perf_counter() - nxt)
            while time.perf_counter() < nxt:
                time.sleep(0.0005)
    except KeyboardInterrupt:
        print("\navbrutet")
    return st, worst


def release(uno):
    try:
        uno.torque_off()
    except LinkError:
        pass


def fmt_status(s):
    names = [(P.ST_TORQUE, "moment"), (P.ST_WATCHDOG, "WATCHDOG"), (P.ST_ESTOP, "NÖDSTOPP"),
             (P.ST_LOW_VOLTAGE, "LÅG SPÄNNING"), (P.ST_SERVO_ERROR, "SERVOFEL"), (P.ST_FALL, "FALLREFLEX"),
             (P.ST_IMU_OK, "imu")]
    return ", ".join(n for b, n in names if s & b) or "ok"


def write_csv(path, log, names, joints):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t"] + [f"{n}_{c}" for n in names for c in ("mal_rad", "pos_rad", "vel_rad_s", "last")])
        for t, want, st in log:
            row = [f"{t:.4f}"]
            for n in names:
                j = index(n)
                row += [f"{want[n]:.5f}", f"{joints.to_rad(n, st['pos'][j]):.5f}",
                        f"{joints.vel_to_rad(n, st['vel'][j]):.4f}", st["load"][j]]
            w.writerow(row)
    print(f"logg: {path}")


# --------------------------------------------------------------------------------------------- kommandon
def cmd_ports(a):
    from serial.tools import list_ports
    for p in list_ports.comports():
        tag = "  <- Arduino" if p.vid == 0x2341 else ""
        print(f"{p.device:8s} {p.description} (VID {p.vid or 0:04X} PID {p.pid or 0:04X}){tag}")
    print("vald automatiskt:", find_port())


def cmd_info(a):
    uno, joints, mask = connect(a)
    with uno:
        i = uno.info()
        print(f"firmware v{i['fw']}, watchdog {i['watchdog_ms']} ms, max_step {i['max_step']} rå/kommando, "
              f"lågspänning {i['low_mv'] / 1000:.1f} V, eko på bussen: {'ja' if i['bus_echo'] else 'nej'}")
        st = uno.state
        print(f"status: {fmt_status(st['status'])}, buss {st['bus_mv'] / 1000:.1f} V, fötter {st['feet']:04b}")
        for j, n in enumerate(P.JOINT_NAMES):
            on = mask & (1 << j)
            pos = f"{st['pos'][j]:5d} rå = {math.degrees(joints.to_rad(n, st['pos'][j])):7.1f}°" if on else ""
            print(f"  {n:16s} ID {joints.ids[n]:3d}  {'aktiv ' if on else 'saknas'} {pos}")


def cmd_scan(a):
    with Uno.open(a.port, a.baud) as uno:
        ids = uno.scan()
        if not ids:
            print("inga servon svarade. Kontrollera matning (≥ 6 V), bygel i läge B, TX/RX korsade och gemensam jord.")
        joints = Joints.load(a.config)
        by_id = {v: k for k, v in joints.ids.items()}
        for i in ids:
            model = uno.ping(i)
            pos = uno.read_u16(i, REG_PRESENT_POS)
            v = uno.read(i, REG_VOLTAGE, 2)
            print(f"ID {i:3d}  modell {model}  pos {pos:4d}  {v[0] / 10:.1f} V  {v[1]} °C  {by_id.get(i, '(ingen led)')}")
        if 1 in ids and len(ids) > 1:
            print("OBS: ID 1 finns tillsammans med andra. Nya servon har ID 1 – byt ID med bara ett servo inkopplat.")


def cmd_regs(a):
    with Uno.open(a.port, a.baud) as uno:
        r = lambda addr, n=1: uno.read(a.id, addr, n)
        u16 = lambda addr: uno.read_u16(a.id, addr)
        print(f"ID {r(REG_ID)[0]}, modell {uno.ping(a.id)}")
        print(f"vinkelgränser {u16(REG_MIN_ANGLE)}..{u16(REG_MAX_ANGLE)}, offset {u16(REG_OFFSET)}, läge {r(REG_MODE)[0]}")
        p, d, i = r(REG_P, 3)
        print(f"P {p}, D {d}, I {i}, acc {r(REG_ACC)[0]}, målhastighet {u16(REG_GOAL_SPEED)}, moment {r(REG_TORQUE)[0]}")
        v, t = r(REG_VOLTAGE, 2)
        print(f"position {u16(REG_PRESENT_POS)}, {v / 10:.1f} V, {t} °C")


def cmd_set_id(a):
    with Uno.open(a.port, a.baud) as uno:
        if uno.ping(a.old) is None:
            sys.exit(f"inget servo svarar på ID {a.old}")
        if uno.ping(a.new) is not None:
            sys.exit(f"ID {a.new} är redan upptaget")
        uno.write_eeprom(a.old, REG_ID, bytes([a.new]))
        time.sleep(0.05)
        ok = uno.ping(a.new) is not None and uno.ping(a.old) is None
        print(f"ID {a.old} -> {a.new}: {'klart' if ok else 'MISSLYCKADES, kör scan'}")
        uno.set_ids(Joints.load(a.config).id_list())   # Uno räknar om vilka leder som finns


def cmd_configure(a):
    """Som Open Duck Runtimes configure_motor.py: positionsläge, P/I/D, acceleration 0."""
    with Uno.open(a.port, a.baud) as uno:
        uno.write_eeprom(a.id, REG_MODE, b"\x00")
        uno.write_eeprom(a.id, REG_P, bytes([a.p, a.d, a.i]))
        uno.write(a.id, REG_ACC, b"\x00")
        p, d, i = uno.read(a.id, REG_P, 3)
        lo, hi = uno.read_u16(a.id, REG_MIN_ANGLE), uno.read_u16(a.id, REG_MAX_ANGLE)
        print(f"ID {a.id}: läge 0, P {p} D {d} I {i}, acc 0. Vinkelgränser {lo}..{hi}"
              + ("" if (lo, hi) == (0, 4095) else "  (begränsade – kan hindra rörelsen)"))


def cmd_config(a):
    joints = Joints.load(a.config)
    for spec, field in ((a.id, "ids"), (a.sign, "sign"), (a.offset, "offset")):
        for s in spec or []:
            n, v = s.split("=")
            index(n)
            getattr(joints, field)[n] = float(v) if field == "offset" else int(v)
    if a.id or a.sign or a.offset:
        joints.save(a.config)
        print(f"sparat: {a.config}")
    for n in P.JOINT_NAMES:
        lo, hi = LIMITS[n]
        print(f"  {n:16s} ID {joints.ids[n]:3d}  tecken {joints.sign[n]:+d}  offset {math.degrees(joints.offset[n]):6.2f}°"
              f"  gränser {math.degrees(lo):6.1f}..{math.degrees(hi):6.1f}°")


def cmd_offsets(a):
    """Som find_soft_offsets.py: momentet av, ställ lederna i nolläge för hand, spara skillnaden."""
    uno, joints, mask = connect(a)
    with uno:
        release(uno)
        names = [n for n in (a.joints or active_names(mask)) if mask & (1 << index(n))]
        input(f"Momentet är av. Ställ {', '.join(names)} i nolläge (0° i simulatorn) och tryck Enter... ")
        uno.wait_for(lambda: False, 0.2)
        for n in names:
            raw = uno.state["pos"][index(n)]
            joints.offset[n] = (raw - 2048) / (4096 / (2 * math.pi)) * joints.sign[n]
            print(f"  {n:16s} rå {raw:4d} -> offset {math.degrees(joints.offset[n]):6.2f}°")
        joints.save(a.config)
        print(f"sparat: {a.config}")


def cmd_monitor(a):
    uno, joints, mask = connect(a)
    names = active_names(mask)
    with uno:
        end = time.perf_counter() + (a.seconds or 1e9)
        try:
            while time.perf_counter() < end:
                uno.wait_for(lambda: False, 0.2)
                st = uno.state
                parts = [f"{n.replace('_', ' ')[:10]:10s} {math.degrees(joints.to_rad(n, st['pos'][index(n)])):6.1f}°"
                         f" L{st['load'][index(n)]:5d}" for n in names]
                print(f"[{fmt_status(st['status'])}] {st['bus_mv'] / 1000:4.1f} V fötter {st['feet']:04b} | " + " | ".join(parts))
        except KeyboardInterrupt:
            pass


def cmd_torque_off(a):
    with Uno.open(a.port, a.baud) as uno:
        st = uno.command(uno.state["pos"], torque=False, reset=a.reset)
        print("momentet av" + (", larm nollställda" if a.reset else "") + f" ({fmt_status(st['status'])})")


def cmd_latency(a):
    """Tur och retur kommando -> tillstånd, med momentet av."""
    with Uno.open(a.port, a.baud) as uno:
        targets = uno.state["pos"]
        rtt, lost = [], 0
        for _ in range(a.n):
            t = time.perf_counter()
            try:
                uno.command(targets, torque=False, wait=0.2)
                rtt.append((time.perf_counter() - t) * 1000)
            except LinkError:
                lost += 1
            time.sleep(max(0.0, 1 / RATE - (time.perf_counter() - t)))
        rtt.sort()
        print(f"{len(rtt)} svar, {lost} förlorade. Tur och retur: median {statistics.median(rtt):.2f} ms, "
              f"medel {statistics.mean(rtt):.2f}, p95 {rtt[int(0.95 * len(rtt)) - 1]:.2f}, max {rtt[-1]:.2f} ms")
        print(f"tappade byte i länken: {uno.parser.dropped}")


def cmd_watchdog(a):
    uno, joints, mask = connect(a)
    with uno:
        wd = uno.info()["watchdog_ms"]
        print(f"håller nuvarande position med momentet på i 1 s, slutar sedan skicka (watchdog {wd} ms)")
        run(uno, joints, mask, lambda t, st: {}, 1.0)
        t_stop = time.perf_counter()
        got = uno.wait_for(lambda: uno.state["status"] & P.ST_WATCHDOG and not uno.state["status"] & P.ST_TORQUE,
                           1.0)
        dt = (time.perf_counter() - t_stop) * 1000
        ok1 = got and dt < wd + 150   # plus tid för nästa tillgångspaket (10 Hz)
        print(f"1. momentet släppt och watchdog låst efter {dt:.0f} ms: {'OK' if ok1 else 'FEL'}")
        st = uno.command(uno.state["pos"], torque=True)
        ok2 = not st["status"] & P.ST_TORQUE
        print(f"2. moment begärt utan nollställning -> förblir av: {'OK' if ok2 else 'FEL'}")
        st = uno.command(uno.state["pos"], torque=False, reset=True)
        ok3 = not st["status"] & (P.ST_WATCHDOG | P.ST_TORQUE)
        print(f"3. nollställning: {'OK' if ok3 else 'FEL'}")
        st = uno.command(uno.state["pos"], torque=False, estop=True)
        st = uno.command(uno.state["pos"], torque=True)
        ok4 = bool(st["status"] & P.ST_ESTOP) and not st["status"] & P.ST_TORQUE
        uno.command(uno.state["pos"], torque=False, reset=True)
        print(f"4. nödstopp låser momentet av: {'OK' if ok4 else 'FEL'}")
        print("watchdogtest:", "GODKÄNT" if ok1 and ok2 and ok3 and ok4 else "UNDERKÄNT")


def cmd_move(a):
    uno, joints, mask = connect(a)
    j = require_active(a.joint, mask)
    with uno:
        start = joints.to_rad(a.joint, uno.state["pos"][j])
        goal = joints.clamp(a.joint, math.radians(a.deg))
        if abs(goal - math.radians(a.deg)) > 1e-6:
            print(f"klippt till ledgränsen {math.degrees(goal):.1f}°")
        T = a.time
        log = []
        try:
            st, worst = run(uno, joints, mask,
                            lambda t, st: {a.joint: start + (goal - start) * minjerk(t / T)}, T + a.hold, log)
            got = joints.to_rad(a.joint, st["pos"][j])
            print(f"{a.joint}: {math.degrees(start):.1f}° -> mål {math.degrees(goal):.1f}°, uppmätt {math.degrees(got):.1f}° "
                  f"(fel {math.degrees(got - goal):+.2f}°), last {st['load'][j]}, sämsta loopfördröjning {worst * 1000:.1f} ms")
        finally:
            if not a.keep:
                release(uno)
        if a.csv:
            write_csv(a.csv, log, [a.joint], joints)


def cmd_sweep(a):
    uno, joints, mask = connect(a)
    j = require_active(a.joint, mask)
    with uno:
        c = joints.to_rad(a.joint, uno.state["pos"][j]) if a.center is None else math.radians(a.center)
        A, f = math.radians(a.amp), a.freq
        ramp = 1.0
        dur = a.cycles / f + 2 * ramp

        def fn(t, st):   # amplituden rampas in och ut så att start och slut blir mjuka
            env = min(1.0, t / ramp, max(0.0, (dur - t) / ramp))
            return {a.joint: c + env * A * math.sin(2 * math.pi * f * t)}

        log = []
        try:
            run(uno, joints, mask, fn, dur, log)
        finally:
            release(uno)
        body = [(t, w[a.joint], joints.to_rad(a.joint, st["pos"][j])) for t, w, st in log if ramp < t < dur - ramp]
        if len(body) > 10:
            tgt = [x[1] for x in body]
            meas = [x[2] for x in body]
            # fördröjning: förskjutning (i loopsteg) som ger minst kvadratfel, förfinad med parabel
            errs = [sum((meas[i] - tgt[i - k]) ** 2 for i in range(k, len(tgt))) / (len(tgt) - k) for k in range(0, 15)]
            k = min(range(len(errs)), key=errs.__getitem__)
            frac = 0.0
            if 0 < k < len(errs) - 1:
                den = errs[k - 1] - 2 * errs[k] + errs[k + 1]
                frac = 0.5 * (errs[k - 1] - errs[k + 1]) / den if den else 0.0
            lag_ms = (k + frac) / RATE * 1000
            rms = math.sqrt(sum((m - t) ** 2 for m, t in zip(meas, tgt)) / len(tgt))
            amp_meas = (max(meas) - min(meas)) / 2
            print(f"{a.joint}: {a.amp:.1f}° vid {f:.2f} Hz kring {math.degrees(c):.1f}°. Uppmätt amplitud "
                  f"{math.degrees(amp_meas):.1f}°, fördröjning ≈ {lag_ms:.0f} ms (kommando -> uppmätt position, "
                  f"inkl. länken), följfel RMS {math.degrees(rms):.2f}°")
        if a.csv:
            write_csv(a.csv, log, [a.joint], joints)


def cmd_home(a):
    uno, joints, mask = connect(a)
    names = active_names(mask)
    if not names:
        sys.exit("inga aktiva leder")
    with uno:
        start = {n: joints.to_rad(n, uno.state["pos"][index(n)]) for n in names}
        T = a.time
        print(f"till standardpose på {T:.1f} s: {', '.join(names)}")
        try:
            run(uno, joints, mask, lambda t, st: {n: start[n] + (DEFAULT_POSE[n] - start[n]) * minjerk(t / T)
                                                  for n in names}, T + a.hold)
        finally:
            release(uno)


def body_imu(st):
    """(gyro rad/s, acc m/s²) i robotens ram ur tillståndet, eller None utan IMU."""
    if not st["status"] & P.ST_IMU_OK:
        return None
    return [v / P.IMU_GYRO_SCALE for v in st["imu"][:3]], [v / P.IMU_ACC_SCALE for v in st["imu"][3:]]


def cmd_imu(a):
    with Uno.open(a.port, a.baud) as uno:
        end = time.perf_counter() + (a.seconds or 1e9)
        try:
            while time.perf_counter() < end:
                uno.wait_for(lambda: False, 0.2)
                imu = body_imu(uno.state)
                if imu is None:
                    print("ingen IMU (ST_IMU_OK saknas): kolla kabel, Wire1/Qwiic och adress i config.h")
                    continue
                g, acc = imu
                n = math.sqrt(sum(x * x for x in acc)) or 1.0
                tilt = math.degrees(math.acos(max(-1.0, min(1.0, acc[2] / n))))
                lean = ("framåt" if acc[0] < 0 else "bakåt") if abs(acc[0]) >= abs(acc[1]) else \
                       ("vänster" if acc[1] < 0 else "höger")
                print(f"acc {acc[0]:6.2f} {acc[1]:6.2f} {acc[2]:6.2f} m/s²  gyro {g[0]:5.2f} {g[1]:5.2f} {g[2]:5.2f} "
                      f"rad/s  lutning {tilt:5.1f}° {lean if tilt > 3 else ''}")
        except KeyboardInterrupt:
            pass


def cmd_reflex(a):
    uno, joints, mask = connect(a)
    with uno:
        if a.action == "on":
            fall_reflex.upload(uno, joints, a.deg, a.horizon_ms, a.limp_ms)
        elif a.action == "off":
            uno.set_param(P.PARAM_FALL_DEG, 0)
        i = uno.info()
        state = f"på, {i['fall_deg']}° med prognos {i['fall_horizon_ms']} ms" if i["fall_deg"] else "av"
        limp = f"momentet av efter {i['fall_limp_ms']} ms" if i["fall_limp_ms"] else "håller reflexposen"
        print(f"fallreflex {state}, {limp}; status: {fmt_status(uno.state['status'])}")
        for d, name in fall_reflex.DIR_NAMES.items():
            raw = uno.get_reflex(d)
            parts = [f"{n.replace('_', ' ')} {math.degrees(joints.to_rad(n, r)):.0f}°"
                     for n, r in zip(P.JOINT_NAMES, raw) if r != P.REFLEX_KEEP and mask & (1 << index(n))]
            print(f"  {name:8s} " + (", ".join(parts) if parts else "(ingen pose för aktiva leder)"))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="COM-port, eller 'fake' för simulerad Uno (standard: första Arduino-kortet)")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--config", default=str(CONFIG_PATH), help="ledtabell (ID, tecken, offset)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ports", help="lista seriella portar")
    sub.add_parser("info", help="firmware, parametrar, leder och status")
    sub.add_parser("scan", help="leta servon på bussen (momentet måste vara av)")
    p = sub.add_parser("regs", help="läs ett servos viktigaste register")
    p.add_argument("id", type=int)
    p = sub.add_parser("set-id", help="byt servo-ID (ha bara det servot inkopplat om flera har samma ID)")
    p.add_argument("old", type=int)
    p.add_argument("new", type=int)
    p = sub.add_parser("configure", help="positionsläge, P/I/D, acceleration 0 (som Open Ducks configure_motor)")
    p.add_argument("id", type=int)
    p.add_argument("--p", type=int, default=32)
    p.add_argument("--i", type=int, default=0)
    p.add_argument("--d", type=int, default=0)
    p = sub.add_parser("config", help="visa/ändra joints.json")
    p.add_argument("--id", nargs="*", metavar="LED=ID")
    p.add_argument("--sign", nargs="*", metavar="LED=±1")
    p.add_argument("--offset", nargs="*", metavar="LED=RAD")
    p = sub.add_parser("offsets", help="mät offset: ställ lederna i nolläge för hand")
    p.add_argument("joints", nargs="*")
    p = sub.add_parser("monitor", help="visa tillståndet löpande")
    p.add_argument("--seconds", type=float)
    p = sub.add_parser("torque-off", help="stäng av momentet")
    p.add_argument("--reset", action="store_true", help="nollställ också watchdog- och nödstoppslarm")
    p = sub.add_parser("latency", help="mät tur och retur dator -> Uno -> servon -> dator")
    p.add_argument("--n", type=int, default=250)
    sub.add_parser("watchdog", help="testa watchdog, låsning och nödstopp")
    p = sub.add_parser("move", help="flytta en led mjukt till en vinkel (grader)")
    p.add_argument("joint")
    p.add_argument("deg", type=float)
    p.add_argument("--time", type=float, default=1.5)
    p.add_argument("--hold", type=float, default=1.0)
    p.add_argument("--keep", action="store_true", help="behåll momentet (släpps ändå av watchdog när verktyget slutar)")
    p.add_argument("--csv")
    p = sub.add_parser("sweep", help="sinusrörelse, mäter fördröjning och följfel")
    p.add_argument("joint")
    p.add_argument("--amp", type=float, default=10.0, help="grader")
    p.add_argument("--freq", type=float, default=0.5, help="Hz")
    p.add_argument("--cycles", type=float, default=4)
    p.add_argument("--center", type=float, help="grader (standard: nuvarande position)")
    p.add_argument("--csv")
    p = sub.add_parser("home", help="alla aktiva leder till standardposen (= home i MJCF)")
    p.add_argument("--time", type=float, default=3.0)
    p.add_argument("--hold", type=float, default=2.0)
    p = sub.add_parser("imu", help="visa kroppens IMU (BNO085): lutning och riktning, för vickningstestet")
    p.add_argument("--seconds", type=float)
    p = sub.add_parser("reflex", help="fallreflexen: visa, ladda upp poser och slå på, eller stäng av")
    p.add_argument("action", choices=["show", "on", "off"])
    p.add_argument("--deg", type=int, default=fall_reflex.DEFAULTS["deg"], help="gräns i grader (prognos)")
    p.add_argument("--horizon-ms", type=int, default=fall_reflex.DEFAULTS["horizon_ms"])
    p.add_argument("--limp-ms", type=int, default=fall_reflex.DEFAULTS["limp_ms"],
                   help="momentet av så här länge efter reflexen (0 = håll kvar)")
    a = ap.parse_args()
    try:
        globals()["cmd_" + a.cmd.replace("-", "_")](a)
    except LinkError as e:
        sys.exit(f"fel: {e}")


if __name__ == "__main__":
    main()
