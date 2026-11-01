"""The neck as one object: where the head looks, how it gets there, and when it lets go.

    from openhead_body.neck import Neck
    with Neck.open() as neck:          # "fake" for a simulated Uno with four servos
        neck.look(yaw=20, pitch=10)    # degrees; returns when the move is done
        neck.nod()
        neck.relax()

The Uno releases the servos if no command arrives for 150 ms (its watchdog), so while the
torque is on a thread sends the current targets 50 times a second. Moves follow a minimum-jerk
profile. Angles are in the head frame: +yaw turns left (the head's left), +pitch looks up,
+roll tilts the top of the head to its left. Pitch is shared between the neck and the head,
whose axes are parallel.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass

from . import protocol as P
from .joints import CONFIG_PATH, Joints, index
from .link import LinkError, Uno

NECK = ("neck_pitch", "head_pitch", "head_yaw", "head_roll")
RATE = 50
NECK_SHARE = 0.35          # share of the pitch taken by neck_pitch; head_pitch takes the rest
FAKE_IDS = (30, 31, 32, 33)    # servos on the simulated bus: the neck only


def minjerk(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return s ** 3 * (10 - 15 * s + 6 * s * s)


@dataclass
class Pose:
    """Where the head points, in degrees (head frame)."""
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0


def pose_to_joints(p: Pose) -> dict[str, float]:
    pitch = math.radians(p.pitch)
    return {"neck_pitch": NECK_SHARE * pitch, "head_pitch": (1 - NECK_SHARE) * pitch,
            "head_yaw": math.radians(p.yaw), "head_roll": math.radians(p.roll)}


def joints_to_pose(q: dict[str, float]) -> Pose:
    return Pose(yaw=math.degrees(q["head_yaw"]), pitch=math.degrees(q["neck_pitch"] + q["head_pitch"]),
                roll=math.degrees(q["head_roll"]))


class Neck:
    def __init__(self, uno: Uno, joints: Joints):
        self.uno = uno
        self.joints = joints
        _, self.mask = uno.get_ids()
        missing = [n for n in NECK if not self.mask & (1 << index(n))]
        if missing:
            uno.close()
            raise LinkError(f"no servo answers for {', '.join(missing)}: check power and IDs (scan)")
        self._lock = threading.Lock()
        self._target = self.measured()                 # rad per joint, what the loop sends
        self._motion: tuple[float, dict, dict, float] | None = None  # start time, from, to, seconds
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self.error: str | None = None

    @classmethod
    def open(cls, port: str | None = None, config=CONFIG_PATH) -> Neck:
        if port == "fake":
            from . import fake
            uno = Uno(fake.FakeUno(ids=FAKE_IDS))
            if not uno.wait_for(lambda: uno.state is not None, 1.0):
                raise LinkError("the simulated Uno did not start")
        else:
            uno = Uno.open(port)
        joints = Joints.load(config)
        uno.torque_off()
        ids, _ = uno.get_ids()
        if ids != joints.id_list():          # the Uno's joint table follows joints.json
            uno.set_ids(joints.id_list())
        return cls(uno, joints)

    def __enter__(self) -> Neck:
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ------------------------------------------------------------------ state
    def measured(self) -> dict[str, float]:
        """Joint angles (rad) the servos report."""
        st = self.uno.state
        return {n: self.joints.to_rad(n, st["pos"][index(n)]) for n in NECK}

    def pose(self) -> Pose:
        return joints_to_pose(self.measured())

    @property
    def torque(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def status(self) -> dict:
        st = self.uno.state
        q = self.measured()
        return {
            "torque": self.torque,
            "pose": vars(self.pose()),
            "joints_deg": {n: round(math.degrees(v), 1) for n, v in q.items()},
            "load": {n: st["load"][index(n)] for n in NECK},
            "bus_voltage_v": round(st["bus_mv"] / 1000, 2),
            "flags": [name for bit, name in ((P.ST_TORQUE, "torque"), (P.ST_WATCHDOG, "watchdog"),
                                             (P.ST_ESTOP, "estop"), (P.ST_LOW_VOLTAGE, "low_voltage"),
                                             (P.ST_SERVO_ERROR, "servo_error"))
                      if st["status"] & bit],
            "error": self.error,
        }

    # ------------------------------------------------------------------ motion
    def move(self, goal: dict[str, float], seconds: float = 0.8, wait: bool = True) -> None:
        """Move named joints (rad) to goal; joints not named keep their target."""
        goal = {n: self.joints.clamp(n, v) for n, v in goal.items()}
        self._ensure_running()
        with self._lock:
            start = dict(self._target)
            end = {**start, **goal}
            self._motion = (time.perf_counter(), start, end, max(seconds, 1 / RATE))
        if wait:
            self.wait()

    def look(self, yaw: float = 0.0, pitch: float = 0.0, roll: float = 0.0, seconds: float = 0.8,
             wait: bool = True) -> None:
        """Point the head (degrees, head frame)."""
        self.move(pose_to_joints(Pose(yaw, pitch, roll)), seconds, wait)

    def wait(self) -> None:
        while True:
            with self._lock:
                m = self._motion
            if m is None or not self.torque:
                break
            time.sleep(1 / RATE)
        if self.error:
            raise LinkError(self.error)

    def home(self, seconds: float = 1.2) -> None:
        self.look(0, 0, 0, seconds)

    def nod(self, times: int = 2, deg: float = 12.0, period: float = 0.6) -> None:
        p = self.pose()
        for _ in range(times):
            self.look(p.yaw, p.pitch - deg, p.roll, period / 2)
            self.look(p.yaw, p.pitch + deg / 3, p.roll, period / 2)
        self.look(p.yaw, p.pitch, p.roll, period / 2)

    def shake(self, times: int = 2, deg: float = 20.0, period: float = 0.7) -> None:
        p = self.pose()
        for _ in range(times):
            self.look(p.yaw + deg, p.pitch, p.roll, period / 2)
            self.look(p.yaw - deg, p.pitch, p.roll, period / 2)
        self.look(p.yaw, p.pitch, p.roll, period / 2)

    def tilt(self, deg: float = 15.0, hold: float = 0.8) -> None:
        p = self.pose()
        self.look(p.yaw, p.pitch, deg, 0.5)
        time.sleep(hold)
        self.look(p.yaw, p.pitch, p.roll, 0.5)

    def relax(self) -> None:
        """Torque off: the head goes limp where it is."""
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        self._thread = None
        try:
            self.uno.torque_off()
        except LinkError:
            pass

    def close(self) -> None:
        self.relax()
        self.uno.close()

    # ------------------------------------------------------------------ the 50 Hz loop
    def _ensure_running(self) -> None:
        if self.torque:
            return
        self.error = None
        self.uno.command(self.uno.state["pos"], torque=False, reset=True)  # clear a tripped watchdog
        with self._lock:
            self._target = self.measured()
            self._motion = None
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="neck-50hz", daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        k = 0
        t0 = time.perf_counter()
        try:
            while not self._stop.is_set():
                now = time.perf_counter()
                with self._lock:
                    if self._motion:
                        start_t, a, b, dur = self._motion
                        s = minjerk((now - start_t) / dur)
                        self._target = {n: a[n] + (b[n] - a[n]) * s for n in NECK}
                        if s >= 1.0:
                            self._motion = None
                    target = dict(self._target)
                pos = self.uno.state["pos"]
                raw = [self.joints.to_raw(n, target[n]) if n in target else pos[i]
                       for i, n in enumerate(P.JOINT_NAMES)]
                st = self.uno.command(raw, torque=True)
                if st["status"] & (P.ST_WATCHDOG | P.ST_ESTOP):
                    self.error = "the Uno switched the torque off (watchdog or emergency stop)"
                    break
                k += 1
                nxt = t0 + k / RATE
                delay = nxt - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                elif delay < -0.5:          # fell far behind (a stall): restart the schedule
                    t0, k = time.perf_counter(), 0
        except LinkError as exc:
            self.error = str(exc)
        finally:
            with self._lock:
                self._motion = None
