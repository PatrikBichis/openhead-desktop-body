"""Datorverktygets flöden mot den simulerade Uno:n (samma regler som firmware)."""
import math
import time

import pytest

from openhead_body import protocol as P
from openhead_body.joints import Joints, index
from openhead_body.link import REG_ID, LinkError, Uno


@pytest.fixture
def uno():
    u = Uno.open("fake")
    yield u
    u.close()


def test_info_and_ids(uno):
    i = uno.info()
    assert i["fw"] == P.FW_VERSION and i["watchdog_ms"] == 150
    ids, mask = uno.get_ids()
    assert ids == P.DEFAULT_IDS
    assert mask == (1 << index("left_hip_yaw")) | (1 << index("left_hip_roll"))


def test_scan_and_set_id(uno):
    assert uno.scan() == [20, 21]
    uno.write_eeprom(21, REG_ID, bytes([22]))
    assert uno.scan() == [20, 22]
    _, mask = uno.set_ids(P.DEFAULT_IDS)                 # Uno räknar om aktiva leder
    assert not mask & (1 << index("left_hip_roll"))     # ID 21 finns inte längre


def test_torque_starts_from_present_and_is_rate_limited(uno):
    j = index("left_hip_yaw")
    start = uno.state["pos"][j]
    targets = list(uno.state["pos"])
    targets[j] = start + 500
    st = uno.command(targets, torque=True)
    assert st["status"] & P.ST_TORQUE
    for _ in range(30):
        st = uno.command(targets, torque=True)
        time.sleep(0.02)
    # max_step 40 per kommando: efter 31 kommandon är målet nått, servot på väg dit
    assert abs(st["pos"][j] - (start + 500)) < 60


def test_write_refused_with_torque(uno):
    uno.command(uno.state["pos"], torque=True)
    with pytest.raises(LinkError):
        uno.write(20, 41, b"\x00")


def test_watchdog_latches_until_reset(uno):
    uno.command(uno.state["pos"], torque=True)
    time.sleep(0.3)
    assert uno.wait_for(lambda: uno.state["status"] & P.ST_WATCHDOG, 0.5)
    assert not uno.state["status"] & P.ST_TORQUE
    st = uno.command(uno.state["pos"], torque=True)
    assert not st["status"] & P.ST_TORQUE
    st = uno.command(uno.state["pos"], torque=True, reset=True)
    assert st["status"] & P.ST_TORQUE and not st["status"] & P.ST_WATCHDOG


def test_estop(uno):
    uno.command(uno.state["pos"], torque=True)
    st = uno.command(uno.state["pos"], torque=True, estop=True)
    assert st["status"] & P.ST_ESTOP and not st["status"] & P.ST_TORQUE


def test_joint_conversion_roundtrip():
    j = Joints()
    j.sign["left_knee"] = -1
    j.offset["left_knee"] = 0.1
    for q in (-1.0, 0.0, 0.5, 1.2):
        assert abs(j.to_rad("left_knee", j.to_raw("left_knee", q)) - q) < 2 * math.pi / 4096
    assert j.to_raw("left_hip_yaw", 3.0) == j.to_raw("left_hip_yaw", 0.5236)   # klippt mot MJCF-gränsen


# ---------------------------------------------------------------------------------------------- fallreflex
def _fake(uno):
    return uno.s        # FakeUno bakom länken


def _arm(uno, deg=35):
    j = index("left_hip_yaw")
    pose = [P.REFLEX_KEEP] * P.N_JOINTS
    pose[j] = 2600
    for d in range(P.N_FALL_DIRS):
        assert uno.set_reflex(d, pose) == pose
    uno.set_param(P.PARAM_FALL_DEG, deg)
    uno.command(uno.state["pos"], torque=True)
    uno.command(uno.state["pos"], torque=True)      # upprätt -> reflexen blir beredd
    return j


def test_reflex_off_by_default(uno):
    assert uno.info()["fall_deg"] == 0
    uno.command(uno.state["pos"], torque=True)
    _fake(uno).imu_up = [-0.7, 0.0, 0.7]             # 45° framåt
    st = uno.command(uno.state["pos"], torque=True)
    assert not st["status"] & P.ST_FALL


def test_reflex_triggers_by_prediction_and_holds_pose(uno):
    j = _arm(uno)
    f = _fake(uno)
    f.imu_up = [-0.26, 0.0, 0.966]                   # 15° framåt: under gränsen …
    st = uno.command(uno.state["pos"], torque=True)
    assert not st["status"] & P.ST_FALL
    f.imu_gyro = [0.0, 2.5, 0.0]                     # … men 2,5 rad/s × 150 ms ≈ 21° till: prognos 36°
    st = uno.command(uno.state["pos"], torque=True)
    assert st["status"] & P.ST_FALL and f.fall_dir == P.FALL_FWD
    assert f.applied[j] == 2600                      # reflexposen direkt, utan max_step
    st = uno.command([2048] * P.N_JOINTS, torque=True)
    assert f.applied[j] == 2600                      # kommandomål ignoreras under reflexen


def test_reflex_direction(uno):
    _arm(uno)
    f = _fake(uno)
    f.imu_up = [0.0, -0.64, 0.77]                    # 40° åt vänster
    uno.command(uno.state["pos"], torque=True)
    assert f.fall_dir == P.FALL_LEFT


def test_reflex_goes_limp_and_needs_upright_after_reset(uno):
    _arm(uno)
    uno.set_param(P.PARAM_FALL_LIMP_MS, 100)
    f = _fake(uno)
    f.imu_up = [0.6, 0.0, 0.8]                       # 37° bakåt
    uno.command(uno.state["pos"], torque=True)
    assert f.fall and f.fall_dir == P.FALL_BACK
    time.sleep(0.15)
    st = uno.command(uno.state["pos"], torque=True)
    assert st["status"] & P.ST_FALL and not st["status"] & P.ST_TORQUE   # slapp, och moment går inte att slå på
    st = uno.command(uno.state["pos"], torque=True, reset=True)          # reset medan roboten ligger:
    assert st["status"] & P.ST_TORQUE and not st["status"] & P.ST_FALL   # moment på, reflexen inte beredd
    uno.command(uno.state["pos"], torque=True)
    assert not f.fall
    f.imu_up = [0.0, 0.0, 1.0]                       # rest upp -> beredd igen
    uno.command(uno.state["pos"], torque=True)
    f.imu_up = [0.6, 0.0, 0.8]
    st = uno.command(uno.state["pos"], torque=True)
    assert st["status"] & P.ST_FALL


def test_imu_in_state(uno):
    f = _fake(uno)
    f.imu_gyro = [0.1, -0.2, 0.3]
    st = uno.command(uno.state["pos"], torque=False)
    assert st["status"] & P.ST_IMU_OK
    assert st["imu"][:3] == [100, -200, 300] and abs(st["imu"][5] - 981) <= 1


def test_fall_reflex_poses_upload(uno):
    from openhead_body import fall_reflex
    j = Joints()
    raw = fall_reflex.raw_poses(j)
    assert raw[P.FALL_FWD][index("neck_pitch")] == j.to_raw("neck_pitch", 1.13)
    assert raw[P.FALL_LEFT][index("head_yaw")] < 2048 < raw[P.FALL_RIGHT][index("head_yaw")]
    fall_reflex.upload(uno, j)
    i = uno.info()
    assert (i["fall_deg"], i["fall_horizon_ms"], i["fall_limp_ms"]) == (35, 150, 1500)
    assert uno.get_reflex(P.FALL_BACK) == raw[P.FALL_BACK]
