"""Servoidentifiering, nolläge och huvudkalibrering (openhead-body-calib) mot simulerad Uno och telefon."""
import json
import math
import struct
import sys

import pytest

import openhead_body.fake
from openhead_body import calib as odm_calib
from openhead_body import servo as S
from openhead_body.joints import Joints, index
from openhead_body.link import REG_OFFSET, Uno
from openhead_body.phone import FAKE_TRUE


def calib(monkeypatch, tmp_path, *args):
    monkeypatch.setattr(sys, "argv", ["openhead-body-calib", "--port", "fake", "--yes",
                                      "--config", str(tmp_path / "joints.json"), *args])
    odm_calib.main()


# ------------------------------------------------------------------------------------------- register
def test_offset_sign_magnitude():
    assert S.decode_offset(S.encode_offset(-100)) == -100
    assert S.encode_offset(-1) == 0x801 and S.decode_offset(0x0805) == -5 and S.decode_offset(37) == 37
    with pytest.raises(ValueError):
        S.encode_offset(3000)


def test_classify_gear():
    assert S.classify_gear(3000, 7.4)[0] == "1/345"
    assert S.classify_gear(-5500, 7.4)[0] == "1/191"
    assert S.classify_gear(7000, 7.2)[0] == "1/147"
    assert S.classify_gear(3000 * 8 / 7.4, 8.0)[0] == "1/345"     # normeras till 7,4 V


def test_joints_keep_extra_fields(tmp_path):
    p = tmp_path / "j.json"
    j = Joints()
    j.extra["head_yaw"]["gear"] = "1/191"
    j.save(p)
    j2 = Joints.load(p)
    assert j2.extra["head_yaw"] == {"gear": "1/191"}
    j2.save(p)
    assert json.loads(p.read_text(encoding="utf-8"))["head_yaw"]["gear"] == "1/191"


def test_audit_finds_leftover_offset_and_fix_clears_it(monkeypatch, tmp_path, capsys):
    fake = openhead_body.fake.FakeUno(ids=odm_calib.FAKE_IDS)
    struct.pack_into("<H", fake.servos[32].regs, REG_OFFSET, S.encode_offset(-120))   # LeRobot-rest
    struct.pack_into("<H", fake.servos[32].regs, 9, 800)                               # min-gräns
    monkeypatch.setattr(openhead_body.fake, "FakeUno", lambda ids: fake)
    calib(monkeypatch, tmp_path, "audit")
    out = capsys.readouterr().out
    assert "egen offset -120" in out and "vinkelgränser 800..4095" in out
    calib(monkeypatch, tmp_path, "fix", "head_yaw")
    calib(monkeypatch, tmp_path, "audit")
    assert "alla ok" in capsys.readouterr().out


# ------------------------------------------------------------------------------------------- utväxling
@pytest.mark.parametrize("speed,gear", [(3000.0, "1/345"), (7000.0, "1/147")])
def test_gear_speed_test(monkeypatch, tmp_path, capsys, speed, gear):
    monkeypatch.setattr(openhead_body.fake, "MAX_SPEED", speed)
    calib(monkeypatch, tmp_path, "gear", "head_roll", "--decal", "1/345")
    out = capsys.readouterr().out
    assert f"närmast {gear}" in out
    assert ("VARNING" in out) == (gear != "1/345")
    j = Joints.load(tmp_path / "joints.json")
    assert j.extra["head_roll"]["gear"] == "1/345" and j.extra["head_roll"]["gear_test"]["gissning"] == gear


def test_gear_restores_max_step(monkeypatch, tmp_path):
    fake = openhead_body.fake.FakeUno(ids=odm_calib.FAKE_IDS)
    monkeypatch.setattr(openhead_body.fake, "FakeUno", lambda ids: fake)
    calib(monkeypatch, tmp_path, "gear", "head_roll")
    assert fake.param[1] == 40 and not fake.torque


# ------------------------------------------------------------------------------------------- nolläge
def test_zero_reaches_sim_zero_with_torque_limit(monkeypatch, tmp_path):
    fake = openhead_body.fake.FakeUno(ids=odm_calib.FAKE_IDS)
    monkeypatch.setattr(openhead_body.fake, "FakeUno", lambda ids: fake)
    j = Joints()
    j.sign["head_yaw"], j.offset["head_yaw"] = -1, math.radians(5)
    j.save(tmp_path / "joints.json")
    limits = []
    orig = odm_calib.set_torque_limit
    monkeypatch.setattr(odm_calib, "set_torque_limit", lambda u, jj, nn, v: (limits.append(v), orig(u, jj, nn, v)))
    calib(monkeypatch, tmp_path, "zero", "head_yaw", "--hold", "0.5")
    raw = fake.servos[32].pos
    assert abs(j.to_rad("head_yaw", raw)) < math.radians(0.5)       # 0 rad efter tecken och offset
    assert limits == [300, 1000]                                    # momentgräns under rörelsen, sedan återställd
    assert struct.unpack_from("<H", fake.servos[32].regs, S.REG_TORQUE_LIMIT)[0] == 1000
    assert not fake.torque


def test_zero_raw_goes_to_2048(monkeypatch, tmp_path):
    fake = openhead_body.fake.FakeUno(ids=odm_calib.FAKE_IDS)
    monkeypatch.setattr(openhead_body.fake, "FakeUno", lambda ids: fake)
    j = Joints()
    j.offset["head_roll"] = math.radians(4)
    j.save(tmp_path / "joints.json")
    calib(monkeypatch, tmp_path, "zero", "--raw", "head_roll", "--hold", "0.5")
    assert abs(fake.servos[33].pos - 2048) < 3


def test_zero_aborts_on_load(monkeypatch, tmp_path, capsys):
    """head_pitch slår i något: lasten i tillståndet går över gränsen -> avbryt och släpp momentet."""
    orig = Uno.command

    def heavy(self, *a, **kw):
        st = orig(self, *a, **kw)
        if kw.get("torque"):
            st["load"][index("head_pitch")] = 900
        return st

    monkeypatch.setattr(Uno, "command", heavy)
    calib(monkeypatch, tmp_path, "zero", "head_pitch", "--hold", "0.2")
    out = capsys.readouterr().out
    assert "AVBRUTET" in out and "last 900" in out


# ------------------------------------------------------------------------------------------- huvudet
def test_head_capture_and_fit_end_to_end(monkeypatch, tmp_path, capsys):
    """Simulerad Uno + simulerad telefon med känd kalibrering (FAKE_TRUE): mät, anpassa, spara."""
    base = tmp_path / "stativ.json"
    calib(monkeypatch, tmp_path, "head-base", "--phone", "fake", "--out", str(base))
    cap = tmp_path / "cap.json"
    calib(monkeypatch, tmp_path, "head-capture", "--phone", "fake", "--only", "1", "6", "10", "13", "16",
          "--settle", "0.2", "--measure", "0.3", "--speed", "200", "--out", str(cap))
    d = json.loads(cap.read_text(encoding="utf-8"))
    assert [s["pose"] for s in d["samples"]] == [1, 6, 10, 13, 16]
    assert all(s["summary"]["n"] > 10 for s in d["samples"])
    calib(monkeypatch, tmp_path, "head-fit", str(cap), "--base", str(base), "--save")
    j = Joints.load(tmp_path / "joints.json")
    for n in ("head_pitch", "head_yaw", "head_roll"):
        assert j.sign[n] == FAKE_TRUE["signs"][n], n
        assert abs(j.offset[n] - FAKE_TRUE["offset"][n]) < math.radians(0.3), n
        assert j.extra[n]["kalibrering"]["metod"] == "telefonens gravitation"
    assert "neck_pitch" in j.extra and "kalibrering" not in j.extra["neck_pitch"]
