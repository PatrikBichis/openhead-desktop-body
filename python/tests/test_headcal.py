"""Huvudkalibreringens matematik (openhead_body/headcal.py) och telefonmottagaren (openhead_body/phone.py)."""
import math
import socket

import pytest

from openhead_body import headcal as H
from openhead_body.phone import PhoneUDP, parse

TRUE = {"signs": {"neck_pitch": 1, "head_pitch": -1, "head_yaw": 1, "head_roll": -1},
        "offset": {"neck_pitch": 0.03, "head_pitch": -0.08, "head_yaw": 0.11, "head_roll": 0.05},
        "base": (0.01, -0.004), "backlash": {"head_pitch": 0.01, "head_yaw": 0.015}}


def test_fk_axes_from_cad():
    m = H.HeadModel()
    assert m.axes["neck_pitch"] == m.axes["head_pitch"]            # parallella: bara summan syns
    # nolläget: telefonen liggande med skärmen framåt och överkanten åt vänster -> telefonens +x pekar nedåt
    g = H.HeadModel(mount=H.load_mount()).gravity({})
    assert max(abs(a - b) for a, b in zip(g, (1.0, 0.0, 0.0))) < 1e-9


def test_base_tilt_roundtrip():
    for b in ((0.0, 0.0), (0.02, -0.01), (-0.05, 0.03)):
        g_flat = H.mv(H.T(H.PHONE_ON_BASE), H.base_gravity(b))
        bx, by = H.base_tilt(g_flat)
        assert abs(bx - b[0]) < 1e-9 and abs(by - b[1]) < 1e-9


def test_fit_recovers_offsets_signs_and_backlash():
    m = H.HeadModel(mount=H.load_mount())
    poses = H.plan_poses()
    data = H.synthetic(m, TRUE, poses) + H.synthetic(m, TRUE, poses, approach={n: -1 for n in H.NECK}, seed=2)
    f = H.fit(data, m, signs={n: 1 for n in H.NECK}, offsets={"neck_pitch": 0.03},
              fit_backlash=("head_pitch", "head_yaw", "head_roll"), fit_scale=("head_yaw",), base=(0, -0.004))
    assert f.signs == TRUE["signs"]
    for n in ("head_pitch", "head_yaw", "head_roll"):
        assert abs(f.offset[n] - TRUE["offset"][n]) < math.radians(0.2), n
    assert abs(f.backlash["head_yaw"] - 0.015) < math.radians(0.2)
    assert abs(f.scale["head_yaw"] - 1) < 0.01
    assert abs(f.base[0] - 0.01) < math.radians(0.2)
    assert f.rms < math.radians(0.4) and f.second_rms > 10 * f.rms


def test_fit_flips_template_offset_with_sign():
    """neck_pitch-offset från mallen är mätt med gammalt tecken; vid teckenbyte byter den också tecken."""
    m = H.HeadModel(mount=H.load_mount())
    data = H.synthetic(m, TRUE, H.plan_poses(), noise_deg=0.05)
    f = H.fit(data, m, signs={"neck_pitch": -1}, offsets={"neck_pitch": -0.03})
    assert f.signs["neck_pitch"] == 1 and abs(f.offset["neck_pitch"] - 0.03) < 1e-12


def test_fit_refuses_parallel_pitch_offsets():
    with pytest.raises(ValueError):
        H.fit([], H.HeadModel(), {}, {}, fit_offsets=("neck_pitch", "head_pitch"))


def test_phone_parse_and_udp():
    assert parse('{"t": 1.5, "g": [0, -1, 0], "w": [0.1, 0, 0]}')["g"] == (0.0, -1.0, 0.0)
    assert parse("2.0,0.1,0.2,-0.9")["w"] == (0.0, 0.0, 0.0)
    ph = PhoneUDP(port=0, host="127.0.0.1")
    port = ph.sock.getsockname()[1]
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx.sendto(b'{"t": 1, "g": [0.5, 0.5, -0.7], "w": [0, 0, 0.01]}\n3,0,0,-1,0,0,0\ngarbage', ("127.0.0.1", port))
    got = []
    for _ in range(200):
        got += ph.poll()
        if len(got) >= 2:
            break
    tx.close()
    ph.close()
    assert [s["g"][2] for s in got] == [-0.7, -1.0] and ph.bad == 1
