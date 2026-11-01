from pathlib import Path

import pytest

from openhead_body import protocol as P

ROOT = Path(__file__).resolve().parents[2]


def test_sizes():
    assert P.SIZE == {P.TYPE_COMMAND: 39, P.TYPE_STATE: 118, P.TYPE_DIAG: 45}


def test_crc_known_value():
    assert P.crc16(b"123456789") == 0x29B1   # CRC-16/CCITT-FALSE kontrollvärde


def test_roundtrip_and_crc():
    raw = P.pack(P.TYPE_COMMAND, 7, flags=P.FLAG_TORQUE, targets=list(range(100, 114)), antennas=[90, 255])
    d = P.unpack(raw)
    assert d["seq"] == 7 and d["flags"] == 1 and d["targets"][13] == 113 and d["antennas"] == [90, 255]
    bad = bytearray(raw)
    bad[10] ^= 1
    with pytest.raises(ValueError):
        P.unpack(bytes(bad))


def test_stream_parser_resyncs():
    a = P.pack(P.TYPE_STATE, 1, pos=[2048] * 14)
    b = P.pack(P.TYPE_DIAG, 2, op=P.OP_PING, data=b"\x09\x03")
    corrupt = bytearray(P.pack(P.TYPE_COMMAND, 3))
    corrupt[-1] ^= 0xFF
    stream = b"\x00\x44junk" + a + bytes(corrupt) + b"\x44" + b
    sp = P.StreamParser()
    got = []
    for i in range(0, len(stream), 7):   # matas i bitar som från en seriell port
        got += sp.feed(stream[i:i + 7])
    assert [g["type"] for g in got] == [P.TYPE_STATE, P.TYPE_DIAG]
    assert got[1]["data"][:2] == b"\x09\x03"


@pytest.mark.parametrize("sketch", P.SKETCHES)
def test_firmware_header_in_sync(sketch):
    h = (ROOT / "firmware" / sketch / "protocol.h").read_text(encoding="utf-8")
    assert h == P.c_header(), "kör: cd python && uv run python -m openhead_body.protocol --header"


@pytest.mark.parametrize("fname", ["feetech.h", "feetech.cpp"])
def test_feetech_copies_identical(fname):
    """Servolagret finns i båda sketcherna (Arduino kan inte dela filer mellan sketchmappar)."""
    a = (ROOT / "firmware" / "body_uno" / fname).read_bytes().replace(b"\r\n", b"\n")
    b = (ROOT / "firmware" / "servo_setup" / fname).read_bytes().replace(b"\r\n", b"\n")
    assert a == b, f"ändra i firmware/body_uno/{fname} och kopiera till servo_setup"


def test_labels():
    assert len(P.JOINT_LABELS) == len(P.JOINT_NAMES) == len(P.DEFAULT_IDS) == P.N_JOINTS
