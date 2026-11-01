"""The neck controller, face following and the MCP server against the simulated Uno."""

import asyncio
import math
import time
from types import SimpleNamespace

from mcp import Client

from openhead_body.follow import Follower
from openhead_body.mcp_server import Body, build_server
from openhead_body.neck import NECK, Neck, Pose, joints_to_pose, pose_to_joints


def test_pose_splits_pitch_between_neck_and_head():
    q = pose_to_joints(Pose(yaw=30, pitch=20, roll=-10))
    assert math.degrees(q["neck_pitch"] + q["head_pitch"]) == 20
    assert 0 < q["neck_pitch"] < q["head_pitch"]
    p = joints_to_pose(q)
    assert (round(p.yaw), round(p.pitch), round(p.roll)) == (30, 20, -10)


def test_look_moves_the_servos_and_relax_lets_go():
    with Neck.open("fake") as neck:
        neck.look(25, 10, 5, seconds=0.4)
        time.sleep(0.4)                     # the servos trail the targets a little
        p = neck.pose()
        assert abs(p.yaw - 25) < 2 and abs(p.pitch - 10) < 2 and abs(p.roll - 5) < 2
        assert neck.torque and "torque" in neck.status()["flags"]
        neck.relax()
        assert not neck.torque
        time.sleep(0.1)
        assert "torque" not in neck.status()["flags"]


def test_targets_are_clipped_to_the_joint_limits():
    with Neck.open("fake") as neck:
        neck.look(0, 200, 0, seconds=0.3)   # far past the pitch limits
        time.sleep(0.3)
        q = neck.status()["joints_deg"]
        assert q["head_pitch"] <= 45.1 and q["neck_pitch"] <= 65.1


def test_gestures_come_back_to_where_they_started():
    with Neck.open("fake") as neck:
        neck.look(10, 0, 0, seconds=0.3)
        neck.nod(times=1, period=0.4)
        neck.shake(times=1, period=0.4)
        time.sleep(0.3)
        p = neck.pose()
        assert abs(p.yaw - 10) < 2 and abs(p.pitch) < 2


def test_the_loop_keeps_the_watchdog_fed():
    with Neck.open("fake") as neck:
        neck.look(15, 0, 0, seconds=0.2)
        time.sleep(0.5)                     # well past the 150 ms watchdog with no new move
        assert neck.error is None and "watchdog" not in neck.status()["flags"]


class FakePhone:
    """Just enough of openhead.client.Head: a mode, the face commands and a faces stream."""

    def __init__(self, bearings):
        self.bearings = bearings
        self.calls = []
        self.mode = SimpleNamespace(get=self._mode_get, set=self._mode_set)
        self.face = SimpleNamespace(look_at_face=self._call("look_at_face"), look_at=self._call("look_at"))

    async def _mode_get(self):
        return "idle"

    async def _mode_set(self, mode):
        self.calls.append(("mode", mode))

    def _call(self, name):
        async def f(*a, **k):
            self.calls.append((name, a))
        return f

    def subscribe(self, topic, rate):
        phone = self

        class Sub:
            async def __aenter__(self):
                return self

            async def __aexit__(self, *a):
                return False

            def __aiter__(self):
                return self

            async def __anext__(self):
                await asyncio.sleep(1 / 15)
                b = phone.bearings.pop(0) if phone.bearings else 0.0
                face = SimpleNamespace(bearing=b, elevation=0.0, distance_m=None, locked=True)
                return SimpleNamespace(faces=[face])

        return Sub()


async def test_follower_turns_towards_the_face():
    with Neck.open("fake") as neck:
        phone = FakePhone([20.0] * 6)       # someone 20° to the head's left
        follower = Follower(neck, phone)
        await follower.start()
        await asyncio.sleep(0.8)
        await follower.stop()
        assert ("mode", "capture") in phone.calls and phone.calls[1][0] == "look_at_face"
        assert neck.pose().yaw > 10         # turned left, towards them
        assert follower.seen >= 5


async def test_mcp_tools():
    body = Body("fake", None)
    try:
        async with Client(build_server(body)) as client:
            names = {t.name for t in (await client.list_tools()).tools}
            assert {"look", "gesture", "home", "relax", "status", "follow_face", "move_joint"} <= names
            r = await client.call_tool("look", {"yaw": 20, "pitch": 5, "seconds": 0.3})
            assert not r.is_error and "yaw +" in r.content[0].text
            r = await client.call_tool("gesture", {"kind": "nod", "times": 1})
            assert not r.is_error
            r = await client.call_tool("status", {})
            assert not r.is_error and "joints_deg" in r.content[0].text
            r = await client.call_tool("follow_face", {"on": True})
            assert r.is_error and "--phone" in r.content[0].text
            r = await client.call_tool("relax", {})
            assert not r.is_error
    finally:
        await body.close()


assert NECK == ("neck_pitch", "head_pitch", "head_yaw", "head_roll")
