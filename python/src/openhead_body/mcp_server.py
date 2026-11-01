"""MCP server: the desk body's neck as tools for LLM agents.

Runs next to OpenHead's own MCP server (the face, the cameras, the sensors): this one moves the
head. follow_face joins the two: the phone finds the person, the neck turns to them.
"""

from __future__ import annotations

import asyncio
import contextlib
import math
from typing import Literal

from mcp.server.mcpserver import MCPServer
from mcp.server.mcpserver.exceptions import ToolError
from mcp.types import ToolAnnotations

from . import __version__
from .link import LinkError
from .neck import NECK, Neck

INSTRUCTIONS = """\
Moves the OpenHead desk body: an iPhone on a four-servo neck (Open Duck Mini's neck).
- look(yaw, pitch, roll) points the head in degrees: +yaw turns to the head's left, +pitch
  looks up, +roll tilts the top of the head to its left. Yaw ±100, pitch -20..+35, roll ±25 are
  comfortable.
- gesture nod / shake / tilt for yes, no and curiosity; home to look straight ahead.
- follow_face(True) turns the neck towards the person the phone's front camera sees (needs the
  OpenHead app running and paired); the eyes on the screen follow too.
- relax switches the torque off (the head goes limp); any move switches it on again.
- status shows the pose, the joint angles, the servo load and the bus voltage.
The face itself (expressions, speech, what is on screen) is the openhead MCP server's job."""

MOVES = ToolAnnotations(readOnlyHint=False, destructiveHint=False, idempotentHint=True,
                        openWorldHint=False)
READ_ONLY = ToolAnnotations(readOnlyHint=True, destructiveHint=False, idempotentHint=True,
                            openWorldHint=False)


class Body:
    """The neck (opened on first use) and, for face following, the phone."""

    def __init__(self, port: str | None, phone: str | None):
        self.port = port
        self.phone = phone
        self.neck: Neck | None = None
        self.head = None
        self.follower = None
        self._lock = asyncio.Lock()

    async def get_neck(self) -> Neck:
        async with self._lock:
            if self.neck is None:
                try:
                    self.neck = await asyncio.to_thread(Neck.open, self.port)
                except (LinkError, OSError) as exc:
                    raise ToolError(f"cannot reach the neck ({self.port or 'auto'}): {exc}") from exc
            return self.neck

    async def run(self, fn, *args):
        neck = await self.get_neck()
        try:
            return await asyncio.to_thread(fn, neck, *args)
        except LinkError as exc:
            raise ToolError(str(exc)) from exc

    async def get_follower(self):
        if not self.phone:
            raise ToolError("face following needs the phone: start the server with --phone <address> "
                            "or set OPENHEAD_DEVICE")
        from openhead.client import Head, resolve_device

        from .follow import Follower
        if self.head is None:
            host, port = await resolve_device(self.phone)
            self.head = await Head.connect(host, port, client="openhead-body")
            if not self.head.paired:
                raise ToolError(f"not paired with the phone {self.head.key}: run `openhead pair "
                                f"--device {self.phone}` once")
        if self.follower is None:
            self.follower = Follower(await self.get_neck(), self.head)
        return self.follower

    async def close(self) -> None:
        if self.follower:
            await self.follower.stop()
        if self.head:
            await self.head.close()
        if self.neck:
            await asyncio.to_thread(self.neck.close)


def build_server(body: Body) -> MCPServer:
    mcp = MCPServer("openhead-body", title="OpenHead desk body", instructions=INSTRUCTIONS,
                    version=__version__)

    @mcp.tool(annotations=READ_ONLY)
    async def status() -> dict:
        """Where the head points, each joint's angle and load, the bus voltage and any alarm."""
        s = await body.run(lambda n: n.status())
        if body.follower:
            s["following"] = body.follower.running
            s["last_face"] = body.follower.last_face
        return s

    @mcp.tool(annotations=MOVES)
    async def look(yaw: float = 0.0, pitch: float = 0.0, roll: float = 0.0, seconds: float = 0.8) -> str:
        """Point the head, degrees in the head frame (+yaw left, +pitch up, +roll top to the left).
        Returns when the move is done."""
        await body.run(lambda n: n.look(yaw, pitch, roll, max(0.2, seconds)))
        p = await body.run(lambda n: n.pose())
        return f"looking yaw {p.yaw:+.0f}°, pitch {p.pitch:+.0f}°, roll {p.roll:+.0f}°"

    @mcp.tool(annotations=MOVES)
    async def move_joint(joint: Literal["neck_pitch", "head_pitch", "head_yaw", "head_roll"],
                         degrees: float, seconds: float = 0.8) -> str:
        """Move one neck joint (degrees, clipped to its limits). For tests and calibration; use
        look for normal moves."""
        await body.run(lambda n: n.move({joint: math.radians(degrees)}, max(0.2, seconds)))
        return f"{joint} at {(await body.run(lambda n: n.status()))['joints_deg'][joint]:+.1f}°"

    @mcp.tool(annotations=MOVES)
    async def gesture(kind: Literal["nod", "shake", "tilt"], times: int = 2) -> str:
        """Nod (yes), shake (no) or tilt the head (curious)."""
        times = max(1, min(times, 5))
        if kind == "nod":
            await body.run(lambda n: n.nod(times))
        elif kind == "shake":
            await body.run(lambda n: n.shake(times))
        else:
            await body.run(lambda n: n.tilt())
        return f"{kind} done"

    @mcp.tool(annotations=MOVES)
    async def home() -> str:
        """Look straight ahead."""
        await body.run(lambda n: n.home())
        return "home"

    @mcp.tool(annotations=MOVES)
    async def relax() -> str:
        """Torque off: the head goes limp where it is. Stops face following."""
        if body.follower:
            await body.follower.stop()
        await body.run(lambda n: n.relax())
        return "relaxed"

    @mcp.tool(annotations=MOVES)
    async def follow_face(on: bool = True) -> str:
        """Turn the neck towards the person the phone sees (the eyes follow too), or stop."""
        follower = await body.get_follower()
        if on:
            await follower.start()
            return "following the locked face"
        await follower.stop()
        return "stopped following"

    return mcp


async def serve_stdio(port: str | None, phone: str | None) -> None:
    body = Body(port, phone)
    try:
        await build_server(body).run_stdio_async()
    finally:
        with contextlib.suppress(Exception):
            await body.close()


__all__ = ["NECK", "Body", "build_server", "serve_stdio"]
