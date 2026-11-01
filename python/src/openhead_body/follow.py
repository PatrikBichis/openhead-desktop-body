"""Turn the neck towards the person the phone sees.

The phone (OpenHead) reports faces about 15 times a second with a bearing and elevation in
degrees from the front camera, which moves with the head. Each report nudges the head's
target by a share of that angle, so the face drifts to the middle of the camera's view; the
eyes on the screen follow the face at the same time (look_at_face), so they lead and the neck
catches up. With no face for a while the head drifts back to the middle.
"""

from __future__ import annotations

import asyncio
import contextlib
import time

from openhead.client import Head

from .neck import Neck, Pose

GAIN = 0.45            # share of the face's angle corrected per report
DEADBAND = 2.5         # degrees: closer than this to the middle is good enough
LOST_AFTER = 2.0       # seconds without a face before drifting home
MAX_YAW, MAX_PITCH = 100.0, 35.0


class Follower:
    def __init__(self, neck: Neck, head: Head):
        self.neck = neck
        self.head = head
        self.task: asyncio.Task | None = None
        self.seen = 0
        self.last_face: dict | None = None

    @property
    def running(self) -> bool:
        return self.task is not None and not self.task.done()

    async def start(self) -> None:
        if self.running:
            return
        if await self.head.mode.get() == "idle":
            await self.head.mode.set("capture")       # front camera on; the face stays on screen
        await self.head.face.look_at_face()
        self.task = asyncio.create_task(self._run())

    async def stop(self) -> None:
        if self.task:
            self.task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self.task
        self.task = None
        with contextlib.suppress(Exception):
            await self.head.face.look_at(0, 0)

    async def _run(self) -> None:
        target = self.neck.pose()
        last_seen = time.monotonic()
        async with self.head.subscribe("faces", 15) as sub:
            async for msg in sub:
                faces = msg.faces or []
                face = next((f for f in faces if f.locked), faces[0] if faces else None)
                now = time.monotonic()
                if face is None:
                    if now - last_seen > LOST_AFTER:   # nobody there: back to the middle, slowly
                        target = Pose(target.yaw * 0.97, target.pitch * 0.97, 0.0)
                        await asyncio.to_thread(self.neck.look, target.yaw, target.pitch, 0.0, 0.25, False)
                    continue
                last_seen = now
                self.seen += 1
                self.last_face = {"bearing": face.bearing, "elevation": face.elevation,
                                  "distance_m": face.distance_m}
                # +bearing is the head's left = +yaw; +elevation is up = +pitch
                dy = face.bearing if abs(face.bearing) > DEADBAND else 0.0
                dp = face.elevation if abs(face.elevation) > DEADBAND else 0.0
                if dy or dp:
                    target = Pose(max(-MAX_YAW, min(MAX_YAW, target.yaw + GAIN * dy)),
                                  max(-MAX_PITCH, min(MAX_PITCH, target.pitch + GAIN * dp)), 0.0)
                    await asyncio.to_thread(self.neck.look, target.yaw, target.pitch, 0.0, 0.2, False)
