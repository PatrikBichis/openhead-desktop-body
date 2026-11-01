"""openhead-body: move the desk body's head from the command line, or serve it over MCP.

  openhead-body status                      pose, joint angles, load, bus voltage
  openhead-body look 30 10                  yaw, pitch (and roll) in degrees
  openhead-body nod | shake | tilt          gestures
  openhead-body home | relax                straight ahead | torque off
  openhead-body follow --phone 192.168.1.20 turn towards the person the phone sees (Ctrl+C stops)
  openhead-body mcp --phone 192.168.1.20    MCP server on stdio for AI agents

--port picks the Uno's serial port (default: the first Arduino found; "fake" simulates one).
Servo setup and calibration: openhead-body-bench and openhead-body-calib (docs/calibration.md).
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys

from .link import LinkError
from .neck import Neck


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="openhead-body", description=__doc__.splitlines()[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter,
                                epilog="\n".join(__doc__.splitlines()[2:]))
    p.add_argument("--port", default=os.environ.get("OPENHEAD_BODY_PORT"),
                   help="Uno serial port, e.g. COM5 or /dev/ttyACM0 ($OPENHEAD_BODY_PORT; fake = simulated)")
    p.add_argument("--phone", default=os.environ.get("OPENHEAD_DEVICE"),
                   help="the OpenHead phone, address[:port] ($OPENHEAD_DEVICE); for follow and mcp")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status")
    look = sub.add_parser("look")
    look.add_argument("yaw", type=float)
    look.add_argument("pitch", type=float, nargs="?", default=0.0)
    look.add_argument("roll", type=float, nargs="?", default=0.0)
    look.add_argument("--seconds", type=float, default=0.8)
    look.add_argument("--keep", action="store_true", help="hold the pose (torque stays on until Ctrl+C)")
    for name in ("nod", "shake", "tilt", "home"):
        sub.add_parser(name)
    sub.add_parser("relax")
    sub.add_parser("follow")
    sub.add_parser("mcp")
    a = p.parse_args(argv)

    if a.cmd == "mcp":
        from .mcp_server import serve_stdio
        asyncio.run(serve_stdio(a.port, a.phone))
        return 0
    if a.cmd == "follow":
        if not a.phone:
            p.error("follow needs --phone (or OPENHEAD_DEVICE)")
        return asyncio.run(_follow(a.port, a.phone))

    try:
        with Neck.open(a.port) as neck:
            match a.cmd:
                case "status":
                    print(json.dumps(neck.status(), indent=2))
                case "look":
                    neck.look(a.yaw, a.pitch, a.roll, a.seconds)
                    print(json.dumps(vars(neck.pose())))
                    if a.keep:
                        _hold()
                case "nod":
                    neck.nod()
                case "shake":
                    neck.shake()
                case "tilt":
                    neck.tilt()
                case "home":
                    neck.home()
                case "relax":
                    pass                         # closing the neck switches the torque off
    except LinkError as exc:
        print(f"openhead-body: {exc}", file=sys.stderr)
        return 1
    return 0


def _hold() -> None:
    import time
    print("holding; Ctrl+C releases")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


async def _follow(port: str | None, phone: str) -> int:
    from openhead.client import Head, resolve_device

    from .follow import Follower
    host, hport = await resolve_device(phone)
    head = await Head.connect(host, hport, client="openhead-body")
    if not head.paired:
        print(f"not paired with {head.key}: run `openhead pair --device {phone}` once", file=sys.stderr)
        await head.close()
        return 1
    neck = await asyncio.to_thread(Neck.open, port)
    follower = Follower(neck, head)
    try:
        await follower.start()
        print("following; Ctrl+C stops")
        while follower.running:
            await asyncio.sleep(0.5)
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        await follower.stop()
        await head.close()
        await asyncio.to_thread(neck.close)
    return 0


if __name__ == "__main__":
    sys.exit(main())
