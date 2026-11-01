"""
Fallreflexens poser (rad) per fallriktning, samma som strategin "skydda" i sim/fall_test.py, och uppladdning till
Uno i råenheter via joints.json (tecken och offset per led).

  framåt   halsen så långt bakåt det går (skärmen bort från golvet), benen i standardposen
  bakåt    huvudet framåt och benen hukar (standardposen × 1,5)
  vänster  skärmen vrids åt höger och huvudet bakåt
  höger    skärmen vrids åt vänster och huvudet bakåt
"""
from . import protocol as P
from .joints import DEFAULT_POSE, LIMITS

HEAD_BACK = {"neck_pitch": 1.13, "head_pitch": 0.78}     # ledernas gränser: huvudet bakåt, skärmen uppåt
HEAD_FWD = {"neck_pitch": -0.3, "head_pitch": -0.7}
LEGS = [n for n in P.JOINT_NAMES if n.startswith(("left_", "right_"))]
DIR_NAMES = {P.FALL_FWD: "framåt", P.FALL_BACK: "bakåt", P.FALL_LEFT: "vänster", P.FALL_RIGHT: "höger"}
DEFAULTS = {"deg": 35, "horizon_ms": 150, "limp_ms": 1500}  # sim/fall_test.py --detect prognos --detect-deg 35


def poses():
    """{riktning: {led: rad}} för alla 14 leder."""
    out = {}
    for d in DIR_NAMES:
        q = dict(DEFAULT_POSE)
        if d == P.FALL_BACK:
            for n in LEGS:
                lo, hi = LIMITS[n]
                q[n] = min(max(DEFAULT_POSE[n] * 1.5, lo), hi)
            q.update(HEAD_FWD)
        else:
            q.update(HEAD_BACK)
            if d == P.FALL_LEFT:
                q["head_yaw"] = -1.2
            elif d == P.FALL_RIGHT:
                q["head_yaw"] = 1.2
        out[d] = q
    return out


def raw_poses(joints):
    """{riktning: [14 råmål]} med ledkonfigurationen (joints.json)."""
    return {d: [joints.to_raw(n, q[n]) for n in P.JOINT_NAMES] for d, q in poses().items()}


def upload(uno, joints, deg=DEFAULTS["deg"], horizon_ms=DEFAULTS["horizon_ms"], limp_ms=DEFAULTS["limp_ms"]):
    """Laddar upp poserna och sätter parametrarna; deg = 0 stänger av reflexen."""
    for d, raw in raw_poses(joints).items():
        assert uno.set_reflex(d, raw) == raw
    uno.set_param(P.PARAM_FALL_HORIZON_MS, horizon_ms)
    uno.set_param(P.PARAM_FALL_LIMP_MS, limp_ms)
    uno.set_param(P.PARAM_FALL_DEG, deg)
