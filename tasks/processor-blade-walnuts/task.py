"""On the station table, in front of the arms: the clear work bowl of a 7-cup food processor (KitchenAid KFP0718
type), off its motor base and with its lid off, holding 40 pieces of chopped walnut. Its multipurpose S-blade is
still in it, on its black drive adapter, which sits loose over the bowl's centre post: the adapter's 14 mm stem stands
up in the middle of the bowl and the blade's two steel arms lie down among the nuts. The bowl's loop handle points
back toward the right arm. Front right stands a blue melamine mixing bowl; front left lies a green cutting board.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from bladekit import (Contacts, PartState, Slide, on_env_reset, on_physics, on_reset, part, rot_z,  # noqa: E402
                      set_slide_enabled, set_slide_target)
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Put the chopped walnuts in the mixing bowl."

CLEAR, BLACK, STEEL = (0.70, 0.84, 0.95), (0.10, 0.10, 0.11), (0.72, 0.74, 0.77)
BLUE, GREEN = (0.15, 0.32, 0.72), (0.22, 0.58, 0.30)
WALNUT = ((0.58, 0.40, 0.22), (0.66, 0.48, 0.28), (0.50, 0.34, 0.18))


def ring(r_in: float, wall: float, z0: float, z1: float, color, n: int = 16, skip=()) -> list:
    """A round wall as n flat boxes: inner radius (apothem) ``r_in``, thickness ``wall``, from z0 to z1."""
    r = r_in + wall / 2
    side = 2 * (r_in + wall) * math.tan(math.pi / n) + 0.001
    return [((r * math.cos(a), r * math.sin(a), (z0 + z1) / 2), (wall, side, z1 - z0), color, a)
            for i in range(n) if i not in skip for a in [2 * math.pi * i / n]]


def disk(radius: float, z0: float, z1: float, color, strips: int = 10) -> list:
    """A round plate as ``strips`` boxes along x, each as long as the circle is at the strip's inner edge."""
    w = 2 * radius / strips
    out = []
    for i in range(strips):
        x0, x1 = -radius + i * w, -radius + (i + 1) * w
        inner = min(abs(x0), abs(x1)) if x0 * x1 > 0 else 0.0
        half = math.sqrt(max(radius**2 - inner**2, 0.0))
        out.append((((x0 + x1) / 2, 0.0, (z0 + z1) / 2), (w, 2 * half, z1 - z0), color))
    return out


# -- the processor's work bowl (origin: centre of its base, on the table) -------------------------------------------
BOWL_XY = (0.36, 0.09)
R_IN, WALL, FLOOR = 0.080, 0.005, 0.005        # 16 cm inside, 5 mm wall and floor
RIM = 0.115                                    # rim 11.5 cm above the table
LIP, LIP_H = 0.010, 0.015                      # lid-lock rim: the top 1.5 cm of wall is 1 cm thick
POST, POST_TOP = 0.008, 0.0875                 # centre post: 16 mm square, to 8.75 cm
HANDLE_AT = -math.pi / 2                       # the handle points to the right (-y), toward the right arm
_hc, _hs = math.cos(HANDLE_AT), math.sin(HANDLE_AT)
HANDLE_OUT, HANDLE_ZT, HANDLE_ZB = 0.055, RIM - 0.015, 0.055
HANDLE_BAR_R = R_IN + WALL + HANDLE_OUT - 0.008   # the grip bar's centre: 13.2 cm from the bowl's axis ...
HANDLE_BAR_Z = (HANDLE_ZT + HANDLE_ZB) / 2        # ... and 7.75 cm above the table


def _handle():
    """A loop handle: two arms (2 cm wide, 2 cm tall) reach 5.5 cm out from the wall, 1.5 cm below the rim and 5.5 cm
    above the base, joined by an upright grip bar 1.6 cm thick (radially) and 2.5 cm wide, 6.5 cm tall. Between bar and
    wall is a 3.9 cm opening for the fingers. The grip bar has a textured grip, friction 1.2."""
    r0 = R_IN + WALL
    parts = [((r0 + HANDLE_OUT / 2, 0.0, HANDLE_ZT), (HANDLE_OUT, 0.02, 0.02)),
             ((r0 + HANDLE_OUT / 2, 0.0, HANDLE_ZB), (HANDLE_OUT, 0.02, 0.02)),
             ((HANDLE_BAR_R, 0.0, HANDLE_BAR_Z), (0.016, 0.025, HANDLE_ZT - HANDLE_ZB + 0.02))]
    return [((c[0] * _hc, c[0] * _hs, c[2]), size, CLEAR, HANDLE_AT, mu)
            for (c, size), mu in zip(parts, (None, None, 1.2))]


BOWL_BOXES = (disk(R_IN + WALL, 0.0, FLOOR, CLEAR) + ring(R_IN, WALL, FLOOR - 0.001, RIM - LIP_H, CLEAR)
              + ring(R_IN, LIP, RIM - LIP_H, RIM, CLEAR) + _handle()
              + [((0.0, 0.0, (FLOOR + POST_TOP) / 2), (2 * POST, 2 * POST, POST_TOP - FLOOR), CLEAR)])
BOWL_MASS = 0.40
BOWL = part("bowl", pos=(*BOWL_XY, TOP), boxes=BOWL_BOXES, mass=BOWL_MASS, color=CLEAR, friction=0.4)

# -- the blade on its drive adapter (origin: the bowl's origin when seated) -----------------------------------------
HUB_IN, HUB_WALL = 0.0105, 0.005               # adapter sleeve: 21 mm bore over the 16 mm post, 31 mm across
HUB_Z0, CAP_Z0 = 0.007, POST_TOP + 0.0005       # sleeve from 7 mm above the floor; its cap 0.5 mm above the post
CAP_Z1 = CAP_Z0 + 0.004
STEM, STEM_TOP = 0.007, 0.105                  # 14 mm square stem, top 1 cm below the rim
ARM_LEN, ARM_W, ARM_T = 0.054, 0.020, 0.003   # two steel arms reaching 7 cm from the axis
_hub = HUB_IN + HUB_WALL
BLADE_BOXES = ([((sx * (HUB_IN + HUB_WALL / 2), 0.0, (HUB_Z0 + CAP_Z0) / 2), (HUB_WALL, 2 * _hub, CAP_Z0 - HUB_Z0),
                 BLACK) for sx in (1, -1)]
               + [((0.0, sy * (HUB_IN + HUB_WALL / 2), (HUB_Z0 + CAP_Z0) / 2), (2 * HUB_IN, HUB_WALL, CAP_Z0 - HUB_Z0),
                   BLACK) for sy in (1, -1)]
               + [((0.0, 0.0, (CAP_Z0 + CAP_Z1) / 2), (2 * _hub, 2 * _hub, CAP_Z1 - CAP_Z0), BLACK),
                  ((0.0, 0.0, (CAP_Z1 + STEM_TOP) / 2), (2 * STEM, 2 * STEM, STEM_TOP - CAP_Z1), BLACK),
                  ((_hub + ARM_LEN / 2, 0.0, 0.014), (ARM_LEN, ARM_W, ARM_T), STEEL, 0.35),
                  ((-_hub - ARM_LEN / 2, 0.0, 0.034), (ARM_LEN, ARM_W, ARM_T), STEEL, 0.35)])
BLADE_MASS = 0.10


def centroid(boxes) -> np.ndarray:
    """Volume-weighted centre of boxes (PhysX's centre of mass for a body of uniform density)."""
    v = np.array([np.prod(b[1]) for b in boxes])
    return (v[:, None] * np.array([b[0] for b in boxes])).sum(0) / v.sum()


BLADE_COM = centroid(BLADE_BOXES)
TRAVEL = POST_TOP - HUB_Z0                      # the sleeve clears the post after 8.05 cm of travel
BLADE = part("blade", pos=(*BOWL_XY, TOP), boxes=BLADE_BOXES, mass=BLADE_MASS, color=BLACK, friction=0.8,
             slide=Slide("bowl", pos0=tuple(BLADE_COM), pos1=tuple(BLADE_COM), travel=TRAVEL), contacts=True)

# -- mixing bowl and cutting board (fixed) --------------------------------------------------------------------------
MIX_XY, MIX_R, MIX_H = (0.34, -0.19), 0.110, 0.095   # 22 cm inside, 9.5 cm tall
MIX = part("mixing_bowl", pos=(*MIX_XY, TOP), boxes=disk(MIX_R + 0.004, 0.0, 0.005, BLUE)
           + ring(MIX_R, 0.004, 0.004, MIX_H, BLUE), mass=0.6, color=BLUE, fixed=True)
BOARD_XY, BOARD_SIZE = (0.40, 0.30), (0.30, 0.20, 0.013)
BOARD = part("board", pos=(*BOARD_XY, TOP), boxes=[((0.0, 0.0, BOARD_SIZE[2] / 2), BOARD_SIZE, GREEN)], mass=0.5,
             color=GREEN, fixed=True)

# -- the walnuts: 40 rigid pieces, 9-13 mm, 0.65 g/cm^3 -------------------------------------------------------------
N_NUTS = 40
_rng = np.random.default_rng(7)


def _nuts():
    out, k = [], 0
    for layer in range(4):
        for i in range(10):
            a = 2 * math.pi * (i + 0.5 * layer) / 10
            r = 0.045 + 0.018 * (layer % 2)
            size = tuple(float(s) for s in np.round(_rng.uniform(0.009, 0.013, 3), 4))
            pos = (BOWL_XY[0] + r * math.cos(a), BOWL_XY[1] + r * math.sin(a), TOP + 0.048 + 0.014 * layer)
            out.append(part(f"nut_{k}", pos=pos, boxes=[((0.0, 0.0, 0.0), size, WALNUT[k % 3])],
                            mass=round(650 * float(np.prod(size)), 5), yaw=float(_rng.uniform(0, math.pi)),
                            color=WALNUT[k % 3], friction=0.3))
            k += 1
    return out


NUTS = _nuts()
NUT_NAMES = [n.name for n in NUTS]
OBJECTS = [BOWL, BLADE, MIX, BOARD, *NUTS]

# -- the blade on the post: breakaway friction that wedges when the hub is tilted ----------------------------------
# Every physics step the hub's sideways load F (N) and tilting moment M (N m) about its centre are read from how far
# the joint's springs are deflected (the moment only beyond the sleeve's play, TILT_PLAY). Sliding along the post
# takes F_B = F0 + MU_SIDE * F + MU_WEDGE * M / BEARING.
# A sideways load alone (the blade's weight in a tipped bowl) adds little; a moment cocks the sleeve on the post and
# wedges it. A pull on the stem at angle a off the post's axis, h above the hub's centre, therefore slides the
# blade only while tan(a) < 1 / (MU_SIDE + MU_WEDGE h / BEARING): 8 degrees at the stem.
F0, MU_SIDE, MU_WEDGE, BEARING = 0.05, 0.1, 5.4, 0.035
# The sleeve rocks a little on the post before it bears on two edges: tilt within that play carries no moment.
TILT_PLAY = math.radians(0.6)
STEM_LEVER = STEM_TOP - 0.010 - BLADE_COM[2]    # grip 1 cm below the stem's top, above the hub's centre


def jam_angle_deg() -> float:
    return math.degrees(math.atan(1.0 / (MU_SIDE + MU_WEDGE * STEM_LEVER / BEARING)))


def breakaway(side_n: float, moment_nm: float) -> float:
    return F0 + MU_SIDE * side_n + MU_WEDGE * moment_nm / BEARING


def drag(travel: float, target: float, limit_n: float, k: float) -> float:
    """Breakaway friction as a spring of stiffness k dragged along: the target follows the blade, at most
    limit_n / k behind it."""
    lag = limit_n / k
    return min(max(target, travel - lag), travel + lag)


_S: dict = {}
DEBUG = bool(os.environ.get("BLADE_DEBUG"))


def _reset_state(environment=None) -> None:
    _S.update(target=0.0, attached=True, released_unheld=False, riding=False, in_mix=False, on_floor=False,
              edge_up=False,
              travel=0.0, held=None, side=0.0, moment=0.0, fb=F0, t=0.0, log=[])
    if environment is not None:
        set_slide_enabled("blade", True)
        set_slide_target("blade", 0.0)


_reset_state()


@on_env_reset
def _env_reset(environment) -> None:
    _reset_state(environment)


@on_reset
def _after_settle(environment) -> None:
    for k in ("released_unheld", "riding", "in_mix", "on_floor", "edge_up"):
        _S[k] = False
    _S["t"] = 0.0
    _S["log"] = []


def _say(msg: str) -> None:
    _S["log"].append(msg)
    print(f"[blade] t {_S['t']:6.2f} {msg}", flush=True)


_CONTACTS: dict = {}


def held_by(forces: dict, min_n: float = 0.2) -> str | None:
    """The arm whose two finger pads both press on the blade, if any."""
    for arm, f in forces.items():
        if len(f) == 2 and min(f) > min_n:
            return arm
    return None


def relative(bowl_R, bowl_p, blade_R, blade_p):
    """(travel along the post, sideways offset (m), tilt (rad)) of the blade's joint point from the post axis."""
    c = BLADE_COM
    d = bowl_R.T @ (blade_p + blade_R @ c - bowl_p) - c
    axis = bowl_R.T @ blade_R[:, 2]
    return float(d[2]), float(math.hypot(d[0], d[1])), float(math.acos(min(1.0, max(-1.0, axis[2]))))


@on_physics
def _mechanics(environment) -> None:
    """Breakaway friction of the hub on the post, release at the post's top, and the blade latches."""
    from bladekit import _np, quat_to_matrix

    base = environment.unwrapped
    dt = float(base.physics_dt)
    _S["t"] += dt
    if "c" not in _CONTACTS:
        _CONTACTS["c"] = Contacts(environment, "blade")
    held = held_by(_CONTACTS["c"].forces())
    _S["held"] = held
    origin = _np(base.scene.env_origins)[0]
    pb = _np(base.scene["bowl"].data.root_link_pose_w)[0]
    pl = _np(base.scene["blade"].data.root_link_pose_w)[0]
    Rb, Rl = quat_to_matrix(pb[3:7]), quat_to_matrix(pl[3:7])
    if DEBUG and round(_S["t"] / dt) % 60 == 0:
        _say(f"attached {_S['attached']} travel {_S['travel'] * 100:.2f} cm side {_S['side']:.2f} N tilt "
             f"{math.degrees(_S.get('tilt', 0.0)):.2f} deg moment "
             f"{_S['moment']:.3f} N m breakaway {_S['fb']:.2f} N held {held} bowl tilt "
             f"{math.degrees(math.acos(min(1, Rb[2, 2]))):.0f} deg pos {np.round(pb[:3] - origin, 3)} handle dir "
             f"{np.round(Rb @ np.array([_hc, _hs, 0.0]), 2)} up {np.round(Rb[:, 2], 2)}")
    if _S["attached"]:
        s = BLADE.slide
        travel, off, tilt = relative(Rb, pb[:3], Rl, pl[:3])
        _S["travel"], _S["side"], _S["tilt"] = travel, s.lateral_k * off, tilt
        _S["moment"] = s.tilt_k * max(0.0, tilt - TILT_PLAY)
        _S["fb"] = breakaway(_S["side"], _S["moment"])
        target = drag(travel, _S["target"], _S["fb"], s.axial_k)
        if abs(target - _S["target"]) > 1e-5:
            _S["target"] = target
            set_slide_target("blade", target)
        if travel > RIDE_M and held is None and not _S["riding"]:
            _S["riding"] = True
            _say(f"LATCH: the blade rode {travel * 100:.1f} cm up the post with no gripper on it (bowl tilt "
                 f"{math.degrees(math.acos(min(1, Rb[2, 2]))):.0f} deg)")
        if travel >= TRAVEL:
            set_slide_enabled("blade", False)
            _S["attached"] = False
            if held is None:
                _S["released_unheld"] = True
                _say(f"LATCH: the blade slid off the post with no gripper on it (bowl tilt "
                     f"{math.degrees(math.acos(min(1, Rb[2, 2]))):.0f} deg)")
            else:
                _say(f"blade lifted off the post by the {held}")
        return
    p = pl[:3] - origin + Rl @ BLADE_COM
    if not _S["in_mix"] and in_mixing_bowl(p):
        _S["in_mix"] = True
        _say("LATCH: the blade is in the mixing bowl")
    if not _S["on_floor"] and p[2] < TOP - 0.02:
        _S["on_floor"] = True
        _say("LATCH: the blade fell off the table")
    v = _np(base.scene["blade"].data.root_vel_w)[0]
    tilt = math.degrees(math.acos(min(1.0, abs(Rl[2, 2]))))
    if (not _S["edge_up"] and held is None and p[2] < TOP + 0.06 and np.linalg.norm(v[:3]) < 0.02 and tilt > 45
            and not in_processor_bowl(p, pb[:3] - origin, Rb)):
        _S["edge_up"] = True
        _say(f"LATCH: the blade lies on its side, an edge up ({tilt:.0f} deg)")


def in_mixing_bowl(p) -> bool:
    d = np.asarray(p, float) - np.array([*MIX_XY, TOP])
    return bool(math.hypot(d[0], d[1]) < MIX_R and 0.0 < d[2] < MIX_H + 0.02)


def in_processor_bowl(p, bowl_pos, bowl_R) -> bool:
    d = bowl_R.T @ (np.asarray(p, float) - np.asarray(bowl_pos, float))
    return bool(math.hypot(d[0], d[1]) < R_IN + 0.002 and 0.0 < d[2] < RIM + 0.005)


# -- scoring --------------------------------------------------------------------------------------------------------
FULL_FRACTION = 0.9      # 90 % of the pieces (by mass) in the mixing bowl is a full transfer
BOARD_TILT_DEG = 20.0


def nut_fraction(objects) -> float:
    bowl = objects["bowl"]
    total = sum(n.mass for n in NUTS)
    got = sum(n.mass for n in NUTS if in_mixing_bowl(objects[n.name].pos)
              and not in_processor_bowl(objects[n.name].pos, bowl.pos, bowl.rotation()))
    return got / total


RIDE_M = 0.02           # a blade that rides 2 cm up the post with no gripper on it is on its way out: latched
SEATED_M = 0.01          # on the post counts as stowed only while the sleeve is within 1 cm of fully down


def blade_stowed(blade: PartState) -> bool:
    """Seated on the post (within 1 cm of fully down), or released and standing on its hub (upright within 20
    degrees) on the cutting board, at rest. A blade sliding up the post in a tipped bowl is not stowed."""
    if _S["attached"]:
        return _S["travel"] < SEATED_M
    p = blade.to_world(BLADE_COM)
    on_board = (abs(p[0] - BOARD_XY[0]) < BOARD_SIZE[0] / 2 and abs(p[1] - BOARD_XY[1]) < BOARD_SIZE[1] / 2
                and p[2] < TOP + BOARD_SIZE[2] + 0.06)
    return bool(on_board and math.degrees(blade.tilt()) < BOARD_TILT_DEG and _S["held"] is None
                and blade.resting(0.01, 0.5))


def latched() -> bool:
    return bool(_S["released_unheld"] or _S["riding"] or _S["in_mix"] or _S["on_floor"] or _S["edge_up"])


def score(objects) -> float:
    """0 if, at any time, the blade rode 2 cm or more up the post (or off it) with no gripper holding it, was in the
    mixing bowl, fell off the table, or lay at rest on its side (tilted over 45 degrees). Otherwise the fraction of
    the walnut mass in the mixing bowl (not inside the processor bowl), divided by 0.9 and capped at 1, times 0.8
    unless at the end the blade is seated on the post or stands released on the cutting board."""
    if latched():
        return 0.0
    base = min(1.0, nut_fraction(objects) / FULL_FRACTION)
    return base if blade_stowed(objects["blade"]) else 0.8 * base


def success(objects) -> bool:
    """Full score with the blade stowed; must hold 1 s."""
    return score(objects) >= 1.0 and blade_stowed(objects["blade"])


# -- scripted demo (closed loop; see bladedemo.py) -------------------------------------------------------------------


def controller(bases, dt):
    """yam_scripted plays this: it sees the top camera image and the joint positions only."""
    import types

    import bladedemo

    return bladedemo.Controller(bases, dt, types.SimpleNamespace(**globals()))
