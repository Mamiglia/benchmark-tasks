"""On the station table: a closed white square jug-style bottle (6 x 6 cm, 16.3 cm tall with its cap, its neck at
the edge nearest the arms) holding 40 small blue beads, a little left of centre; and an empty clear-grey square cup
(8 x 8 x 7 cm) between the bottle and the arms. The bottle's orange cap can be opened in two ways: it unscrews (it
comes off after 1.25 turns counterclockwise, seen from above), and the flip-top lid on the cap snaps open when its
front tab, which points toward the right arm, is pushed up. The robot must open the bottle and pour the beads into
the cup.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from jointkit import (Attach, Hinge, PartState, ball, on_reset, on_step, part, rot_z, set_attach_target,
                      set_attached, set_hinge_target)  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Open the bottle and pour its contents into the cup."

WHITE, ORANGE, GREY, BLUE = (0.92, 0.92, 0.90), (1.0, 0.45, 0.05), (0.75, 0.78, 0.80), (0.10, 0.30, 0.90)


def _tube(half_x, half_y, wall, z0, z1, color):
    """Four walls of a hollow rectangle: outer half-widths, wall thickness, from z0 to z1."""
    zc, h = (z0 + z1) / 2, z1 - z0
    return [((half_x - wall / 2, 0, zc), (wall, 2 * half_y, h), color),
            ((-half_x + wall / 2, 0, zc), (wall, 2 * half_y, h), color),
            ((0, half_y - wall / 2, zc), (2 * half_x - 2 * wall, wall, h), color),
            ((0, -half_y + wall / 2, zc), (2 * half_x - 2 * wall, wall, h), color)]


def _plate(half, hole, z0, z1, color):
    """A square plate (half-width ``half``) with a square hole (half-width ``hole``) in its centre."""
    zc, h, w = (z0 + z1) / 2, z1 - z0, half - hole
    return [((hole + w / 2, 0, zc), (w, 2 * half, h), color), ((-hole - w / 2, 0, zc), (w, 2 * half, h), color),
            ((0, hole + w / 2, zc), (2 * hole, w, h), color), ((0, -hole - w / 2, zc), (2 * hole, w, h), color)]


# -- the bottle (origin: centre of its base) --------------------------------------------------------------------------
BX, BY = 0.26, 0.06
B_HALF, B_WALL, BODY_H = 0.030, 0.004, 0.110        # 6 x 6 cm body, 4 mm walls, 11 cm tall (cap clear of the left hand)
NECK_HALF, NECK_HOLE = 0.013, 0.010                  # 2.6 cm neck with a 2.0 cm bore ...
NECK_X = -(B_HALF - NECK_HALF)                       # ... flush with the -x wall (facing the cup), as on a jug
NECK_TOP = BODY_H + 0.037                            # neck from the shoulder (11.3 cm) to 14.7 cm
_SH0, _SH1 = BODY_H, BODY_H + 0.003                  # the flat shoulder, with the neck's bore cut out of it
_HX0, _HX1 = NECK_X - NECK_HOLE, NECK_X + NECK_HOLE
SHOULDER = [(((_HX1 + B_HALF) / 2, 0, (_SH0 + _SH1) / 2), (B_HALF - _HX1, 2 * B_HALF, 0.003), WHITE),
            (((_HX0 - B_HALF) / 2, 0, (_SH0 + _SH1) / 2), (_HX0 + B_HALF, 2 * B_HALF, 0.003), WHITE)] + [
            ((NECK_X, sy * (NECK_HOLE + B_HALF) / 2, (_SH0 + _SH1) / 2), (2 * NECK_HOLE, B_HALF - NECK_HOLE, 0.003),
             WHITE) for sy in (1, -1)]
NECK = [((NECK_X + c[0], c[1], c[2]), size, col)
        for c, size, col in _tube(NECK_HALF, NECK_HALF, 0.003, _SH1, NECK_TOP, WHITE)]
BOTTLE_BOXES = ([((0, 0, B_WALL / 2), (2 * B_HALF, 2 * B_HALF, B_WALL), WHITE)]
                + _tube(B_HALF, B_HALF, B_WALL, B_WALL, BODY_H, WHITE) + SHOULDER + NECK)
# 0.17 kg, plus 32 g of beads: about an empty rigid bottle plus the 200 mL of water the beads stand in for
BOTTLE = part("bottle", pos=(BX, BY, TOP), boxes=BOTTLE_BOXES, mass=0.17, color=WHITE, friction=1.0)

# -- the cap (origin: centre of its top plate's underside, resting on the neck's rim) ------------------------------
CAP_HALF, ORIFICE, SKIRT = 0.023, 0.010, 0.010       # 4.6 cm cap, 2.0 cm flip-top opening, 1 cm skirt
CAP_YAW = math.radians(45)                           # its tab points front-right, toward the right arm
CAP_BOXES = _plate(CAP_HALF, ORIFICE, 0.0, 0.003, ORANGE) + _tube(CAP_HALF, CAP_HALF, 0.003, -SKIRT, 0.0, ORANGE)
TAB = 0.015                                          # the lid's thumb tab sticks out 1.5 cm past the cap
LID = Hinge("lid_hinge", anchor=(0.0, CAP_HALF, 0.003), yaw=math.pi,  # hinge along the cap's back (+y) edge
            boxes=(((0, -CAP_HALF, 0.0015), (2 * CAP_HALF, 2 * CAP_HALF, 0.003), ORANGE),
                   ((0, -2 * CAP_HALF - TAB / 2, 0.0015), (0.024, TAB, 0.003), ORANGE)),
            lower=0.0, upper=math.radians(120), stiffness=0.3, damping=0.002, mass=0.003)
# The thread: a real revolute joint about the neck's axis. Its friction is a stiff spring toward a set angle that
# _mechanisms drags along whenever the cap has turned more than THREAD_SLIP past it: the cap holds until about
# 0.2 N m is applied, then slips at about that torque, which reacts on the bottle (an unheld bottle turns with its
# cap). It is switched off after 1.25 turns of net loosening.
THREAD_STIFFNESS, THREAD_SLIP = 3.0, math.radians(4)  # N m/rad, rad: breakaway about 0.21 N m
THREAD = Attach("bottle", pos0=(NECK_X, 0.0, NECK_TOP), rot1=-CAP_YAW, stiffness=THREAD_STIFFNESS, damping=0.02)
CAP = part("cap", pos=(BX + NECK_X, BY, TOP + NECK_TOP), boxes=CAP_BOXES, mass=0.008, yaw=CAP_YAW, hinge=LID,
           color=ORANGE, attach=THREAD)

# -- the cup (origin: centre of its base) --------------------------------------------------------------------------
CX, CY = 0.15, 0.06
CUP_HALF, CUP_WALL, CUP_H = 0.040, 0.004, 0.070
CUP = part("cup", pos=(CX, CY, TOP),
           boxes=[((0, 0, CUP_WALL / 2), (2 * CUP_HALF, 2 * CUP_HALF, CUP_WALL), GREY)]
           + _tube(CUP_HALF, CUP_HALF, CUP_WALL, CUP_WALL, CUP_H, GREY), mass=0.15, color=GREY, friction=0.8)

# -- the beads: 40 x 6 mm, 5 x 4 x 2 on the bottle's floor ----------------------------------------------------------
BEAD_R, N_BEADS = 0.003, 40
BEAD_MASS = 0.0008  # heavier than a real 6 mm bead (0.3 g): light beads in a heavy bottle are unstable in PhysX
BEADS = [ball(f"bead_{i}", (BX + (i % 5 - 2) * 0.009, BY + (i // 5 % 4 - 1.5) * 0.009,
                             TOP + B_WALL + BEAD_R + 0.0005 + (i // 20) * 0.009), BEAD_R, BEAD_MASS, BLUE)
         for i in range(N_BEADS)]
BEAD_NAMES = [b.name for b in BEADS]
OBJECTS = [BOTTLE, CAP, CUP, *BEADS]

# -- mechanisms -----------------------------------------------------------------------------------------------------
RELEASE = 2.5 * math.pi          # the thread lets go after 1.25 turns (450 degrees) of net loosening
LID_SNAP = math.radians(15)      # detent: below 15 degrees the lid's spring pulls it shut, past it pulls it open
LID_OPEN = math.radians(115)     # where the open lid rests (joint stop at 120)
OPEN_ANGLE = math.radians(60)    # the lid counts as open from 60 degrees


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _twist(bottle: PartState, cap: PartState) -> float:
    """The cap's turn about the bottle's axis, relative to the bottle (rad, counterclockwise from above +)."""
    r = bottle.rotation().T @ cap.rotation()
    return math.atan2(r[1, 0], r[0, 0])


def _slipped(twist: float, hold: float) -> float | None:
    """Breakaway friction: once the cap has turned more than THREAD_SLIP from the thread's set angle ``hold``, the set
    angle follows it, THREAD_SLIP behind (so the drive pulls back with about THREAD_STIFFNESS * THREAD_SLIP)."""
    slip = _wrap(twist - hold)
    return _wrap(twist - math.copysign(THREAD_SLIP, slip)) if abs(slip) > THREAD_SLIP else None


_STATE: dict = {}


def _reset_state(environment=None) -> None:
    _STATE.update(turned=0.0, twist=None, released=False, opened=False, cup_knocked=False, lid_target=0.0,
                  hold=CAP_YAW)
    if environment is not None:
        set_hinge_target(environment, "cap", 0.0)
        set_attach_target(environment, "cap", 0.0)
        set_attached(environment, "cap", True)


_reset_state()
on_reset(_reset_state)


@on_step
def _mechanisms(environment) -> None:
    """The flip-top's detent, and counting the cap's turns until the thread lets go (see README, "Mechanisms")."""
    from yam_arena.scene import object_states

    s = object_states(environment, ["bottle", "cap"])
    bottle, cap = s["bottle"], s["cap"]
    target = LID_OPEN if cap.hinge > LID_SNAP else 0.0
    if target != _STATE["lid_target"]:
        set_hinge_target(environment, "cap", target)
        _STATE["lid_target"] = target
    if not _STATE["released"]:
        twist = _twist(bottle, cap)
        if _STATE["twist"] is not None:
            _STATE["turned"] += _wrap(twist - _STATE["twist"])
        _STATE["twist"] = twist
        hold = _slipped(twist, _STATE["hold"])
        if hold is not None:
            _STATE["hold"] = hold
            set_attach_target(environment, "cap", _wrap(hold - CAP_YAW))
        if _STATE["turned"] >= RELEASE:
            set_attached(environment, "cap", False)  # the cap is now a free body resting on the neck's rim
            _STATE["released"] = True


# -- scoring --------------------------------------------------------------------------------------------------------
CUP_MOVE, CUP_TILT = 0.03, math.radians(20)
CAP_OFF = 0.03                                    # the cap is off when it is 3 cm from its seat on the neck
CREDIT = ((32, 1.0), (20, 0.75), (4, 0.5))        # beads in the cup: 80 %, 50 %, 10 % of 40


def _in_cup(p, cup: PartState, bottle: PartState) -> bool:
    """Inside the cup's walls, below its rim, and not inside the bottle (a bottle dropped into the cup doesn't
    count its beads)."""
    c, b = cup.to_local(p), bottle.to_local(p)
    in_bottle = abs(b[0]) < B_HALF - B_WALL and abs(b[1]) < B_HALF - B_WALL and 0 < b[2] < NECK_TOP
    return bool(abs(c[0]) < CUP_HALF - CUP_WALL and abs(c[1]) < CUP_HALF - CUP_WALL and 0 < c[2] < CUP_H
                and not in_bottle)


def _update(objects) -> tuple[float, bool]:
    bottle, cap, cup = objects["bottle"], objects["cap"], objects["cup"]
    _STATE["cup_knocked"] |= bool(np.linalg.norm(cup.pos[:2] - np.array(CUP.pos[:2])) > CUP_MOVE
                                  or cup.tilt() > CUP_TILT)
    seat = bottle.to_world((NECK_X, 0.0, NECK_TOP))
    _STATE["opened"] |= bool(cap.hinge >= OPEN_ANGLE or np.linalg.norm(cap.pos - seat) > CAP_OFF)
    if _STATE["cup_knocked"] or not _STATE["opened"]:
        return 0.0, False
    inside = [objects[n] for n in BEAD_NAMES if _in_cup(objects[n].pos, cup, bottle)]
    # beads count as settled under 1 cm/s; their spin is ignored (a 6 mm bead rolling at 2 mm/s spins at 0.7 rad/s)
    settled = sum(float(np.linalg.norm(b.lin_vel)) < 0.01 for b in inside) >= CREDIT[0][0] and cup.resting()
    n = len(inside)
    return next((c for k, c in CREDIT if n >= k), 0.25), settled


def success(objects) -> bool:
    """Opened either way, at least 32 of the 40 beads at rest in the cup, the cup never knocked; held 1 s."""
    return _update(objects)[1]


def score(objects) -> float:
    """0 if the cup was ever moved 3 cm or tilted 20 degrees, or the bottle was never opened. Otherwise 0.25 for
    opening it (lid open 60 degrees or cap 3 cm off its seat), 0.5 for 4 beads in the cup, 0.75 for 20, 1.0 for 32."""
    return _update(objects)[0]


# -- scripted demos (pure NumPy plans; demo() wraps them in yam_arena Waypoints) -------------------------------------
OPEN, SHUT = -0.04695, 0.0        # finger joint targets (m): yam_arena.constants.FINGER_OPEN / FINGER_CLOSED


def frame(z, x) -> np.ndarray:
    """Gripper rotation with approach axis ``z`` and finger-closing axis ``x`` (world)."""
    z = np.asarray(z, float) / np.linalg.norm(z)
    x = np.asarray(x, float) - z * np.dot(x, z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def down(yaw: float) -> np.ndarray:
    """Straight-down grip turned ``yaw`` about the vertical (yam_arena.demo.grip_rotation)."""
    return rot_z(yaw) @ frame((0, 0, -1), (0, -1, 0))


# Left arm: pinches the bottle's front and back faces 7 cm up, from its left, pitched 20 degrees down.
HOLD = frame((0, -math.cos(math.radians(20)), -math.sin(math.radians(20))), (-1, 0, 0))
GRASP = (BX, BY, TOP + 0.07)
POUR_TILT = -2.0                   # 115 degrees about the world y axis: the bottle's top swings back over the cup
POUR_AT = (BX - 0.02, BY, TOP + 0.20)  # the neck ends over the cup's centre


def _left(wait_s: float):
    """(duration, pos, rot, finger, label): grasp, hold while the right arm opens the bottle, lift, pour."""
    pre = (BX, BY + 0.066, TOP + 0.094)
    return [(2.0, pre, HOLD, OPEN, "to the bottle"), (1.5, GRASP, HOLD, OPEN, "around the bottle"),
            (0.8, GRASP, HOLD, SHUT, "grip"), (wait_s, GRASP, HOLD, SHUT, "hold steady"),
            (2.0, (BX, BY, TOP + 0.20), HOLD, SHUT, "lift"),
            (3.0, POUR_AT, rot_y(POUR_TILT) @ HOLD, SHUT, "tip over the cup"),
            (8.0, POUR_AT, rot_y(POUR_TILT) @ HOLD, SHUT, "pour")]


def _flip():
    """Right arm, fingers closed and pointing left: under the lid's tab, then straight up until the lid snaps open."""
    probe = frame((0, 1, 0), (1, 0, 0))
    # pad tips under the tab's middle; the 27 mm tall fingertips end 2 mm below it
    x, y, z = BX + NECK_X + 0.0212, BY - 0.0342, TOP + NECK_TOP - 0.0127
    return [(4.5, (x, BY - 0.12, z), probe, SHUT, "beside the tab"), (1.5, (x, y, z), probe, SHUT, "under the tab"),
            (1.2, (x, y, z + 0.02), probe, SHUT, "lift the tab"),
            (1.5, (x, y + 0.02, z + 0.045), probe, SHUT, "flip the lid"),
            (1.5, (x, BY - 0.15, z + 0.04), probe, SHUT, "away")]


STROKES = 4


def _unscrew():
    """Right arm, straight down: pinch the cap's side faces and turn it 180 degrees counterclockwise, four times
    (720 commanded; each stroke slips a few degrees, and the thread lets go at 450), going out and back in diagonally
    between turns; lift the cap off, drop it aside."""
    z = TOP + NECK_TOP - SKIRT + 0.002 + 0.013  # pad tips 2 mm above the skirt's lower edge (TCP 13 mm above them)
    at, out = (BX + NECK_X, BY, z), (BX + NECK_X + 0.06, BY - 0.06, z)
    plan = []
    for k in range(STROKES):
        plan += [(3.0 if k == 0 else 1.0, out, down(-math.pi / 4), OPEN, "beside the cap"),
                 (1.2, at, down(-math.pi / 4), OPEN, "around the cap"), (0.8, at, down(-math.pi / 4), SHUT, "pinch"),
                 (1.25, at, down(math.pi / 4), SHUT, "turn"), (1.25, at, down(3 * math.pi / 4), SHUT, "turn")]
        if k < STROKES - 1:
            plan += [(0.6, at, down(3 * math.pi / 4), OPEN, "let go"), (1.0, out, down(3 * math.pi / 4), OPEN, "out"),
                     (0.8, out, down(math.pi / 4), OPEN, "turn back")]
    lift = (BX + NECK_X - 0.02, BY - 0.04, TOP + NECK_TOP + 0.03)
    drop = (0.18, -0.14, TOP + 0.08)
    return plan + [(1.2, lift, down(3 * math.pi / 4), SHUT, "lift the cap off"),
                   (2.0, drop, down(3 * math.pi / 4), SHUT, "carry it aside"),
                   (0.6, drop, down(3 * math.pi / 4), OPEN, "drop it"),
                   (1.0, (0.18, -0.14, TOP + 0.14), down(3 * math.pi / 4), OPEN, "clear")]


def plan(route: str = "unscrew") -> dict[str, list]:
    right = _unscrew() if route == "unscrew" else _flip()
    gripped = sum(w[0] for w in _left(0.0)[:3])
    return {"left": _left(sum(w[0] for w in right) - gripped), "right": right}


def demo():
    """The unscrew route (DEMO_ROUTE=flip plays the flip-top route): the left arm grips the bottle; the right
    arm opens it; the left arm lifts the bottle, tips it over the cup and holds it there."""
    from yam_arena.demo import Waypoint

    route = os.environ.get("DEMO_ROUTE", "unscrew")
    return {arm: [Waypoint(d, tuple(p), f, label, r) for d, p, r, f, label in wps] for arm, wps in plan(route).items()}
