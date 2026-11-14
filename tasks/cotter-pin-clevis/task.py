"""On the station table, in front of the right arm: a clevis joint clamped on a steel stand. A dark fork clevis holds a
yellow rod end between its two horizontal prongs on an upright 8 mm steel clevis pin, with a washer on the top prong.
Through the cross-hole near the pin's top, pointing away from the arms, is a zinc-plated 2 mm cotter pin, pushed fully
home (eye against the pin) but with its two legs still straight and together.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

# Idea first proposed in robocurve/benchmark-tasks#994; independent implementation.

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from cotterkit import CotterRig, LegParams  # noqa: E402
from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Finish securing the clevis pin with its cotter pin, then move both arms clear."

# -- layout --------------------------------------------------------------------------------------------------------
# Rig frame: origin on the clevis pin's axis at the height of the cotter's axis, 10 cm above the table; x forward
# (away from the arms), y left, z up; metres. Table top at z = -0.100.
PIN_XY = (0.30, -0.08)
H = 0.100
FORK, PIN, WASHER, ROD, STAND = (0.20, 0.22, 0.26), (0.66, 0.67, 0.70), (0.55, 0.56, 0.58), (0.92, 0.72, 0.08), \
    (0.32, 0.33, 0.35)
COTTER = (0.80, 0.80, 0.76)


def _box(x0, x1, y0, y1, z0, z1, color):
    return (((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color)


def _cyl(d, z0, z1, color, x=0.0, y=0.0):
    return ((x, y, (z0 + z1) / 2), (d, d, z1 - z0), color, "cyl")


BOXES = (
    _cyl(0.008, -0.019, 0.003, PIN),  # ISO 2341-B 8 x 22 clevis pin, cross-hole 2 mm, 3 mm below its top
    _cyl(0.014, -0.022, -0.019, PIN),  # its head, under the bottom prong
    _cyl(0.016, -0.0030, -0.0014, WASHER),  # ISO 7089 8 mm washer on the top prong
    _box(-0.030, 0.0, -0.008, 0.008, -0.007, -0.003, FORK),  # top prong, 4 mm thick ...
    _cyl(0.016, -0.007, -0.003, FORK),  # ... with a rounded end round the pin
    _box(-0.030, 0.0, -0.008, 0.008, -0.019, -0.015, FORK),  # bottom prong
    _cyl(0.016, -0.019, -0.015, FORK),
    _box(-0.040, -0.030, -0.008, 0.008, -0.019, -0.003, FORK),  # fork base
    _box(-0.060, -0.040, -0.006, 0.006, -0.017, -0.005, FORK),  # fork shank (M8 rod end thread)
    _box(-0.075, -0.030, -0.020, 0.020, -H, -0.019, STAND),  # stand, clamped to the table
    _box(-0.008, 0.110, -0.006, 0.006, -0.0148, -0.0072, ROD),  # rod end between the prongs, 7.6 mm thick
    _box(0.095, 0.115, -0.010, 0.010, -H, -0.0148, STAND),  # support under the rod's far end
    _box(-0.0065, -0.0040, -0.0018, 0.0018, -0.0009, 0.0009, COTTER),  # the cotter's eye against the pin
)
RIG = CotterRig("cotter", pos=(*PIN_XY, TOP + H), size=(0.19, 0.04, H + 0.003), color=FORK, boxes=BOXES,
                root_x=0.004, long_len=0.021, short_len=0.017, leg_t=0.0009, leg_w=0.0018, leg_color=COTTER,
                params=LegParams())
OBJECTS = [RIG]

# -- scoring -------------------------------------------------------------------------------------------------------
BENT_DEG = 60.0  # each leg bent this far or more from straight (they can only open away from each other)
STARTED_DEG = 60.0
CLEAR_M = 0.02  # every finger 2 cm or more from the cotter
STILL_DPS = 5.0  # legs turning slower than this (deg/s)


def bent(s, leg: str) -> bool:
    return s.bend_deg[leg] >= BENT_DEG


def score(objects) -> float:
    """End of episode. 0 if a leg was ever re-bent (bent back 15 degrees or more after it had been set 30 degrees or
    more). Otherwise 0.75 both legs bent 60 degrees or more, every finger 2 cm or more from the cotter and the legs
    still (success, 1.0, once that has held 1 s); 0.5 both legs bent 60 degrees or more but touched or moving; 0.25
    one leg bent 60 degrees or more; else 0."""
    s = objects["cotter"]
    if any(s.rebent.values()):
        return 0.0
    n = sum(bent(s, leg) for leg in ("long", "short"))
    if n == 2:
        return 0.75 if s.gap_m >= CLEAR_M and s.leg_speed < STILL_DPS else 0.5
    return 0.25 if n == 1 else 0.0


def success(objects) -> bool:
    """Both legs bent 60 degrees or more, nothing re-bent, fingers clear, legs still; must hold 1 s."""
    return score(objects) == 0.75


# -- demo (yam_scripted plays it open loop) ------------------------------------------------------------------------
# The layout is fixed, so the demo is a fixed plan; it reads nothing from the simulator. The fingers straddle a leg,
# 4 mm apart, and the trailing finger's flat inner pad pushes it round its root. (A closed fingertip's outer edges
# slope, so pushed against a 1.8 mm wire it rides up and over it.) Each finger's tip is 4 mm thick and 15.4 mm wide.
HALF_W = 0.0077
GAP = 0.004  # fingers this far apart (finger joint target -GAP/2)
TIP_Z = -0.0045  # finger tip ends 4.5 mm below the cotter's axis, so the legs (1.8 mm tall) meet the flat pads
PAD_TIP = 0.013  # yam_arena.constants.PAD_TIP_PAST_TCP
HOVER = 0.06
ROOT = np.array([RIG.root_x, 0.0])
SWING_DEG = float(os.environ.get("DEMO_SWING_DEG", 100.0))  # commanded swing: past the 60 deg goal for arm lag
START_DEG = 70.0  # commanded push on the long leg's extension; the arm lags, and the leg ends about 18 deg out
OPENED_DEG = 18.0  # where the demo expects the long leg after that push
PINCH_GAP = 0.007  # fingers this far apart going down astride a leg to grip it (tolerates about +-10 deg)
PRESS = 0.0023  # straddle centre shifted this far toward the push: the trailing pad 0.8 mm into the leg


def _world(p_rig, z) -> tuple[float, float, float]:
    return (PIN_XY[0] + p_rig[0], PIN_XY[1] + p_rig[1], TOP + H + z + PAD_TIP)


def _rot(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


def push(label: str, start_rel, yaw0_deg: float, sweep_deg: float, seconds: float) -> list[Waypoint]:
    """Down astride a leg at ``start_rel`` (straddle centre relative to the leg root), then swing round the root by
    ``sweep_deg`` (counter-clockwise +), turning with the leg, the trailing pad pressed into it; then back up."""
    steps = max(2, int(abs(sweep_deg) / 5))
    f = -GAP / 2
    p0 = ROOT + start_rel
    start_rel = start_rel + np.array([0.0, math.copysign(PRESS, sweep_deg)])
    out = [
        Waypoint(2.0, _world(p0, HOVER), f, f"{label}: over the leg", math.radians(yaw0_deg)),
        Waypoint(1.5, _world(p0, TIP_Z), f, f"{label}: down astride the leg", math.radians(yaw0_deg)),
    ]
    for i in range(1, steps + 1):
        a = sweep_deg * i / steps
        p = ROOT + _rot(a) @ start_rel
        out.append(Waypoint(seconds / steps, _world(p, TIP_Z), f, f"{label}: push", math.radians(yaw0_deg + a)))
    out.append(Waypoint(0.5, out[-1].pos, f, f"{label}: hold", out[-1].yaw))
    a = sweep_deg - math.copysign(3.0, sweep_deg)  # ease off along the arc, then lift
    p = ROOT + _rot(a) @ (start_rel - np.array([0.0, math.copysign(PRESS, sweep_deg)]))
    out.append(Waypoint(0.8, _world(p, TIP_Z), f, f"{label}: ease off", math.radians(yaw0_deg + a)))
    out.append(Waypoint(1.2, _world(p, HOVER), f, f"{label}: up", math.radians(yaw0_deg + a)))
    return out


def pinch(label: str, leg_deg: float, sweep_deg: float, d: float = 0.013, seconds: float = 6.0) -> list[Waypoint]:
    """Down astride a leg lying at ``leg_deg`` (counter-clockwise +), the straddle centred ``d`` out from its root;
    close on it like pliers, swing it ``sweep_deg`` round its root (the gripper turning with it), open and lift.
    Gripped, the leg's bending moment is taken mostly as a twist at the wrist, which lags far less than a push."""
    steps = max(2, int(abs(sweep_deg) / 5))
    f = -PINCH_GAP / 2

    def at(a):
        return _world(ROOT + _rot(a) @ np.array([d, 0.0]), TIP_Z)

    up = _world(ROOT + _rot(leg_deg) @ np.array([d, 0.0]), HOVER)
    out = [
        Waypoint(2.0, up, f, f"{label}: over the leg", math.radians(leg_deg)),
        Waypoint(1.5, at(leg_deg), f, f"{label}: down astride the leg", math.radians(leg_deg)),
        Waypoint(0.8, at(leg_deg), FINGER_CLOSED, f"{label}: grip", math.radians(leg_deg)),
    ]
    for i in range(1, steps + 1):
        a = leg_deg + sweep_deg * i / steps
        out.append(Waypoint(seconds / steps, at(a), FINGER_CLOSED, f"{label}: bend", math.radians(a)))
    end = leg_deg + sweep_deg
    out += [
        Waypoint(0.5, at(end), FINGER_CLOSED, f"{label}: hold", math.radians(end)),
        Waypoint(0.6, at(end), f, f"{label}: let go", math.radians(end)),
        Waypoint(1.2, _world(ROOT + _rot(end) @ np.array([d, 0.0]), HOVER), f, f"{label}: up", math.radians(end)),
    ]
    return out


def demo():
    """Right arm. The legs start together, so first the fingers go down astride the long leg's 4 mm extension, just
    past the short leg's end, and push it out about 15 degrees toward +y. Now it is clear of the short leg: the
    fingers grip it along its length like pliers and swing it round its root to 100 degrees. Then they grip the short
    leg, which the long leg has left room beside, and swing it to 100 degrees toward -y. Then the arm moves clear.
    DEMO_LONG_ONLY=1 bends only the long leg (scores 0.25); DEMO_REBEND=1 grips the long leg again afterwards,
    swings it back 40 degrees and out again, then does the short leg (scores 0; needs about 1,150 steps)."""
    t = RIG.leg_t / 2
    long_start = np.array([RIG.short_len + 0.0006 + HALF_W, t])  # pads over x 21.6..37 mm in rig
    plan = push("long leg: start", long_start, 0.0, START_DEG, 4.2)
    plan += pinch("long leg", OPENED_DEG, SWING_DEG - OPENED_DEG)
    if os.environ.get("DEMO_REBEND"):  # the long leg ends about 73 deg out (arm lag): straighten it, bend it again
        plan += pinch("straighten", 73.0, -40.0, seconds=3.0)
        plan += pinch("bend again", 36.0, 64.0, seconds=3.0)
    if not os.environ.get("DEMO_LONG_ONLY"):
        plan += pinch("short leg", 0.0, -SWING_DEG)
    plan.append(Waypoint(2.0, _world(ROOT + (0.0, -0.10), 0.12), -GAP / 2, "clear"))
    return {"right": plan}
