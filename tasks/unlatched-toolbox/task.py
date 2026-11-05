"""On the station table, a little left of centre: a blue plastic toolbox (22 cm front to back, 12 cm wide, 9.5 cm tall
without its handle): a shallow 2.5 cm tray under a deep 7 cm lid. The lid is hinged along the back edge and carries
the only handle, a black bar running front to back on top. The lid rests shut, but both orange latch levers on the
front face (facing the arms) lean out open: the lid is not fastened. Inside are four tools (a wrench, a tape
measure, a tin of screws, pliers). To the right, a fixed wooden shelf 8 cm high with a 1.2 cm lip round its edge.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from boxkit import LatchParams, ToolBox, on_reset, rot_y  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Put the toolbox on the shelf."

BLUE, DARK, ORANGE, STEEL = (0.10, 0.30, 0.70), (0.06, 0.06, 0.07), (1.0, 0.45, 0.0), (0.62, 0.63, 0.66)
WOOD, WOOD_DARK = (0.72, 0.55, 0.36), (0.55, 0.40, 0.25)

# -- the toolbox (base frame: origin on the table under the base centre, x forward, y left; metres) ---------------
L, W, H, LH, T = 0.22, 0.12, 0.025, 0.07, 0.006  # length (x), width (y), base (tray) height, lid height, wall
BASE = (
    ((0, 0, T / 2), (L, W, T), BLUE),  # floor
    ((-(L - T) / 2, 0, (H + T) / 2), (T, W, H - T), BLUE),  # front wall
    (((L - T) / 2, 0, (H + T) / 2), (T, W, H - T), BLUE),  # back wall
    ((0, (W - T) / 2, (H + T) / 2), (L - 2 * T, T, H - T), BLUE),  # side walls
    ((0, -(W - T) / 2, (H + T) / 2), (L - 2 * T, T, H - T), BLUE),
) + tuple(((-L / 2 - 0.003, sy * 0.035, H - 0.015), (0.006, 0.03, 0.008), STEEL) for sy in (-1, 1))  # latch brackets
LID_AT = (L / 2, 0.0, H)  # hinge axis: the base's back top edge
HANDLE_Z = LH + 0.034  # bar centre above the hinge axis (lid frame)
LID_BOXES = (
    ((-L / 2, 0, LH - T / 2), (L, W, T), BLUE),  # top
    ((-L + T / 2, 0, (LH - T) / 2), (T, W, LH - T), BLUE),  # front
    ((-T / 2, 0, (LH - T) / 2), (T, W, LH - T), BLUE),  # back
    ((-L / 2, (W - T) / 2, (LH - T) / 2), (L - 2 * T, T, LH - T), BLUE),  # sides
    ((-L / 2, -(W - T) / 2, (LH - T) / 2), (L - 2 * T, T, LH - T), BLUE),
    ((-L / 2 - 0.045, 0, LH + 0.011), (0.012, 0.016, 0.022), DARK),  # handle posts
    ((-L / 2 + 0.045, 0, LH + 0.011), (0.012, 0.016, 0.022), DARK),
    ((-L / 2, 0, HANDLE_Z), (0.11, 0.016, 0.024), DARK),  # handle bar, 2.4 cm deep, 2.2 cm above the lid
) + tuple(((-L - 0.002, sy * 0.035, 0.012), (0.004, 0.024, 0.010), STEEL) for sy in (-1, 1))  # keepers
LATCH_AT = tuple((-L / 2 - 0.004, sy * 0.035, H - 0.015) for sy in (1, -1))  # latch_l (+y), latch_r (-y) pivots
LATCH = (
    ((0, 0, 0.021), (0.004, 0.022, 0.042), ORANGE),  # lever plate: from its pivot to 4.2 cm above it
    ((-0.004, 0, 0.040), (0.008, 0.022, 0.004), ORANGE),  # finger tab at the top, pointing out
)
PARAMS = LatchParams()
BX, BY = 0.30, 0.10
TOOLBOX = ToolBox("toolbox", pos=(BX, BY, TOP + 0.0005), size=(L, W, H + LH + 0.046), color=BLUE, mass=0.12,
                  base_boxes=BASE, lid_at=LID_AT, lid_boxes=LID_BOXES, lid_mass=0.20, latch_at=LATCH_AT,
                  latch_boxes=LATCH, latch_mass=0.01, friction=0.8, params=PARAMS)

# -- the tools, lying on the base's floor (base frame x, y; world z) -----------------------------------------------
TOOLS = [
    Box("wrench", pos=(BX, BY - 0.035, TOP + T + 0.003), size=(0.16, 0.025, 0.006), color=STEEL, mass=0.08),
    Cylinder("tape", pos=(BX - 0.065, BY + 0.022, TOP + T + 0.0175), radius=0.028, height=0.035,
             color=(0.95, 0.80, 0.10), mass=0.09),
    Cylinder("tin", pos=(BX + 0.065, BY + 0.025, TOP + T + 0.0225), radius=0.022, height=0.045,
             color=(0.15, 0.45, 0.25), mass=0.07),
    Box("pliers", pos=(BX, BY + 0.025, TOP + T + 0.0075), size=(0.05, 0.03, 0.015), color=(0.80, 0.10, 0.10),
        mass=0.07),
]
TOOL_NAMES = [t.name for t in TOOLS]

# -- the shelf: fixed, 8 cm high, a 1.2 cm lip round its top --------------------------------------------------------
SX, SY, S_LX, S_LY, S_H, LIP, LIP_W = 0.30, -0.13, 0.28, 0.20, 0.08, 0.012, 0.01
SHELF = [
    Box("shelf", pos=(SX, SY, TOP + S_H / 2), size=(S_LX, S_LY, S_H), color=WOOD),
    Box("lip_front", pos=(SX - (S_LX - LIP_W) / 2, SY, TOP + S_H + LIP / 2), size=(LIP_W, S_LY, LIP), color=WOOD_DARK),
    Box("lip_back", pos=(SX + (S_LX - LIP_W) / 2, SY, TOP + S_H + LIP / 2), size=(LIP_W, S_LY, LIP), color=WOOD_DARK),
    Box("lip_left", pos=(SX, SY + (S_LY - LIP_W) / 2, TOP + S_H + LIP / 2), size=(S_LX - 2 * LIP_W, LIP_W, LIP),
        color=WOOD_DARK),
    Box("lip_right", pos=(SX, SY - (S_LY - LIP_W) / 2, TOP + S_H + LIP / 2), size=(S_LX - 2 * LIP_W, LIP_W, LIP),
        color=WOOD_DARK),
]
OBJECTS = [TOOLBOX, *TOOLS, *SHELF]

# -- scoring --------------------------------------------------------------------------------------------------------
INSIDE = (L / 2 - T, W / 2 - T, H + LH - T)  # the box's inside: |x|, |y| below these, 0 < z < the lid's underside
SHELF_TOP = TOP + S_H
SHELF_IN = ((S_LX - 2 * LIP_W) / 2, (S_LY - 2 * LIP_W) / 2)  # inside the lip
LID_SHUT_DEG = 5.0    # lid within this of shut for success
LID_OPEN_DEG = 30.0   # lid opened this far while the box is off the table and the shelf: 0
TIP_DEG, FALL, DROP_V = 45.0, 0.05, 1.0  # tipped over; fell off the table; dropped (falling faster than 1 m/s)
LIFT = 0.05            # lifted: base 5 cm or more above the table
REST_TILT = 10.0

_STATE: dict = {}
_DEBUG = [0] if os.environ.get("BOX_DEBUG") else []


def _reset_state() -> None:
    _STATE.update(failed="", lifted=False)


_reset_state()
on_reset(_reset_state)


def tool_inside(box, tool) -> bool:
    p = box.to_local(tool.pos)
    return bool(abs(p[0]) < INSIDE[0] and abs(p[1]) < INSIDE[1] and 0.0 < p[2] < INSIDE[2])


def on_shelf(box) -> bool:
    """Base bottom within 1 cm of the shelf top and its centre inside the lip."""
    return bool(abs(box.pos[2] - SHELF_TOP) < 0.01 and abs(box.pos[0] - SX) < SHELF_IN[0]
                and abs(box.pos[1] - SY) < SHELF_IN[1])


def _update(objects) -> float:
    box = objects["toolbox"]
    supported = box.pos[2] < TOP + 0.01 or on_shelf(box)
    checks = [("tool out", not all(tool_inside(box, objects[n]) for n in TOOL_NAMES)),
              ("tipped", box.tilt_deg() > TIP_DEG), ("off the table", box.pos[2] < TOP - FALL),
              ("dropped", box.lin_vel[2] < -DROP_V), ("lid opened in the air", box.lid_deg > LID_OPEN_DEG and not supported)]
    for reason, bad in checks:
        if bad and not _STATE["failed"]:
            _STATE["failed"] = reason
    if _DEBUG:
        _DEBUG[0] += 1
        if _DEBUG[0] % 20 == 0:
            out = [n for n in TOOL_NAMES if not tool_inside(box, objects[n])]
            print(f"[task] {_DEBUG[0]} failed {_STATE['failed']!r} tools out {out} "
                  f"z {[round(float(objects[n].pos[2] - TOP), 3) for n in out]}", flush=True)
    if _STATE["failed"]:
        return 0.0
    both = all(box.engaged)
    _STATE["lifted"] |= both and box.pos[2] >= TOP + LIFT
    if on_shelf(box) and box.lid_deg < LID_SHUT_DEG and box.tilt_deg() < REST_TILT and box.resting(0.02, 0.2):
        return 1.0
    if both and _STATE["lifted"]:
        return 0.75
    return 0.5 if both else 0.25 if any(box.engaged) else 0.0


def success(objects) -> bool:
    """On the shelf (inside the lip, upright within 10 degrees, still), lid shut, every tool inside (latched or not)."""
    return _update(objects) == 1.0


def score(objects) -> float:
    """0 if a tool ever left the box, the box tipped past 45 degrees, fell off the table, dropped (faster than 1 m/s)
    or its lid opened past 30 degrees while off the table and shelf. Otherwise 0.25 one latch closed; 0.5 both;
    0.75 both and the box was lifted 5 cm with both closed; 1.0 at rest on the shelf, upright, lid shut, every tool inside
    (latched or not)."""
    return _update(objects)


# -- scripted demo ------------------------------------------------------------------------------------------------
def frame(z, x) -> np.ndarray:
    """Gripper rotation with approach axis ``z`` and finger-closing axis ``x`` (world)."""
    z = np.asarray(z, float) / np.linalg.norm(z)
    x = np.asarray(x, float) - z * np.dot(x, z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


DOWN = frame((0, 0, -1), (0, -1, 0))  # straight down, fingers closing along y
PITCH = math.radians(-50)  # handle grip: tilted 50 degrees forward (for reach), fingers across the bar (y)
GRIP = rot_y(PITCH) @ DOWN
APPROACH = GRIP[:, 2]
HANDLE = np.array([BX + LID_AT[0] - L / 2, BY, TOP + H + HANDLE_Z])
PINCH = -(0.008 - 0.005)             # finger target 5 mm inside the 1.6 cm bar (about 25 N per finger)
PUSH_Z = TOP + 0.038                 # TCP height pushing a latch: pad tips 2.5 cm up, 1.5 cm above its pivot
PUSH_FROM, PUSH_TO = BX - L / 2 - 0.06, BX - L / 2 + 0.008  # the low arm lags ~3.5 cm here
CARRY_Z = SHELF_TOP + LIP + 0.05 + H + HANDLE_Z  # base bottom 5 cm above the lip (the loaded arm sags ~3 cm)
PLACE_Z = SHELF_TOP + 0.002 + H + HANDLE_Z


def _left_latches():
    plan = []
    # the left arm lags more at the +y latch: it gets a deeper commanded push
    for y, to in ((BY + 0.035, PUSH_TO + 0.012), (BY - 0.035, PUSH_TO)):
        plan += [(1.5, (PUSH_FROM, y, TOP + 0.12), DOWN, FINGER_CLOSED, "in front of a latch"),
                 (1.0, (PUSH_FROM, y, PUSH_Z), DOWN, FINGER_CLOSED, "down"),
                 (1.5, (to, y, PUSH_Z), DOWN, FINGER_CLOSED, "push the latch shut"),
                 (0.5, (to, y, PUSH_Z), DOWN, FINGER_CLOSED, "hold"),
                 (0.8, (PUSH_FROM, y, PUSH_Z), DOWN, FINGER_CLOSED, "back off")]
    return plan + [(1.5, (PUSH_FROM - 0.05, BY + 0.10, TOP + 0.15), DOWN, FINGER_CLOSED, "clear")]


PARK = (0.18, -0.20, TOP + 0.18)


def _right_carry(wait: float):
    pre = HANDLE - 0.06 * APPROACH
    lift = (HANDLE[0], HANDLE[1], CARRY_Z)
    over = (SX, SY, CARRY_Z)
    place = (SX, SY, PLACE_Z)
    away = np.array(place) - 0.08 * APPROACH
    return [(2.0, PARK, DOWN, FINGER_OPEN, "wait clear of the left arm"), (wait, PARK, DOWN, FINGER_OPEN, "wait"),
            (2.5, tuple(pre), GRIP, FINGER_OPEN, "above the handle"),
            (1.5, tuple(HANDLE), GRIP, FINGER_OPEN, "around the handle"),
            (0.8, tuple(HANDLE), GRIP, PINCH, "grip the handle"),
            (3.0, lift, GRIP, PINCH, "lift"),
            (4.0, over, GRIP, PINCH, "carry over the shelf"),
            (1.0, over, GRIP, PINCH, "let it settle"),
            (2.5, place, GRIP, PINCH, "set it down"),
            (0.6, place, GRIP, FINGER_OPEN, "let go"),
            (1.2, tuple(away), GRIP, FINGER_OPEN, "away")]


def plan(route: str = "demo") -> dict[str, list]:
    if route == "naive":
        return {"left": [(2.0, (0.15, 0.25, TOP + 0.15), DOWN, FINGER_OPEN, "wait")], "right": _right_carry(0.1)}
    left = _left_latches()
    return {"left": left, "right": _right_carry(sum(w[0] for w in left) - 1.0)}


def demo():
    """Left arm pushes each orange latch shut with its closed fingers (they snap closed and lock the lid). Right arm
    then grips the handle, lifts the box, carries it over the shelf, sets it down inside the lip and lets go.
    DEMO_NAIVE=1 skips the latches: the right arm lifts by the handle at once, the tray swings down off the lid's hinge,
    tools fall onto the table and the trial scores 0."""
    from yam_arena.demo import Waypoint

    route = "naive" if os.environ.get("DEMO_NAIVE") else "demo"
    return {arm: [Waypoint(d, tuple(p), f, label, r) for d, p, r, f, label in wps] for arm, wps in plan(route).items()}
