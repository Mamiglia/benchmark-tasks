"""On the station table, in front of the arms: a sealed aluminium tube 1 m long and 5 cm across lies on the table,
left to right (along y), its middle at x 18, y -8 cm. Its left end has a red plug, its right end a blue plug. A steel
slug is glued inside at an unknown place, so the tube's balance point is not at its middle. Beyond it, a fixed
hanging station stands in for one hook of an overhead coating-line conveyor: a post at x 48, y -5 cm carries an arm
reaching toward the arms at 22 cm height, and from its end a J-hook hangs down. The hook's cradle, at x 33, y -5 cm,
is only 4 cm wide (along y) and lined with black rubber; it opens toward the arms, its lip 10 cm above the table.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from tubekit import TubeRig, surface_gap  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 2.0
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Hang the tube level on the hook."

# -- layout (env frame: x forward, y left, z up; m) ------------------------------------------------------------------
L, R = 1.0, 0.025  # tube: 1 m long, 50 mm outside diameter
TUBE_XY = (0.18, -0.08)
HOOK_XY = (0.33, -0.05)  # centre of the hook's cradle
CRADLE_LEN = 0.04  # the hook's cradle: 4 cm wide (along y)
REST_AXIS = 0.110  # tube axis 11 cm above the table when it hangs in the cradle
STEEL_GREY, RUBBER = (0.28, 0.29, 0.31), (0.07, 0.07, 0.07)
_hx, _hy = HOOK_XY
# The cradle is a rubber-lined 90-degree V: two faces at 45 degrees, each touching the seated tube at R sin 45 to either
# side of its axis, R cos 45 below it. Each face is a 1 cm thick rubber plate 3 cm across (turned about y).
_s = R * math.sqrt(0.5)
FACE_TOP = REST_AXIS - _s + 0.015 * math.sqrt(0.5)  # height of the faces' upper edges: 10.4 cm (the lip)
_back = _hx + _s + 0.015 * math.sqrt(0.5) + 0.017  # the shank rises here, 2 cm behind a seated tube
ARM_Z = 0.22  # underside of the overhead arm
POST_X = _hx + 0.15
_floor_top = REST_AXIS - _s - 0.015 * math.sqrt(0.5) - 0.002


def _face(side: int):
    """A V face (side -1: the front face, the lip; +1: the back face) as (centre, size, colour, rotation about y)."""
    nx, nz = -side * math.sqrt(0.5), math.sqrt(0.5)  # unit normal, toward the tube's axis
    px, pz = _hx + side * _s, REST_AXIS - _s  # contact point
    return ((px - 0.005 * nx, _hy, TOP + pz - 0.005 * nz), (0.03, CRADLE_LEN, 0.01), RUBBER, -side * 45.0)


HOOK_BOXES = (
    ((POST_X, _hy, TOP + 0.006), (0.12, 0.12, 0.012), STEEL_GREY),  # base plate, clamped to the table
    ((POST_X, _hy, TOP + (ARM_Z + 0.02) / 2), (0.03, 0.03, ARM_Z + 0.02), STEEL_GREY),  # post
    (((_back + POST_X) / 2, _hy, TOP + ARM_Z + 0.01), (POST_X - _back, 0.02, 0.02), STEEL_GREY),  # overhead arm
    ((_back + 0.005, _hy, TOP + (_floor_top - 0.015 + ARM_Z) / 2), (0.01, 0.01, ARM_Z - _floor_top + 0.015),
     STEEL_GREY),  # shank
    (((_hx - 0.03 + _back + 0.01) / 2, _hy, TOP + _floor_top - 0.0075), (_back + 0.01 - _hx + 0.03, CRADLE_LEN, 0.015),
     STEEL_GREY),  # the hook's bottom bend, under the V
    _face(-1),  # front face: the lip
    _face(+1),  # back face
)
RIG = TubeRig("tube_rig", pos=(*TUBE_XY, TOP + R + 0.0005), size=(0.1, L, 2 * R), length=L, radius=R,
              hook_boxes=HOOK_BOXES, table_z=TOP)
OBJECTS = [RIG]

# -- scoring ---------------------------------------------------------------------------------------------------------
LEVEL_DEG = 3.0  # tube axis within 3 degrees of horizontal
CLEAR_M = 0.05  # every finger pad and tool point at least 5 cm from the tube's surface
OPEN_FRAC = 0.8  # both grippers at least 80% open
STILL_V, STILL_W = 0.01, 0.05  # m/s, rad/s
ON_HOOK_XY, ON_HOOK_Z = 0.015, 0.01  # tube axis within 1.5 cm (sideways) and 1 cm (height) of seated in the cradle
TOUCH_N = 0.05  # N


def on_hook(t) -> bool:
    """The tube hangs in the hook's cradle: the hook carries it, its axis passes over the cradle within 1.5 cm at the
    seated height (within 1 cm), the hook is between its plugs, and its lowest point is clear of the table."""
    if t.hook_n <= TOUCH_N or t.lowest <= TOP + 0.002:
        return False
    k = (_hy - t.pos[1]) / t.axis[1] if abs(t.axis[1]) > 1e-6 else 0.0  # along the axis to the hook's y
    if abs(k) > L / 2 - 0.03:
        return False
    p = t.pos + k * t.axis
    return abs(p[0] - _hx) <= ON_HOOK_XY and abs(p[2] - (TOP + REST_AXIS)) <= ON_HOOK_Z


def released(t) -> bool:
    """Nothing of the robot touches the tube, both grippers are open and every pad is 5 cm or more from it."""
    gap = min(surface_gap(p, t.pos, t.axis, L, R) for pads in t.pads.values() for p in pads)
    return t.robot_n <= TOUCH_N and min(t.open_frac.values()) >= OPEN_FRAC and gap >= CLEAR_M


def still(t) -> bool:
    return float(np.linalg.norm(t.lin_vel)) < STILL_V and float(np.linalg.norm(t.ang_vel)) < STILL_W


def score(objects) -> float:
    """0 if the tube, after being lifted clear (lowest point 5 cm above the table, or the hook carrying it), ever
    touched the table or floor again, or was dropped (fell 2 cm with nothing touching it). Otherwise 0.75 hanging in
    the hook, level within 3 degrees, still, released (both grippers open, 5 cm clear, no robot contact); success
    (1.0) once that has held 2 s. 0.5 on the hook and level but held, touched or moving; 0.25 lifted clear; else 0."""
    t = objects["tube_rig"]
    if t.regrounded or t.dropped:
        return 0.0
    if on_hook(t) and t.tilt_deg < LEVEL_DEG:
        return 0.75 if released(t) and still(t) else 0.5
    return 0.25 if t.lifted else 0.0


def success(objects) -> bool:
    """Hanging level on the hook, still and released; must hold 2 s (SUCCESS_HOLD_S)."""
    return score(objects) == 0.75


# -- scripted demo (closed loop; see tubedemo.py) --------------------------------------------------------------------


def controller(bases, dt):
    """yam_scripted plays this: it sees the top camera image and the joint positions only."""
    import types

    import tubedemo

    return tubedemo.Controller(bases, dt, types.SimpleNamespace(R=R, REST_AXIS=REST_AXIS, HOOK_XY=HOOK_XY))
