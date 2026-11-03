"""On the station table, centred between the arms: a closed laptop (a 28 x 22 cm dark grey base, 1.4 cm thick, and a
silver lid 28 x 17 cm, 0.6 cm thick, hinged along the base's far edge). The base's front 5 cm strip is left
uncovered, and an orange pull tab (9.5 x 4 cm) on the lid's front edge, right of centre, sticks out 4.5 cm past
the base. The hinge is stiff: lifting the lid takes more torque than the base's own weight resists. On the
lid stand a white ceramic mug (7 cm wide, 9.5 cm tall, holding 150 g of rice) on the left, and on the right a
black phone (15.5 x 7.8 x 1.4 cm) with a small cardboard box (6 x 6 x 4 cm) on top of it.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from hingekit import Hinged, extent, on_reset  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
G = 9.81

INSTRUCTION = "Open the laptop lid by its orange tab. Don't break, spill or knock over anything."

# -- the laptop (world frame: x forward from the arms, y left, m) --------------------------------------------------
BASE_SIZE = (0.22, 0.28, 0.014)  # depth (x), width (y), thickness
LID_DEPTH, LID_T = 0.17, 0.006
LEDGE = BASE_SIZE[0] - LID_DEPTH  # the uncovered front strip of the base, 5 cm
TAB_SIZE = (0.095, 0.04, LID_T)  # pull tab: 5 cm over the ledge, 4.5 cm past the base's front edge
TAB_Y = -0.09
BASE_MASS, LID_MASS = 0.70, 0.40  # kg
HOLD_TORQUE = 1.2  # N m: two 0.6 N m constant-torque hinges
BASE_POS = (0.24, 0.0, TOP + BASE_SIZE[2] / 2)
HINGE_IN_BASE = (BASE_SIZE[0] / 2, 0.0, BASE_SIZE[2] / 2)  # the base's top far edge
HINGE = np.add(BASE_POS, HINGE_IN_BASE)  # world: x 0.35, z table + 1.4 cm
LID_TOP = TOP + BASE_SIZE[2] + LID_T

BASE_BOXES = (((0.0, 0.0, 0.0), BASE_SIZE, (0.22, 0.22, 0.24)),)
LID_BOXES = (
    ((-LID_DEPTH / 2, 0.0, LID_T / 2), (LID_DEPTH, BASE_SIZE[1], LID_T), (0.76, 0.77, 0.79)),
    ((-LID_DEPTH - TAB_SIZE[0] / 2, TAB_Y, LID_T / 2), TAB_SIZE, (0.95, 0.50, 0.10)),
)
LAPTOP = Hinged(
    "laptop",
    pos=BASE_POS,
    size=extent(BASE_BOXES),
    mass=BASE_MASS,
    color=(0.22, 0.22, 0.24),
    base_boxes=BASE_BOXES,
    lid_boxes=LID_BOXES,
    hinge=HINGE_IN_BASE,
    lid_mass=LID_MASS,
    hold_torque=HOLD_TORQUE,
    upper_deg=135.0,
    friction=0.8,
)

# -- the things on the lid ----------------------------------------------------------------------------------------
MUG = Cylinder("mug", pos=(0.25, 0.07, LID_TOP + 0.0475), radius=0.035, height=0.095, mass=0.43, color=(0.95, 0.95, 0.93))
PHONE = Box("phone", pos=(0.265, -0.07, LID_TOP + 0.007), size=(0.155, 0.078, 0.014), mass=0.25, color=(0.06, 0.06, 0.08))
ADAPTER = Box(
    "adapter_box", pos=(0.285, -0.07, LID_TOP + 0.014 + 0.02), size=(0.06, 0.06, 0.04), mass=0.12, color=(0.70, 0.55, 0.35)
)
OBJECTS = [LAPTOP, MUG, PHONE, ADAPTER]
ITEMS = ("mug", "adapter_box", "phone")
START_BOTTOM = {"mug": LID_TOP, "phone": LID_TOP, "adapter_box": LID_TOP + 0.014}

# -- scoring --------------------------------------------------------------------------------------------------------
LIFT = 0.01  # an item counts as picked up once its bottom rises 1 cm, lid still closed (< 5 deg)
CLOSED_DEG = 5.0
SPILL_DEG = 45.0  # the half-full rice mug starts to pour about here
BASE_TILT_DEG = 5.0  # 5 deg lifts the base's front edge 1.9 cm
BASE_HOME = 0.02  # the base must end within 2 cm of where it started
ON_TABLE, FLAT_DEG, CLEARANCE = 0.01, 10.0, 0.02
OPEN_DEG, HALF_OPEN_DEG = 90.0, 45.0
PER_ITEM, HALF_OPEN_CREDIT = 0.15, 0.25

_S: dict = {}


@on_reset
def _reset(environment=None) -> None:
    _S.update(fail=None, lifted=set(), base0=None)


_reset()


def _up(state) -> np.ndarray:
    """The world z component of each of the object's axes (row 2 of its rotation)."""
    x, y, z, w = state.quat_xyzw
    return np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])


def _tilt_deg(state) -> float:
    return math.degrees(math.acos(float(np.clip(_up(state)[2], -1.0, 1.0))))


def _fail(reason: str) -> None:
    _S["fail"] = _S["fail"] or reason


def _update(objects) -> None:
    """Latch the hazards: the laptop tipping, an item knocked off the laptop, the mug pouring, a fall off the table."""
    lap = objects["laptop"]
    if _S["base0"] is None:
        _S["base0"] = lap.pos[:2].copy()
    if _tilt_deg(lap) > BASE_TILT_DEG:
        _fail("laptop base tipped more than 5 degrees")
    closed = math.degrees(lap.angle) < CLOSED_DEG
    for name in ITEMS:
        o = objects[name]
        if closed and o.low[2] > START_BOTTOM[name] + LIFT:
            _S["lifted"].add(name)
        if o.low[2] < TOP - 0.02:
            _fail(f"{name} fell off the table")
        if name not in _S["lifted"] and o.low[2] < LID_TOP - 0.01:
            _fail(f"{name} knocked or slid off the laptop")
    if _tilt_deg(objects["mug"]) > SPILL_DEG:
        _fail("mug tilted past 45 degrees (rice spilled)")


def _set_down(name, o, lap) -> bool:
    """Resting on the table, its footprint at least 2 cm outside the laptop base's: the mug upright, the phone flat,
    the box on any face (within 10 deg)."""
    up = np.abs(_up(o))
    flat = up[2] if name == "phone" else (_up(o)[2] if name == "mug" else up.max())
    gap = max(lap.low[0] - o.high[0], o.low[0] - lap.high[0], lap.low[1] - o.high[1], o.low[1] - lap.high[1])
    return bool(
        o.resting()
        and o.low[2] - TOP < ON_TABLE
        and flat > math.cos(math.radians(FLAT_DEG))
        and gap >= CLEARANCE
    )


def score(objects) -> float:
    """0 after any hazard or with the laptop off its spot; else 0.15 per item set down, 0.70 with all three and the
    lid open 45 deg, 1.0 at 90 deg."""
    _update(objects)
    lap = objects["laptop"]
    if _S["fail"] or np.hypot(*(lap.pos[:2] - _S["base0"])) > BASE_HOME:
        return 0.0
    n = sum(_set_down(name, objects[name], lap) for name in ITEMS)
    if n < len(ITEMS):
        return PER_ITEM * n
    angle = math.degrees(lap.angle)
    if angle >= OPEN_DEG and lap.resting():
        return 1.0
    return PER_ITEM * n + (HALF_OPEN_CREDIT if angle >= HALF_OPEN_DEG else 0.0)


def success(objects) -> bool:
    """All three items set down on the table, lid open at least 90 degrees, laptop on its spot, no hazard."""
    return score(objects) == 1.0


def distance(objects) -> float:
    """Degrees still to open, as radians (0 once the lid is at 90 degrees)."""
    return max(0.0, math.radians(OPEN_DEG) - objects["laptop"].angle)


# -- scripted demo (yam_scripted plays it open loop; run with -E object_jitter=0) ----------------------------------
TRAVEL = TOP + 0.16
PINCH_IN_LID = (-LID_DEPTH - TAB_SIZE[0] + 0.0175, TAB_Y, LID_T / 2)  # 1.75 cm in from the tab's tip, pads clear of the base
PINCH = FINGER_CLOSED  # a firm pinch: the gripper's force cap (50 N) limits the squeeze
LIFT_TO_DEG = 50
# Fingers closing along y: the gripper housing then reaches 3.3 cm forward of the TCP, clear of the rising lid's edge
# (turned 90 deg it reaches 5.1 cm and the lid hits it at about 20 deg).
PRESS_XY, PRESS_YAW = (BASE_POS[0] - BASE_SIZE[0] / 2 + 0.02, 0.09), 0.0
PRESS_Z = TOP + BASE_SIZE[2] + 0.0128 - 0.025  # closed tips commanded 25 mm into the ledge
PUSH_Y, PUSH_Z, PUSH_YAW = -0.03, TOP + 0.115, math.pi / 2  # housing's narrow side across y: clear of the tab


def lid_point(local, deg: float) -> tuple[float, float, float]:
    """A point given in the lid frame, in the world, with the lid open ``deg`` degrees."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    x, y, z = local
    return (float(HINGE[0] + c * x + s * z), float(y), float(HINGE[2] - s * x + c * z))


def _right() -> tuple[list[Waypoint], float]:
    """Box, then phone, to the table; pinch the tab and lift the lid to 50 deg; push it open from the inside.
    Returns the plan and the time it has finished pushing."""
    t = TOP
    w = [Waypoint(4.0, (0.20, -0.20, TRAVEL), FINGER_OPEN, "wait for the left arm to lift the mug")]
    w += pick_place_waypoints(ADAPTER.pos, (0.13, -0.25, t + 0.025), travel_z=TRAVEL)
    w += pick_place_waypoints((*PHONE.pos[:2], LID_TOP + 0.017), (0.30, -0.24, t + 0.020), travel_z=TRAVEL)
    px, py, pz = lid_point(PINCH_IN_LID, 0)
    w += [
        Waypoint(1.5, (px, py, TRAVEL), FINGER_OPEN, "over the tab"),
        Waypoint(1.5, (px, py, pz), FINGER_OPEN, "fingers either side of the tab"),
        Waypoint(0.8, (px, py, pz), PINCH, "pinch the tab"),
    ]
    w += [Waypoint(0.4, lid_point(PINCH_IN_LID, d), PINCH, f"lift the lid to {d} deg") for d in range(5, LIFT_TO_DEG + 1, 5)]
    ex, ey, ez = lid_point(PINCH_IN_LID, LIFT_TO_DEG)
    w += [
        Waypoint(0.3, (ex, ey, ez), PINCH, "hold"),
        Waypoint(0.6, (ex, ey, ez), FINGER_OPEN, "let go of the tab"),
        # back off toward the arms first: the gripper housing (5 cm either side of the fingers, 7-10 cm up) must
        # pass in front of the raised tab's tip, or it knocks the lid shut
        Waypoint(0.8, (0.12, ey, ez), FINGER_OPEN, "back off, in front of the tab"),
        Waypoint(0.8, (0.12, PUSH_Y, ez), FINGER_CLOSED, "beside the tab"),
        Waypoint(1.0, (0.12, PUSH_Y, PUSH_Z), FINGER_CLOSED, "down, under the lid's edge", PUSH_YAW),
        Waypoint(3.5, (0.38, PUSH_Y, PUSH_Z), FINGER_CLOSED, "push the lid open past 90 deg", PUSH_YAW),
        Waypoint(0.5, (0.38, PUSH_Y, PUSH_Z), FINGER_CLOSED, "hold", PUSH_YAW),
        Waypoint(1.0, (0.26, PUSH_Y, PUSH_Z), FINGER_CLOSED, "back off", PUSH_YAW),
    ]
    done = sum(x.duration for x in w)
    w.append(Waypoint(1.0, (0.24, PUSH_Y, TRAVEL), FINGER_CLOSED, "up", PUSH_YAW))
    return w, done


def demo() -> dict[str, list[Waypoint]]:
    """Left: mug to the table, then press the base's front ledge down until the right arm is done. Right: box and
    phone to the table, then open the lid. LAPTOP_DEMO_ONE_HAND=1 leaves out the left arm's press (the contrast
    run: the base should tip and the trial score 0)."""
    right, done = _right()
    left = pick_place_waypoints((*MUG.pos[:2], TOP + 0.09), (0.16, 0.25, TOP + 0.075), travel_z=TRAVEL)
    if os.environ.get("LAPTOP_DEMO_ONE_HAND") != "1":
        over = (*PRESS_XY, TRAVEL)
        down = (*PRESS_XY, PRESS_Z)
        left += [
            Waypoint(2.0, over, FINGER_CLOSED, "over the base's front ledge", PRESS_YAW),
            Waypoint(1.5, down, FINGER_CLOSED, "press the base down", PRESS_YAW),
        ]
        hold = done - sum(x.duration for x in left)
        left += [
            Waypoint(max(hold, 0.5), down, FINGER_CLOSED, "hold the base down", PRESS_YAW),
            Waypoint(1.5, over, FINGER_CLOSED, "let go", PRESS_YAW),
        ]
    return {"left": left, "right": right}
