"""On the station table: a compact white microwave oven at the far side, its cavity facing the arms (front at
x 31 cm, cavity floor 3 cm above the table), with its black door hinged on the left side and standing open about 95
degrees. In front of it on the right: a square white bowl of orange soup with a solid upright handle pointing
toward the arms and a steel spoon resting in it, its handle over the right rim; nearer the right arm, an empty
light-blue utensil tray. The robot must put the soup in the microwave and close the door.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from jointkit import Hinge, PartState, matrix_to_quat, on_reset, on_step, part, rot_y, rot_z, set_hinge_target  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Put the soup in the microwave and close the door."

WHITE, CAVITY, BLACK, GLASS = (0.90, 0.90, 0.88), (0.80, 0.80, 0.78), (0.10, 0.10, 0.11), (0.05, 0.06, 0.08)
DISPLAY, HANDLE = (0.20, 0.75, 0.60), (0.70, 0.70, 0.72)
CERAMIC, SOUP, STEEL, TRAY_BLUE = (0.96, 0.96, 0.94), (0.88, 0.38, 0.08), (0.82, 0.84, 0.87), (0.55, 0.72, 0.88)


def _box(lo, hi, color, yaw=None):
    """A box spec from its two corners."""
    c = tuple((a + b) / 2 for a, b in zip(lo, hi))
    s = tuple(b - a for a, b in zip(lo, hi))
    return (c, s, color) if yaw is None else (c, s, color, yaw)


# -- the microwave (origin: the middle of the cavity's front edge, on the table) --------------------------------------
MX, MY = 0.31, 0.0                    # front face at x 31 cm, cavity centred on y 0
D, W, H = 0.22, 0.25, 0.15            # cavity 22 deep, 25 wide, 15 tall
T, FLOOR, PANEL = 0.015, 0.03, 0.06   # 1.5 cm walls; cavity floor 3 cm above the table; 6 cm control panel
TOP_Z = FLOOR + H + T                 # 19.5 cm tall
MICRO_BOXES = [
    _box((0, -W / 2, 0), (D, W / 2, FLOOR), CAVITY),                                 # base; its top is the floor
    _box((0, W / 2, 0), (D + T, W / 2 + T, TOP_Z), WHITE),                           # left wall
    _box((0, -W / 2 - T, 0), (D + T, -W / 2, TOP_Z), WHITE),                         # right wall
    _box((D, -W / 2, FLOOR), (D + T, W / 2, FLOOR + H), CAVITY),                     # back wall
    _box((0, -W / 2 - T, FLOOR + H), (D + T, W / 2 + T, TOP_Z), WHITE),              # roof
    _box((0, -W / 2 - T - PANEL, 0), (D + T, -W / 2 - T, TOP_Z), BLACK),             # control panel
    _box((-0.002, -W / 2 - T - 0.05, 0.14), (0, -W / 2 - T - 0.01, 0.16), DISPLAY),  # its display
]
# The door: 28 cm wide (cavity plus both side walls), on a vertical hinge at the front-left corner. Its geometry is
# drawn open by DOOR0; the hinge angle runs from 0 (open DOOR0) to DOOR0 (shut against the front).
DOOR0 = math.radians(95)
DT = 0.015
DOOR_LEN = W + 2 * T
_DOOR_CLOSED = [  # child frame, door shut: x out of the front, y from the hinge toward the panel
    _box((-DT / 2, -DOOR_LEN, 0.003), (DT / 2, 0, TOP_Z), BLACK),
    _box((-DT / 2 - 0.002, -DOOR_LEN + 0.05, FLOOR + 0.02), (-DT / 2, -0.03, FLOOR + H - 0.01), GLASS),  # window
    _box((-DT / 2 - 0.022, -DOOR_LEN + 0.012, 0.06), (-DT / 2, -DOOR_LEN + 0.024, 0.15), HANDLE),       # handle
]


def _opened(spec, angle):
    """A closed-door box turned ``angle`` clockwise (seen from above) about the hinge."""
    c, s, color = spec
    x, y, z = c
    ca, sa = math.cos(angle), math.sin(angle)
    return ((x * ca + y * sa, -x * sa + y * ca, z), s, color, -math.degrees(angle))


DOOR = Hinge("door_hinge", anchor=(-DT / 2 - 0.001, W / 2 + T, 0.0), pitch=-math.pi / 2,  # axis up: + closes it
             boxes=tuple(_opened(b, DOOR0) for b in _DOOR_CLOSED), lower=-math.radians(5), upper=DOOR0,
             stiffness=2.0, damping=0.3, mass=0.4)
# 8 kg, about a compact microwave: too heavy for the arms to shove or lift
MICROWAVE = part("microwave", pos=(MX, MY, TOP), boxes=MICRO_BOXES, mass=8.0, hinge=DOOR, color=WHITE, friction=1.0)

# -- the bowl (origin: centre of its base) ----------------------------------------------------------------------------
BX, BY = 0.18, -0.07
B, BW, BH, SOUP_Z = 0.0475, 0.005, 0.055, 0.030   # 9.5 cm square, 5 mm walls, 5.5 cm tall, soup 2.5 cm deep
TAB_L, TAB_T, TAB_Z0, TAB_Z1 = 0.03, 0.012, 0.015, 0.050  # upright handle toward the arms: 3 cm out, 1.2 cm thick
BOWL_BOXES = [
    _box((-B, -B, 0), (B, B, BW), CERAMIC),
    _box((-B, -B, BW), (-B + BW, B, BH), CERAMIC), _box((B - BW, -B, BW), (B, B, BH), CERAMIC),
    _box((-B + BW, -B, BW), (B - BW, -B + BW, BH), CERAMIC), _box((-B + BW, B - BW, BW), (B - BW, B, BH), CERAMIC),
    _box((-B + BW, -B + BW, BW), (B - BW, B - BW, SOUP_Z), SOUP),                     # the soup (solid in sim)
    _box((-B - TAB_L, -TAB_T / 2, TAB_Z0), (-B, TAB_T / 2, TAB_Z1), CERAMIC),        # the handle
]
BOWL = part("bowl", pos=(BX, BY, TOP), boxes=BOWL_BOXES, mass=0.25, color=CERAMIC, friction=1.0)

# -- the spoon: a straight 15 cm steel bar, head toward +y on the soup, handle out over the bowl's -y rim -----------
SP_L, SP_T, HEAD_L, HEAD_W, HANDLE_W = 0.15, 0.004, 0.045, 0.032, 0.012
SPOON_BOXES = [_box((SP_L / 2 - HEAD_L, -HEAD_W / 2, -SP_T / 2), (SP_L / 2, HEAD_W / 2, SP_T / 2), STEEL),
               _box((-SP_L / 2, -HANDLE_W / 2, -SP_T / 2), (SP_L / 2 - HEAD_L, HANDLE_W / 2, SP_T / 2), STEEL)]
_RIM = np.array([BX, BY - (B - BW), TOP + BH])                  # the rim's inner top edge the spoon lies on
_DROP, _RUN = BH - SOUP_Z, 2 * (B - BW) - 0.004                 # rim to soup; rim to the head's end (4 mm short)
SP_TILT = math.atan2(_DROP, _RUN)                               # 17 degrees
_U = np.array([0.0, math.cos(SP_TILT), -math.sin(SP_TILT)])     # along the spoon, toward the head
_N = np.array([0.0, math.sin(SP_TILT), math.cos(SP_TILT)])      # its upper face's normal
SP_POS = _RIM + (math.hypot(_DROP, _RUN) - SP_L / 2) * _U + (SP_T / 2 + 0.001) * _N
SP_ROT = rot_z(math.pi / 2) @ rot_y(SP_TILT)
SPOON = part("spoon", pos=tuple(SP_POS), boxes=SPOON_BOXES, mass=0.035, color=STEEL, friction=0.6,
             rot=matrix_to_quat(SP_ROT), metallic=0.5, ccd=True)

# -- the utensil tray (origin: centre of its base) ------------------------------------------------------------------
TX, TY, TRAY_HX, TRAY_HY, TRAY_H, TRAY_W = 0.12, -0.26, 0.06, 0.09, 0.02, 0.004
TRAY = part("tray", pos=(TX, TY, TOP), mass=0.2, color=TRAY_BLUE, friction=0.8, boxes=[
    _box((-TRAY_HX, -TRAY_HY, 0), (TRAY_HX, TRAY_HY, TRAY_W), TRAY_BLUE),
    _box((-TRAY_HX, -TRAY_HY, TRAY_W), (-TRAY_HX + TRAY_W, TRAY_HY, TRAY_H), TRAY_BLUE),
    _box((TRAY_HX - TRAY_W, -TRAY_HY, TRAY_W), (TRAY_HX, TRAY_HY, TRAY_H), TRAY_BLUE),
    _box((-TRAY_HX + TRAY_W, -TRAY_HY, TRAY_W), (TRAY_HX - TRAY_W, -TRAY_HY + TRAY_W, TRAY_H), TRAY_BLUE),
    _box((-TRAY_HX + TRAY_W, TRAY_HY - TRAY_W, TRAY_W), (TRAY_HX - TRAY_W, TRAY_HY, TRAY_H), TRAY_BLUE)])
OBJECTS = [MICROWAVE, BOWL, SPOON, TRAY]

# -- the door's latch ----------------------------------------------------------------------------------------------
# Door opening angle = DOOR0 - hinge. Above CATCH the door swings freely (the drive's target follows it, so only its
# damping resists); between LATCH and CATCH the latch's cam pushes it back out to CATCH + 2 degrees (about 0.2 N m,
# 1 N at the door's middle); at LATCH or less the latch catches and holds it shut against its stop.
LATCH, CATCH = math.radians(3), math.radians(8)


def door_open(mw: PartState) -> float:
    """How far the door stands open (rad, 0 = shut)."""
    return DOOR0 - mw.hinge


def latch_target(opening: float) -> float | None:
    """The door drive's hinge target for a door open ``opening`` rad (None: follow the door, it swings freely)."""
    if opening <= LATCH:
        return DOOR0 + math.radians(2)        # pressed against the stop
    if opening <= CATCH:
        return DOOR0 - CATCH - math.radians(2)
    return None


_STATE: dict = {}


def _reset_state(environment=None) -> None:
    _STATE.update(arced=False, spilled=False, moved=False)
    if environment is not None:
        set_hinge_target(environment, "microwave", 0.0)


_reset_state()
on_reset(_reset_state)


@on_step
def _latch(environment) -> None:
    from yam_arena.scene import object_states

    mw = object_states(environment, ["microwave"])["microwave"]
    target = latch_target(door_open(mw))
    set_hinge_target(environment, "microwave", mw.hinge if target is None else target)


# -- scoring --------------------------------------------------------------------------------------------------------
MOVE, MW_TILT, SPILL = 0.03, math.radians(10), math.radians(30)
NEAR_SHUT = math.radians(15)


def in_cavity(p, mw: PartState) -> bool:
    x, y, z = mw.to_local(p)
    return bool(0.0 <= x <= D and abs(y) <= W / 2 and FLOOR - 0.005 <= z <= FLOOR + H)


def spoon_points(sp: PartState):
    """The spoon's two ends and its middle."""
    return [sp.to_world((s, 0.0, 0.0)) for s in (-SP_L / 2, 0.0, SP_L / 2)]


def _in_bowl(p, bowl: PartState) -> bool:
    x, y, z = bowl.to_local(p)
    return bool(abs(x) <= B and abs(y) <= B and -0.01 <= z <= BH + 0.02)


def spoon_out(sp: PartState, bowl: PartState, mw: PartState) -> bool:
    """No part of the spoon in (or just above) the bowl or in the cavity, and the spoon not on the floor."""
    pts = spoon_points(sp)
    return bool(sp.pos[2] > TOP - 0.02 and not any(_in_bowl(p, bowl) or in_cavity(p, mw) for p in pts))


def bowl_inside(bowl: PartState, mw: PartState) -> bool:
    """The bowl's whole body (tab included) in the cavity, standing on its floor."""
    x, y, z = mw.to_local(bowl.pos)
    return bool(B + TAB_L <= x <= D - B and abs(y) <= W / 2 - B and abs(z - FLOOR) <= 0.015)


def _update(o) -> tuple[float, bool]:
    mw, bowl, sp = o["microwave"], o["bowl"], o["spoon"]
    opening = door_open(mw)
    _STATE["moved"] |= bool(np.linalg.norm(mw.pos[:2] - np.array(MICROWAVE.pos[:2])) > MOVE or mw.tilt() > MW_TILT)
    _STATE["spilled"] |= bool(bowl.tilt() > SPILL or bowl.pos[2] < TOP - 0.05)
    _STATE["arced"] |= bool(opening <= LATCH and any(in_cavity(p, mw) for p in spoon_points(sp)))
    if _STATE["moved"] or _STATE["spilled"] or _STATE["arced"] or not spoon_out(sp, bowl, mw):
        return 0.0, False
    if not bowl_inside(bowl, mw):
        return 0.25, False
    if opening > NEAR_SHUT:
        return 0.5, False
    if opening > LATCH:
        return 0.75, False
    return 1.0, bool(bowl.resting() and sp.resting(0.02, 1.0))


def success(objects) -> bool:
    """Spoon out, bowl upright on the cavity floor, door latched, nothing ever disqualified; held 1 s."""
    return _update(objects)[1]


def score(objects) -> float:
    """0 if the door was ever latched with any part of the spoon in the cavity (arcing), the bowl ever tilted past
    30 degrees or fell off the table, the microwave ever moved 3 cm or tilted 10 degrees, or the spoon is not out.
    Otherwise 0.25 spoon out of the bowl and the microwave; 0.5 and the bowl inside; 0.75 and the door within 15
    degrees of shut; 1.0 and the door latched (3 degrees or less)."""
    return _update(objects)[0]


# -- scripted demos (pure NumPy plans; demo() wraps them in yam_arena Waypoints) -------------------------------------
OPEN, SHUT = -0.04695, 0.0        # finger joint targets (m): yam_arena.constants.FINGER_OPEN / FINGER_CLOSED
TAB_OPEN = -0.02                  # a 40 mm gap for the 12 mm handle


def frame(z, x) -> np.ndarray:
    """Gripper rotation with approach axis ``z`` and finger-closing axis ``x`` (world)."""
    z = np.asarray(z, float) / np.linalg.norm(z)
    x = np.asarray(x, float) - z * np.dot(x, z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


_P = math.radians(45)
TAB_GRIP = frame((math.cos(_P), 0, -math.sin(_P)), (0, -1, 0))   # forward, pitched 45 degrees down, closing along y
SPOON_GRIP = frame((0, 0, -1), (1, 0, 0))                       # straight down, fingers closing along x
PROBE = frame((0, 0, -1), (0, -1, 0))                           # straight down, fingers shut: a pusher
TAB_X = -B - 0.017                                              # TCP 1.7 cm out from the wall (pad tips 6 mm off)


def tab_tcp(bowl_xy, base_z):
    return (bowl_xy[0] + TAB_X, bowl_xy[1], base_z + (TAB_Z0 + TAB_Z1) / 2)


SET_XY = (MX + 0.115, MY + 0.01)  # where the bowl goes: its centre 11.5 cm in, so its handle ends 3.5 cm inside the door


def _spoon_out():
    """Right arm: pinch the spoon's handle 2 cm from its end, lift it clear of the bowl, lay it in the tray."""
    grip = SP_POS - (SP_L / 2 - 0.02) * _U
    above = grip + (0, 0, 0.08)
    lifted = grip + (0, 0, 0.12)
    over_tray = (TX, TY + 0.01, TOP + 0.08)
    low = (TX, TY + 0.01, TOP + 0.035)
    return [(3.0, tuple(above), SPOON_GRIP, OPEN, "above the spoon handle"),
            (1.5, tuple(grip), SPOON_GRIP, OPEN, "around the spoon handle"), (0.8, tuple(grip), SPOON_GRIP, SHUT, "pinch"),
            (1.5, tuple(lifted), SPOON_GRIP, SHUT, "lift the spoon out"),
            (2.0, over_tray, SPOON_GRIP, SHUT, "over the tray"), (1.0, low, SPOON_GRIP, SHUT, "lower"),
            (0.6, low, SPOON_GRIP, OPEN, "let go"), (1.0, over_tray, SPOON_GRIP, OPEN, "clear")]


def _bowl_in():
    """Right arm: pinch the bowl's handle from the front, lift it 2.5 cm over the cavity floor, carry it in, set it
    down, back out."""
    at = tab_tcp((BX, BY), TOP)
    back = tuple(np.subtract(at, 0.07 * TAB_GRIP[:, 2]))     # 7 cm back along the approach
    lift = (at[0], at[1], TOP + FLOOR + 0.025 + (TAB_Z0 + TAB_Z1) / 2)
    mouth = (MX - 0.03 + TAB_X, SET_XY[1], lift[2])
    inside = (SET_XY[0] + TAB_X, SET_XY[1], lift[2])
    down = (inside[0], inside[1], TOP + FLOOR + 0.004 + (TAB_Z0 + TAB_Z1) / 2)
    out = tuple(np.subtract(down, 0.14 * TAB_GRIP[:, 2]) + (0, 0, 0.02))
    return [(3.0, back, TAB_GRIP, TAB_OPEN, "in front of the handle"), (1.5, at, TAB_GRIP, TAB_OPEN, "around the handle"),
            (0.8, at, TAB_GRIP, SHUT, "pinch the handle"), (1.5, lift, TAB_GRIP, SHUT, "lift the bowl"),
            (2.5, mouth, TAB_GRIP, SHUT, "to the microwave"), (2.0, inside, TAB_GRIP, SHUT, "into the cavity"),
            (1.0, down, TAB_GRIP, SHUT, "set it down"), (0.6, down, TAB_GRIP, TAB_OPEN, "let go"),
            (1.5, out, TAB_GRIP, TAB_OPEN, "back out"),
            (1.5, (0.12, -0.20, TOP + 0.12), TAB_GRIP, TAB_OPEN, "clear")]


HINGE_XY = np.array([MX + DOOR.anchor[0], MY + DOOR.anchor[1]])
PUSH_Z = TOP + 0.12
OUTSIDE = DT / 2 + 0.014   # TCP this far off the door's middle plane, on its outer side (finger half-width + gap)


def door_point(opening: float, r: float, off: float = OUTSIDE) -> tuple:
    """A point ``r`` along the door from the hinge and ``off`` out from it, with the door open ``opening`` rad."""
    along = np.array([-math.sin(opening), -math.cos(opening)])
    out = np.array([-math.cos(opening), math.sin(opening)])
    x, y = HINGE_XY + r * along + off * out
    return (float(x), float(y), PUSH_Z)


def _close_door(start_s: float):
    """Left arm, fingers shut and pointing down: wait, come to the door's outer face, sweep it shut in 15-degree
    steps (moving out along it as it closes), push it past the latch, back off."""
    sweep = [(90, 0.12), (75, 0.13), (60, 0.14), (45, 0.15), (30, 0.16), (15, 0.17), (0, 0.18)]
    plan = [(start_s, (0.15, 0.25, TOP + 0.20), PROBE, SHUT, "wait"),
            (2.0, door_point(DOOR0, 0.12, 0.05), PROBE, SHUT, "beside the door"),
            (1.0, door_point(DOOR0, 0.12), PROBE, SHUT, "on its outer face")]
    plan += [(0.8, door_point(math.radians(a), r), PROBE, SHUT, "swing it shut") for a, r in sweep]
    plan += [(1.0, door_point(math.radians(-2), 0.18, OUTSIDE - 0.004), PROBE, SHUT, "push past the latch"),
             (1.0, door_point(0.0, 0.18, 0.06), PROBE, SHUT, "back off"),
             (1.5, (0.15, 0.25, TOP + 0.20), PROBE, SHUT, "clear")]
    return plan


def plan(route: str = "safe") -> dict[str, list]:
    """``safe``: take the spoon out, then load the bowl, then close the door. ``naive``: load the bowl with the
    spoon still in it and close the door."""
    right = _bowl_in() if route == "naive" else _spoon_out() + _bowl_in()
    return {"right": right, "left": _close_door(sum(w[0] for w in right) - 1.5)}


def demo():
    """The safe route (DEMO_ROUTE=naive plays the naive contrast: the bowl goes in with the spoon, which scores 0)."""
    from yam_arena.demo import Waypoint

    route = os.environ.get("DEMO_ROUTE", "safe")
    return {arm: [Waypoint(d, tuple(p), f, label, r) for d, p, r, f, label in wps] for arm, wps in plan(route).items()}
