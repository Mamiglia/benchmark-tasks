"""On the station table: a small steel block (4 x 4 x 0.6 cm) is screwed to the table 30 cm in front of the arms, a
little right of centre. A white-and-red cord, 19 cm long, is tied to the middle of its top and lies slack and straight
on the table toward the arms; a small yellow block (3 x 3 x 1.6 cm) is tied to its free end and lies flat on the table.
A kitchen knife lies flat on the table in front of the right arm, its dark handle toward the right, its blade pointing
left with the cutting edge facing away from the arms. The robot must cut the cord with the knife.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from cordkit import CordRig, CordState, on_reset  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Cut the string with the knife."

WOOD, CREAM, RED, YELLOW = (0.80, 0.65, 0.45), (0.95, 0.93, 0.85), (0.80, 0.12, 0.10), (1.0, 0.80, 0.10)
HANDLE, STEEL = (0.22, 0.13, 0.08), (0.55, 0.57, 0.60)

# -- the anchor (fixed) and the cord -------------------------------------------------------------------------------
PLATE = (0.04, 0.04, 0.006)              # steel block screwed to the table; the cord is tied at the middle of its top
ANCHOR = (0.30, -0.06, TOP + 0.008)      # where the cord is tied (its axis 2 mm above the block)
ANCHOR_BOXES = (((ANCHOR[0], ANCHOR[1], TOP + PLATE[2] / 2), PLATE, STEEL),)
N_SEG, SEG = 19, 0.01                    # 19 cm of cord in 1 cm links, banded white and red every 1 cm
LAY = (-1.0, 0.0, 0.0)                   # the cord starts laid out straight toward the arms (-x)
TAG = (0.016, 0.03, 0.03)                # block, own frame: z along the cord; lying flat, its 1.6 cm (x) is vertical
# knife: handle along y, blade pointing +y (left), edge facing +x (away from the arms); frame = handle centre
KNIFE = (0.22, -0.23, TOP + 0.008)
HANDLE_BOXES = (((0.0, 0.0, 0.0), (0.024, 0.10, 0.016), HANDLE),           # 10 x 2.4 x 1.6 cm handle
                ((0.0, 0.054, 0.0), (0.030, 0.008, 0.016), STEEL))          # bolster
BLADE_C, BLADE = (0.0, 0.088, 0.0), (0.020, 0.060, 0.002)                   # 6 x 2 cm blade, 2 mm thick
CUT_N = 1.5
CUT_HOLD = 24                            # physics steps at 240 Hz: the edge must stay pressed 0.1 s

RIG = CordRig(name="cord", pos=ANCHOR, size=(0.05, 0.25, 0.02), color=STEEL, anchor_boxes=ANCHOR_BOXES, lay=LAY,
              n_seg=N_SEG, seg_len=SEG, radius=0.0015, seg_mass=0.001, seg_inertia=1e-7, seg_colors=(CREAM, RED),
              tag_size=TAG, tag_mass=0.006, tag_color=YELLOW, knife_pos=KNIFE, handle_boxes=HANDLE_BOXES,
              knife_mass=0.05, blade_centre=BLADE_C, blade_size=BLADE, blade_mass=0.015, blade_color=STEEL,
              table_z=TOP, cut_n=CUT_N, cut_hold=CUT_HOLD)
OBJECTS = [RIG]
TAG0 = RIG.tag_centre                    # (0.095, -0.06, TOP + 0.008)

# -- scoring --------------------------------------------------------------------------------------------------------
HAND_M = 0.05        # a cut closer than this to the other gripper's pads: 0
ANCHOR_JOINTS = 3    # a cut within 3 cm of the anchor (joints 0-2): 0
LIFT_M = 0.05        # knife lifted 5 cm off the table
HELD_M = 0.025       # the tag counts as held when a gripper's tool point is within 2.5 cm of its centre
OFF_TABLE = TOP - 0.03
DOWN_Z, FREE_M = TOP + 0.03, 0.06  # set down: handle centre under 3 cm, both tool points 6 cm or more away, at rest

_STATE: dict = {}


def _reset_state(environment=None) -> None:
    _STATE.update(lifted=False, engaged=False)


_reset_state()
on_reset(_reset_state)


def _update(c: CordState) -> float:
    lifted = c.knife_pos[2] >= KNIFE[2] + LIFT_M
    held = any(np.linalg.norm(np.asarray(p) - c.tag_pos) <= HELD_M for p in c.tcp.values())
    _STATE["lifted"] |= bool(lifted)
    _STATE["engaged"] |= bool(lifted and held)
    if c.touched is not None or c.blade_on_table:
        return 0.0
    if any(cut.hand_m < HAND_M or cut.joint < ANCHOR_JOINTS or cut.held_m > HELD_M for cut in c.cuts):
        return 0.0
    if c.knife_pos[2] < OFF_TABLE or c.blade_pos[2] < OFF_TABLE or c.tag_pos[2] < OFF_TABLE:
        return 0.0
    if c.cuts:
        free = all(np.linalg.norm(np.asarray(p) - c.knife_pos) >= FREE_M for p in c.tcp.values())
        down = c.knife_pos[2] <= DOWN_Z and float(np.linalg.norm(c.knife_vel[:3])) < 0.01
        return 1.0 if free and down else 0.75
    return 0.5 if _STATE["engaged"] else 0.25 if _STATE["lifted"] else 0.0


def score(objects) -> float:
    """0 if the blade ever touched a gripper or the table, a cut was under 5 cm from the other gripper's pads, within 3 cm of the
    anchor, or made while the other gripper was not holding the cord or tag, or the knife or tag ended off the table.
    Otherwise 1.0 cut and knife set down; 0.75 cut; 0.5 knife lifted 5 cm while the tag was held; 0.25 knife lifted
    5 cm; else 0."""
    return _update(objects["cord"])


def success(objects) -> bool:
    """Cut safely, knife set down and at rest, held for 1 s."""
    return score(objects) == 1.0


# -- scripted demo (pure NumPy plan; demo() wraps it in yam_arena Waypoints) -----------------------------------------
OPEN, SHUT = -0.04695, 0.0  # yam_arena.constants.FINGER_OPEN / FINGER_CLOSED
PULL = 0.03                          # the left arm is commanded 3 cm past the point where the cord comes taut


def frame(z, x) -> np.ndarray:
    """Gripper rotation with approach axis ``z`` and finger-closing axis ``x`` (world)."""
    z = np.asarray(z, float) / np.linalg.norm(z)
    x = np.asarray(x, float) - z * np.dot(x, z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


DOWN_X = frame((0, 0, -1), (1, 0, 0))                                # right: straight down, fingers closing along x
DOWN_Y = frame((0, 0, -1), (0, -1, 0))                               # left: straight down, fingers closing along y
TILT = math.radians(30)
HOLD = frame((0, -math.sin(TILT), -math.cos(TILT)), (0, -1, 0))      # left, holding up high: tilted in from its side
GRIP_DY, GRIP_DZ = 0.035, 0.009      # tool point on the handle: 3.5 cm toward the bolster, 9 mm above its centre
CUT_Z = ANCHOR[2] + 0.09             # cut 9 cm above the knot, at mid-height of the cord held upright
CONTACT_DY = BLADE_C[1] - BLADE[1] / 2 + 0.035   # cut 3.5 cm from the blade's heel: 9.3 cm from the handle centre
PUSH = float(os.environ.get("DEMO_PUSH_CM", 6)) / 100  # the edge goes 6 cm past the upright cord's line
# left tool point when the cord is just taut and upright: the tag's cord end straight above the knot
TAUT = np.array([ANCHOR[0] - TAG[2] / 2, ANCHOR[1], ANCHOR[2] + N_SEG * SEG + GRIP_DZ])


def _right(wait_s: float):
    """(duration, pos, rot, finger, label): pick the knife up, push its edge into the cord, set it down."""
    kx, ky, kz = KNIFE
    grip = (kx, ky + GRIP_DY, kz + GRIP_DZ)
    ty = ANCHOR[1] - (CONTACT_DY - GRIP_DY)          # tool point y that puts the contact point on the cord
    tz = CUT_Z + GRIP_DZ
    x_touch = ANCHOR[0] - BLADE[0] / 2 - RIG.radius
    back, through = (x_touch - 0.03, ty, tz), (x_touch + PUSH, ty, tz)
    rest = (0.20, -0.21, kz + GRIP_DZ + 0.002)
    return [(2.0, (kx, ky + GRIP_DY, kz + 0.08), DOWN_X, OPEN, "over the knife"),
            (1.5, grip, DOWN_X, OPEN, "around the handle"), (0.8, grip, DOWN_X, SHUT, "grip"),
            (1.5, (kx, ky + GRIP_DY, tz + 0.03), DOWN_X, SHUT, "lift"),
            (2.0, (back[0], ty, tz + 0.03), DOWN_X, SHUT, "beside the cord"),
            (0.8, back, DOWN_X, SHUT, "level with the cut"),
            (wait_s, back, DOWN_X, SHUT, "wait for the left arm"),
            (2.5, through, DOWN_X, SHUT, "push the edge into the cord"),
            (0.5, through, DOWN_X, SHUT, "hold"),
            (1.5, back, DOWN_X, SHUT, "back out"),
            (2.0, (rest[0], rest[1], rest[2] + 0.06), DOWN_X, SHUT, "carry"),
            (1.2, rest, DOWN_X, SHUT, "lay it flat"), (0.6, rest, DOWN_X, OPEN, "let go"),
            (1.0, (rest[0], rest[1], rest[2] + 0.08), DOWN_X, OPEN, "clear")]


def _left():
    """Grip the tag from above, bring it over the knot and lift it 3 cm past taut: the cord stands upright and taut."""
    grip = TAG0 + (0, 0, GRIP_DZ)
    return [(2.5, tuple(grip + (0, 0, 0.07)), DOWN_Y, OPEN, "over the tag"),
            (1.5, tuple(grip), DOWN_Y, OPEN, "around the tag"), (0.8, tuple(grip), DOWN_Y, SHUT, "grip"),
            (1.5, tuple(TAUT + (-0.08, 0, -0.10)), HOLD, SHUT, "lift toward the knot"),
            (1.5, tuple(TAUT - (0, 0, 0.04)), HOLD, SHUT, "over the knot"),
            (1.5, tuple(TAUT + (0, 0, PULL)), HOLD, SHUT, "pull it taut")]


def plan(one_hand: bool = False) -> dict[str, list]:
    left = _left()
    ready = sum(w[0] for w in left)
    reach = sum(w[0] for w in _right(0.0)[:6])
    right = _right(max(0.5, ready - reach + 0.5))
    if one_hand:
        return {"right": right}
    cut_done = sum(w[0] for w in right[:10])     # until the knife has backed out
    total = sum(w[0] for w in right)
    eased = tuple(TAUT - (0, 0, 0.02))
    return {"left": left + [(cut_done - ready, left[-1][1], HOLD, SHUT, "hold"),
                            (1.0, eased, HOLD, SHUT, "ease off"),
                            (total - cut_done - 1.0, eased, HOLD, SHUT, "hold the tag")],
            "right": right}


def demo():
    """The left arm lifts the tag and holds the cord upright and taut; the right arm picks the knife up flat, pushes its
    edge into the middle of the cord, backs out and lays the knife down. DEMO_ONE_HAND=1 leaves the left arm still."""
    from yam_arena.demo import Waypoint

    out = plan(one_hand=bool(os.environ.get("DEMO_ONE_HAND")))
    return {arm: [Waypoint(d, tuple(p), f, label, r) for d, p, r, f, label in wps] for arm, wps in out.items()}
