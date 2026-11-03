"""On the station table, between the arms and a little toward the right one: a loose grey box 20 x 16 x 6 cm standing
on four small feet, with an upright grey handle fin on its left side. Its front strip is a fixed deck carrying a lock
with an orange key standing upright in it (a flat bow, 5 cm wide, 1 cm thick, 4 cm tall), surrounded by a ring of
black tick marks every 15 degrees. Behind the deck, a blue lid with a blue knob covers the box; it slides sideways in
rails once the box is unlocked.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from lockkit import LockBox, LockParams  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "This lock turns the opposite way to a normal lock. Unlock the box with the orange key, then slide its blue lid open."

# The submitted lock: unlocks CLOCKWISE seen from above (the reverse of the usual counter-clockwise), breakaway peak
# 0.40 N m at 6 deg, 0.30 N m running, shear pin at 0.6 N m. Variants: change LockParams (see README).
PARAMS = LockParams()

GREY, DECK, BLACK = (0.45, 0.46, 0.48), (0.62, 0.63, 0.65), (0.05, 0.05, 0.05)
ORANGE, BLUE, KNOB = (1.0, 0.45, 0.0), (0.12, 0.32, 0.85), (0.08, 0.20, 0.60)

# -- geometry (box frame: origin on the table under the box centre, x forward, y left; metres) ---------------------
F = 0.003  # feet height
W = 0.006  # wall, floor, deck and lid thickness
FEET = tuple(((sx * 0.09, sy * 0.07, F / 2), (0.012, 0.012, F), BLACK) for sx in (-1, 1) for sy in (-1, 1))
BODY = (
    ((0.0, 0.0, F + W / 2), (0.20, 0.16, W), GREY),  # floor
    ((-0.097, 0.0, 0.036), (W, 0.16, 0.054), GREY),  # front wall
    ((0.097, 0.0, 0.036), (W, 0.16, 0.054), GREY),  # back wall
    ((0.0, 0.077, 0.036), (0.188, W, 0.054), GREY),  # left wall
    ((0.0, -0.077, 0.036), (0.188, W, 0.054), GREY),  # right wall
    ((-0.06, 0.0, 0.066), (0.08, 0.16, W), DECK),  # deck over the front 8 cm, top 6.9 cm above the table
    ((0.0, 0.105, F + 0.005), (0.06, 0.05, 0.010), GREY),  # handle bracket off the left wall's foot
    ((0.0, 0.13, (F + 0.110) / 2), (0.06, 0.010, 0.107), GREY),  # handle fin, top 11 cm above the table
)
KEY_AT = (-0.06, 0.0, 0.069)  # key axis on the deck top
# Dial: 24 unlabeled ticks every 15 deg at 2.9-3.5 cm from the key axis (the two along the bow's start direction, y,
# 1 cm long). Symmetric: it shows how far the key has turned, not which way unlocks.
MARKS = tuple(
    (
        (KEY_AT[0] + r * math.cos(math.radians(a)), r * math.sin(math.radians(a)), 0.0695),
        (length, 0.0015, 0.001),
        BLACK,
        float(a),
    )
    for a in range(0, 360, 15)
    for length, r in [(0.010, 0.030) if a % 180 == 90 else (0.006, 0.032)]
)
KEY = (
    ((0.0, 0.0, 0.0025), (0.010, 0.010, 0.005), ORANGE),  # shank
    ((0.0, 0.0, 0.025), (0.010, 0.050, 0.040), ORANGE),  # bow, long side along y, 0.5-4.5 cm above the deck
)
LID_AT = (0.04, 0.0, 0.066)  # lid centre, closed: over x -2..+10 cm, resting on the walls
LID = (
    ((0.0, 0.0, 0.0), (0.12, 0.16, W), BLUE),
    ((0.0, 0.0, W / 2 + 0.0125), (0.03, 0.03, 0.025), KNOB),  # knob, top 9.5 cm above the table
)

BOX_XY = (0.30, -0.10)
LOCKBOX = LockBox(
    "lockbox", pos=(*BOX_XY, TOP + 0.001), size=(0.20, 0.215, 0.111), color=GREY, mass=0.55,
    body_boxes=BODY, feet=FEET, marks=MARKS, key_pos=KEY_AT, key_boxes=KEY, key_mass=0.03,
    lid_pos=LID_AT, lid_boxes=LID, lid_mass=0.12, lid_travel=0.10, lid_dir=-1.0,
    friction=1.0, feet_friction=0.3, params=PARAMS,
)  # fmt: skip
OBJECTS = [LOCKBOX]

# World positions the demo uses (box frame + BOX_XY).
KEY_XY = (BOX_XY[0] + KEY_AT[0], BOX_XY[1])  # (0.24, -0.10)
KNOB_XY = (BOX_XY[0] + LID_AT[0], BOX_XY[1])  # (0.34, -0.10)
FIN_XY = (BOX_XY[0], BOX_XY[1] + 0.13)  # (0.30, 0.03)

# -- scoring ---------------------------------------------------------------------------------------------------------
OPEN_FULL, OPEN_PART = 0.08, 0.02  # lid travel (m)
TURN_PART = 30.0  # key angle (deg, unlock sense)
TIP_DEG, FALL_M = 20.0, 0.05  # box ever tilted past this / dropped this far below its start (off the table): 0
SHIFT_M, BOX_TURN_DEG, UNSTEADY_CAP = 0.03, 15.0, 0.5  # box ends this far off its start outline: capped


def box_moved(s) -> bool:
    """The box ends more than 3 cm (centre) or 15 degrees from its taped start outline."""
    return math.hypot(s.pos[0] - BOX_XY[0], s.pos[1] - BOX_XY[1]) > SHIFT_M or abs(s.yaw_deg()) > BOX_TURN_DEG


def score(objects) -> float:
    """End of episode. 0 if the shear pin broke, the locked lid was pried, or the box ever tipped past 20 degrees or
    fell off the table. Otherwise 1.0 lid open >= 8 cm, 0.75 lid open >= 2 cm, 0.5 key at >= 75 deg (unlocked),
    0.25 key at >= 30 deg, else 0; capped at 0.5 if the box ends more than 3 cm or 15 degrees off its start."""
    s = objects["lockbox"]
    if s.damaged or s.pried or s.max_tilt_deg > TIP_DEG or s.max_drop_m > FALL_M:
        return 0.0
    if s.unlocked and s.lid_m >= OPEN_FULL:
        out = 1.0
    elif s.unlocked and s.lid_m >= OPEN_PART:
        out = 0.75
    elif s.key_deg >= PARAMS.unlock_deg:
        out = 0.5
    elif s.key_deg >= TURN_PART:
        out = 0.25
    else:
        out = 0.0
    return min(out, UNSTEADY_CAP) if box_moved(s) else out


def success(objects) -> bool:
    """Lid open at least 8 cm after unlocking, key intact, box steady and in place; must hold for 1 s."""
    return score(objects) == 1.0


# -- demo (yam_scripted plays it open loop) ----------------------------------------------------------------------------
GRIP_YAW = 90.0  # grip yaw on the key at rest: bow along y, so the fingers close along x across its faces
PROBE_DEG = 4.0  # the "usual" way, gently: about 0.35 N m on the stop, under the 0.6 N m pin
TURN_DEG = 92.0  # the unlock way, past the 75 deg release and short of the 95 deg end stop
FORCE_DEG = 60.0  # DEMO_FORCE_WRONG: far past the stop (joint6 stays inside its limit)
GRIP_Z = TOP + 0.093  # key: pad tips 1 cm above the deck
KNOB_Z = TOP + 0.088  # knob: pad tips 0.5 cm above the lid
FIN_Z = TOP + 0.090  # handle fin: pad tips 0.7 cm above the box top
HOVER = TOP + 0.16


def demo():
    """Left arm grips the handle fin and holds the box still throughout. Right arm grips the key bow (fingers across
    its faces), tries the conventional (counter-clockwise) way by 4 deg, holds against the stop, turns 92 deg the
    other way, lets go, grips the knob and slides the lid 9.5 cm toward -y; then both let go. Written for the
    submitted (clockwise-unlocking) lock. DEMO_FORCE_WRONG=1 forces the key 60 deg the conventional way instead of
    the probe: the pin shears and the trial scores 0. Waypoint yaw is counter-clockwise positive seen from above."""
    probe = math.radians(GRIP_YAW + (FORCE_DEG if os.environ.get("DEMO_FORCE_WRONG") else PROBE_DEG))
    grip, turn = math.radians(GRIP_YAW), math.radians(GRIP_YAW - TURN_DEG)
    (kx, ky), (nx, ny), (fx, fy) = KEY_XY, KNOB_XY, FIN_XY
    pulled = (nx, ny - 0.095, KNOB_Z)
    right = [
        Waypoint(2.0, (kx, ky, HOVER), FINGER_OPEN, "over the key", grip),
        Waypoint(2.5, (kx, ky, HOVER), FINGER_OPEN, "wait for the left arm", grip),
        Waypoint(1.5, (kx, ky, GRIP_Z), FINGER_OPEN, "down to the bow", grip),
        Waypoint(0.8, (kx, ky, GRIP_Z), FINGER_CLOSED, "grip the bow", grip),
        Waypoint(1.0, (kx, ky, GRIP_Z), FINGER_CLOSED, "try the usual way", probe),
        Waypoint(0.5, (kx, ky, GRIP_Z), FINGER_CLOSED, "hold: the key does not turn", probe),
        Waypoint(4.0, (kx, ky, GRIP_Z), FINGER_CLOSED, "turn the other way", turn),
        Waypoint(0.5, (kx, ky, GRIP_Z), FINGER_CLOSED, "hold", turn),
        Waypoint(0.6, (kx, ky, GRIP_Z), FINGER_OPEN, "let go of the key", turn),
        Waypoint(1.0, (kx, ky, HOVER), FINGER_OPEN, "up", turn),
        Waypoint(2.0, (nx, ny, HOVER), FINGER_OPEN, "over the knob"),
        Waypoint(1.5, (nx, ny, KNOB_Z), FINGER_OPEN, "down to the knob"),
        Waypoint(0.8, (nx, ny, KNOB_Z), FINGER_CLOSED, "grip the knob"),
        Waypoint(3.0, pulled, FINGER_CLOSED, "slide the lid open"),
        Waypoint(0.6, pulled, FINGER_OPEN, "let go of the knob"),
        Waypoint(1.0, (pulled[0], pulled[1], HOVER), FINGER_OPEN, "up"),
    ]
    reach = [
        Waypoint(2.0, (fx, fy, HOVER), FINGER_OPEN, "over the handle fin"),
        Waypoint(1.5, (fx, fy, FIN_Z), FINGER_OPEN, "down to the fin"),
        Waypoint(0.8, (fx, fy, FIN_Z), FINGER_CLOSED, "grip the fin"),
    ]
    hold = sum(w.duration for w in right[:-1]) - sum(w.duration for w in reach)  # until the right arm lets go
    left = [
        *reach,
        Waypoint(hold, (fx, fy, FIN_Z), FINGER_CLOSED, "hold the box still"),
        Waypoint(0.6, (fx, fy, FIN_Z), FINGER_OPEN, "let go of the fin"),
        Waypoint(1.0, (fx, fy, HOVER), FINGER_OPEN, "up"),
    ]
    if os.environ.get("DEMO_ONE_HAND"):  # contrast: nobody holds the box, so it turns with the key
        return {"right": right}
    return {"left": left, "right": right}
