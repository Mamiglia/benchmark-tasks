"""On the station table, in front of the arms (x away from the arms, y to their left, heights above the table top):
a pine box with a sliding lid, fixed to the table, centre x 30, y 4 cm, 18 x 20 cm outside and 7.5 cm tall. Its lid,
a 1 cm pine board 18 x 20 cm, lies shut on the rim between two guide strips that run along both long sides of the box
and on past its right-hand end, over a fixed pine block of the same size as the box, so the lid can only slide along
y and stays supported when it is slid open to the right. A set of keys on a brown leather fob lies on the lid. A
white ceramic catch-all dish stands to the near left of the box (centre x 13, y 0), a pair of reading glasses stands
open on the table to the left (front frame to the left, arms pointing right, toward the box), and a mug of coffee stands just
beyond the box (x 47, y 4). The box is 7 cm deep inside: the open glasses fit under its lid only standing or upside
down (lying on their lenses or a side, their arms or front stand 13 cm tall).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

# Idea first proposed in robocurve/benchmark-tasks#532; independent implementation.

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from glasskit import ON_RESET, compound, corners, tilt_deg  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
T = TABLE_TOP[SCENE]
INSTRUCTION = "Put the glasses away in the box and the keys in the dish, and close the box."

PINE, PINE_DARK, WHITE = (0.80, 0.64, 0.42), (0.62, 0.46, 0.28), (0.93, 0.92, 0.90)
BLACK, LENS, LEATHER, BRASS, MUG_RED = (0.05, 0.05, 0.06), (0.62, 0.78, 0.86), (0.40, 0.22, 0.10), \
    (0.80, 0.66, 0.30), (0.62, 0.12, 0.10)

# -- layout (env frame: x forward, y left, z up; m) ------------------------------------------------------------------
BOX_XY = (0.30, 0.04)
BOX_X, BOX_Y, BOX_H, WALL, FLOOR = 0.18, 0.20, 0.075, 0.01, 0.005
INNER_X = (BOX_XY[0] - BOX_X / 2 + WALL, BOX_XY[0] + BOX_X / 2 - WALL)  # 0.22 .. 0.38
INNER_Y = (BOX_XY[1] - BOX_Y / 2 + WALL, BOX_XY[1] + BOX_Y / 2 - WALL)  # -0.05 .. 0.13
DECK_XY = (0.30, -0.16)  # the fixed block the open lid slides onto
RAIL_W, RAIL_H, RAIL_Y = 0.01, 0.093, (-0.27, 0.15)  # guide strips, 2 mm clear of the lid on each side
LID_T, LID_MASS = 0.010, 0.16  # 18 x 20 x 1 cm pine, 450 kg/m^3
LID_Z = T + BOX_H + LID_T / 2  # lid centre height when it rests on the rim or the block
DISH_XY, DISH_OUT, DISH_H, DISH_WALL, DISH_BASE = (0.13, 0.0), 0.12, 0.018, 0.008, 0.004
GLASSES_START = (0.16, 0.30)  # origin: bottom middle of the front frame; the arms point right (-y)
GLASSES_MASS = 0.028
KEYS_START = (0.30, 0.07)  # origin: bottom middle of the fob, on the lid
KEYS_MASS = 0.032
MUG_XY, MUG_R, MUG_H, MUG_MASS = (0.47, 0.04), 0.041, 0.095, 0.38


def _box_parts():
    """The box, the lid block and the two guides, as (centre, size, colour) in the frame at (BOX_XY, table top)."""
    h = BOX_H
    hx, hy = BOX_X / 2 - WALL / 2, BOX_Y / 2 - WALL / 2
    gx, gy = BOX_X / 2 + 0.002 + RAIL_W / 2, sum(RAIL_Y) / 2 - BOX_XY[1]
    return (
        ((0.0, 0.0, FLOOR / 2), (BOX_X, BOX_Y, FLOOR), PINE),
        ((-hx, 0.0, h / 2), (WALL, BOX_Y, h), PINE),
        ((hx, 0.0, h / 2), (WALL, BOX_Y, h), PINE),
        ((0.0, -hy, h / 2), (BOX_X - 2 * WALL, WALL, h), PINE),
        ((0.0, hy, h / 2), (BOX_X - 2 * WALL, WALL, h), PINE),
        ((DECK_XY[0] - BOX_XY[0], DECK_XY[1] - BOX_XY[1], h / 2), (BOX_X, BOX_Y, h), PINE_DARK),  # lid block
        *(((s * gx, gy, RAIL_H / 2), (RAIL_W, RAIL_Y[1] - RAIL_Y[0], RAIL_H), PINE_DARK) for s in (-1, 1)),  # guides
    )


def _dish_parts():
    (cx, cy), o, w, h = DISH_XY, DISH_OUT, DISH_WALL, DISH_H
    return [
        Box("dish_base", (cx, cy, T + DISH_BASE / 2), WHITE, size=(o, o, DISH_BASE)),
        Box("dish_near", (cx - o / 2 + w / 2, cy, T + h / 2), WHITE, size=(w, o, h)),
        Box("dish_far", (cx + o / 2 - w / 2, cy, T + h / 2), WHITE, size=(w, o, h)),
        Box("dish_right", (cx, cy - o / 2 + w / 2, T + h / 2), WHITE, size=(o - 2 * w, w, h)),
        Box("dish_left", (cx, cy + o / 2 - w / 2, T + h / 2), WHITE, size=(o - 2 * w, w, h)),
    ]


# Reading glasses, open, in their own frame: the front frame across x at y 0 (its bottom at z 0), the two arms
# running back along +y from hinges at the frame's ends, each ending in an ear bend that comes down to z 0.
# 134 mm wide, 133 mm deep, 43 mm tall; acetate frame, CR-39 lenses, 28 g.
GLASSES_PARTS = (
    ((0.0, 0.0, 0.040), (0.134, 0.005, 0.006), BLACK),  # brow bar and bridge
    *(((s * 0.037, 0.0, 0.004), (0.050, 0.005, 0.004), BLACK) for s in (-1, 1)),  # lower rims
    *(((s * 0.037, 0.0, 0.021), (0.046, 0.003, 0.030), LENS) for s in (-1, 1)),  # lenses
    *(((s * 0.064, 0.0, 0.022), (0.006, 0.005, 0.040), BLACK) for s in (-1, 1)),  # end pieces
    *(((s * 0.064, 0.060, 0.036), (0.004, 0.115, 0.005), BLACK) for s in (-1, 1)),  # arms
    *(((s * 0.064, 0.123, 0.019), (0.004, 0.010, 0.038), BLACK) for s in (-1, 1)),  # ear bends
)
GLASSES_TOP = 0.043
# Keys: a leather fob 40 x 25 x 12 mm with a brass house key lying flat beside it (origin: bottom middle of the fob).
KEYS_PARTS = (
    ((0.0, 0.0, 0.006), (0.040, 0.025, 0.012), LEATHER),
    ((0.035, 0.0, 0.0015), (0.030, 0.018, 0.003), BRASS),
)

# Pine on pine: static friction about 0.4 (wood on wood, clean and dry, 0.25 to 0.5: Engineering ToolBox, "Friction
# and Friction Coefficients for some Common Materials"); dynamic 0.3. glasskit combines a contact's two materials
# by their minimum, so everything touching the box, guides or lid slides with this friction.
PINE_MU = (0.4, 0.3)
BOX = compound("pine_box", (*BOX_XY, T), _box_parts(), None, friction=PINE_MU)
LID = compound("lid", (*BOX_XY, LID_Z), (((0.0, 0.0, 0.0), (BOX_X, BOX_Y, LID_T), PINE),), LID_MASS, friction=PINE_MU)
GLASSES = compound("glasses", (*GLASSES_START, T + 0.0005), GLASSES_PARTS, GLASSES_MASS, yaw=math.pi)
KEYS = compound("keys", (*KEYS_START, T + BOX_H + LID_T + 0.0005), KEYS_PARTS, KEYS_MASS)
MUG = Cylinder("mug", (*MUG_XY, T + MUG_H / 2), MUG_RED, mass=MUG_MASS, radius=MUG_R, height=MUG_H)
OBJECTS = [BOX, *_dish_parts(), LID, GLASSES, KEYS, MUG]
START = {"lid": np.array([*BOX_XY, LID_Z]), "mug": np.array([*MUG_XY, T + MUG_H / 2])}

# -- scoring ---------------------------------------------------------------------------------------------------------
# Lens-safe stances: standing on the lower rims and ear bends, or upside down on the brow bar and arms (both only
# touch the frame). Resting 60 to 120 degrees from standing is lying on the lenses or on one side.
GLASSES_FALLEN_DEG = (60.0, 120.0)
GLASSES_UPRIGHT_DEG = 20.0  # within 20 degrees of standing, or of upside down
REST_V, REST_W = 0.02, 0.5  # m/s, rad/s
MUG_TIP_DEG, MUG_MOVE = 15.0, 0.02
LID_TILT_DEG, LID_LIFT = 10.0, 0.015  # lid off its guides: tilted, or its underside 1.5 cm above the rim or block
LID_SHUT_XY, LID_SHUT_DEG = 0.015, 5.0
FLOOR_Z = T - 0.05  # below this an object has gone off the table
ON_FLOOR_Z = 0.010  # the glasses' lowest point within 1 cm of the box floor's top
STILL_V = 0.02

LATCH: dict[str, bool] = {}
DEBUG = bool(os.environ.get("GSB_DEBUG"))
_N = [0]


def _debug(o) -> None:
    _N[0] += 1
    if _N[0] % 20 == 0:
        print("[glasses-sliding-box] " + " ".join(f"{n} {np.round(o[n].pos, 3)} tilt {tilt_deg(o[n].quat_xyzw):.0f}"
                                                 for n in ("lid", "glasses", "keys", "mug")), flush=True)


def clear_latches() -> None:
    LATCH.clear()


ON_RESET.append(clear_latches)


def inside_box(p, margin=0.0) -> bool:
    return (INNER_X[0] + margin <= p[0] <= INNER_X[1] - margin and INNER_Y[0] + margin <= p[1] <= INNER_Y[1] - margin
            and p[2] < T + BOX_H)


def keys_in_dish(k) -> bool:
    (cx, cy), r = DISH_XY, DISH_OUT / 2 - DISH_WALL
    return abs(k.pos[0] - cx) <= r and abs(k.pos[1] - cy) <= r and k.pos[2] <= T + DISH_H


def glasses_in_box(g) -> bool:
    """On the box floor: every corner within the inner walls, the lowest within 1 cm of the floor's top, within 20
    degrees of standing or of upside down, and still."""
    pts = corners(GLASSES, g.pos, g.quat_xyzw)
    tilt = tilt_deg(g.quat_xyzw)
    return (all(inside_box(p) for p in pts) and abs(pts[:, 2].min() - (T + FLOOR)) <= ON_FLOOR_Z
            and (tilt <= GLASSES_UPRIGHT_DEG or tilt >= 180.0 - GLASSES_UPRIGHT_DEG)
            and float(np.linalg.norm(g.lin_vel)) < STILL_V)


def lid_shut(lid) -> bool:
    return (math.hypot(lid.pos[0] - BOX_XY[0], lid.pos[1] - BOX_XY[1]) <= LID_SHUT_XY
            and tilt_deg(lid.quat_xyzw) <= LID_SHUT_DEG and abs(lid.pos[2] - LID_Z) <= 0.005)


def update_latches(o) -> None:
    """Whole-trial violations; once set they stay set until the next reset."""
    g, k, lid, mug = o["glasses"], o["keys"], o["lid"], o["mug"]
    checks = {
        "glasses_on_lenses_or_side": (GLASSES_FALLEN_DEG[0] < tilt_deg(g.quat_xyzw) < GLASSES_FALLEN_DEG[1]
                                      and float(np.linalg.norm(g.lin_vel)) < REST_V
                                      and float(np.linalg.norm(g.ang_vel)) < REST_W),
        "glasses_off_table": g.pos[2] < FLOOR_Z,
        "keys_in_box": any(inside_box(p) for p in corners(KEYS, k.pos, k.quat_xyzw)),
        "keys_off_table": k.pos[2] < FLOOR_Z,
        "lid_off_guides": (tilt_deg(lid.quat_xyzw) > LID_TILT_DEG or lid.pos[2] > LID_Z + LID_LIFT
                           or lid.pos[2] < LID_Z - 0.01),
        "mug_upset": (tilt_deg(mug.quat_xyzw) > MUG_TIP_DEG
                      or math.hypot(*(mug.pos[:2] - START["mug"][:2])) > MUG_MOVE),
    }
    for name, hit in checks.items():
        if hit and not LATCH.get(name):
            LATCH[name] = True
            print(f"[glasses-sliding-box] violation: {name}", flush=True)


def score(objects) -> float:
    """0 if at any time the glasses came to rest on their lenses or a side (60 to 120 degrees from standing) or left
    the table, any part of the keys went inside the
    box, the keys left the table, the lid left its guides (tilted past 10 degrees, lifted 1.5 cm, or dropped), or the
    mug tipped 15 degrees or moved 2 cm. Otherwise 0.2 for the keys in the dish, plus 0.4 for the glasses on the box
    floor, standing or upside down, plus 0.4 more if the lid is also shut over them."""
    update_latches(objects)
    if DEBUG:
        _debug(objects)
    if LATCH:
        return 0.0
    s = 0.2 if keys_in_dish(objects["keys"]) else 0.0
    if glasses_in_box(objects["glasses"]):
        s += 0.4 + (0.4 if lid_shut(objects["lid"]) else 0.0)
    return round(s, 2)


def success(objects) -> bool:
    return score(objects) == 1.0


# -- scripted demo: an open-loop plan from the fixed layout above (it reads nothing from the simulator) ----------------


def demo():
    from yam_arena.demo import Waypoint
    from yam_arena.constants import FINGER_CLOSED as C, FINGER_OPEN as O

    tip = 0.013  # pad tips below the TCP
    hz = T + 0.14
    hg = T + 0.17  # glasses hang 4.6 cm below the TCP: carry them clear of the guides
    push_z = T + BOX_H + 0.003 + tip  # finger tips 3 mm above the rim
    kx, ky = KEYS_START
    key_z = T + BOX_H + LID_T + 0.004 + tip
    dish_z = T + DISH_BASE + 0.010 + tip
    gx, gy = GLASSES_START
    grip_z = T + GLASSES_TOP - 0.010 + tip
    px, py = BOX_XY[0], INNER_Y[1] - 0.025  # front frame 2.5 cm inside the left-hand wall
    place_z = T + FLOOR + GLASSES_TOP - 0.010 + tip + 0.004
    lid_end = BOX_XY[1] + BOX_Y / 2  # the lid's left end, shut
    # Closed fingers, measured in a trial: the lid's end stops about 2 mm past the TCP when pushed toward -y and 8 mm
    # short of it when pushed toward +y.
    f_open, f_shut = -0.002, 0.008
    opened = 0.20  # lid centre 4 cm onto the block: the box's opening fully clear
    W = Waypoint
    left = [
        W(1.5, (kx, ky, hz), O, "to the keys"),
        W(1.5, (kx, ky, key_z), O, "descend"),
        W(0.8, (kx, ky, key_z), C, "grip the fob"),
        W(1.2, (kx, ky, hz), C, "lift"),
        W(1.8, (*DISH_XY, hz), C, "carry"),
        W(1.2, (*DISH_XY, dish_z), C, "lower"),
        W(0.6, (*DISH_XY, dish_z), O, "release"),
        W(1.0, (*DISH_XY, hz), O, "up"),
        W(1.2, (px, lid_end + 0.025, hz), C, "over the lid's end"),
        W(1.5, (px, lid_end + 0.025, push_z), C, "down beside it"),
        W(1.0, (px, lid_end - f_open, push_z), C, "touch"),
        W(4.0, (px, lid_end - f_open - opened, push_z), C, "slide the lid open"),
        W(1.0, (px, lid_end - f_open - opened, push_z + 0.04), C, "up from the lid"),
        W(1.5, (gx, gy, hz), O, "to the glasses"),
        W(1.5, (gx, gy, grip_z), O, "descend"),
        W(0.8, (gx, gy, grip_z), C, "grip the bridge"),
        W(1.5, (gx, gy, hg), C, "lift the glasses"),
        W(2.0, (px, py, hg), C, "carry the glasses"),
        W(2.0, (px, py, place_z), C, "lower"),
        W(0.8, (px, py, place_z), O, "release"),
        W(1.2, (px, py, hz), O, "up"),
        W(1.5, (0.22, 0.24, hz), O, "clear"),
    ]
    right_start = (px, lid_end - BOX_Y - opened + f_shut, push_z)  # at the open lid's right end
    wait = sum(w.duration for w in left) - 1.0
    right = [
        W(wait, (0.25, -0.24, hz), C, "wait"),
        W(1.5, (px, right_start[1] - 0.03, hz), C, "behind the open lid"),
        W(1.5, (px, right_start[1] - 0.03, push_z), C, "down"),
        W(1.0, (px, right_start[1] - 0.01, push_z), C, "touch"),
        W(4.0, (px, lid_end - BOX_Y + f_shut, push_z), C, "slide the lid shut"),
        W(1.0, (px, lid_end - BOX_Y + f_shut - 0.03, push_z), C, "back off"),
        W(1.2, (0.25, -0.24, hz), O, "clear"),
    ]
    return {"left": left, "right": right}
