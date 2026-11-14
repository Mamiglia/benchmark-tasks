"""On a school science bench, in front of the arms: a green plastic test-tube rack at x 26 cm, its top board 7 cm
high, holds two open 16 x 100 mm glass test tubes in square holes, one at y +8 cm (the left arm's side) and one at
y -8 cm. Each tube is about a quarter full: 2 mL of dark blue 0.5 M copper(II) sulfate solution with 2 mL of clear
water layered on top, so each shows two layers. The tubes have no stoppers. Nothing else is on the table.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

# Idea first proposed in robocurve/benchmark-tasks#1433; independent implementation.

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mixkit import TubeRack, surface_gap  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Mix the solution in each test tube, then stand the tubes back in the rack."

# -- layout (env frame: x forward, y left, z up; m) ------------------------------------------------------------------
RACK_X = 0.26
HOLES_Y = (0.08, -0.08)
HOLE = 0.025  # square holes, 25 mm (the rack takes tubes up to 20 mm)
BASE_T, BOARD_Z, BOARD_T = 0.012, 0.060, 0.010  # base board thickness; top board's underside height and thickness
DEPTH_X, WIDTH_Y = 0.06, 0.24  # rack footprint
WOOD, WOOD_DARK = (0.16, 0.56, 0.30), (0.11, 0.42, 0.22)  # green polypropylene rack
FLOOR = TOP + BASE_T  # a racked tube's bottom stands here


def _rack_boxes():
    zb = TOP + BOARD_Z + BOARD_T / 2
    h = HOLE / 2
    x0, x1 = RACK_X - DEPTH_X / 2, RACK_X + DEPTH_X / 2
    yl, yr = HOLES_Y
    boxes = [((RACK_X, 0.0, TOP + BASE_T / 2), (DEPTH_X, WIDTH_Y, BASE_T), WOOD_DARK),  # base board
             ((RACK_X, 0.0, zb), (DEPTH_X, 2 * (yl - h), BOARD_T), WOOD)]  # top board between the holes
    for y in HOLES_Y:
        s = 1 if y > 0 else -1
        outer0, outer1 = abs(y) + h, WIDTH_Y / 2
        boxes.append(((RACK_X, s * (outer0 + outer1) / 2, zb), (DEPTH_X, outer1 - outer0, BOARD_T), WOOD))
        for xa, xb in ((x0, RACK_X - h), (RACK_X + h, x1)):  # strips in front of and behind the hole
            boxes.append((((xa + xb) / 2, y, zb), (xb - xa, HOLE, BOARD_T), WOOD))
        post_h = BOARD_Z - BASE_T
        boxes.append(((RACK_X, s * (WIDTH_Y / 2 - 0.006), TOP + BASE_T + post_h / 2), (DEPTH_X, 0.012, post_h),
                      WOOD))  # end post
    return tuple(boxes)


RIG = TubeRack("tube_rack", pos=(RACK_X, 0.0, TOP + 0.05), size=(DEPTH_X, WIDTH_Y, 0.1),
               tube_xy=tuple((RACK_X, y) for y in HOLES_Y), rack_boxes=_rack_boxes(), floor_z=FLOOR, table_z=TOP)
OBJECTS = [RIG]

# -- scoring ---------------------------------------------------------------------------------------------------------
RACKED_TILT = 20.0  # deg from upright (a 16 mm tube can lean up to about 16 degrees in a 25 mm hole)
RACKED_XY = 0.006  # where the tube's axis passes through the top board: within 6 mm of the hole's centre
RACKED_Z = 0.008  # bottom within 8 mm above the base board
BOARD_MID = TOP + BOARD_Z + BOARD_T / 2
CLEAR_M = 0.03  # every finger pad and tool point 3 cm or more from each tube
STILL_V = 0.02  # m/s


def racked(t, k: int) -> bool:
    """Tube standing in its own hole: it passes through the hole (its axis crosses the top board within 6 mm of the
    hole's centre), its bottom is on the base board, and it leans 20 degrees or less."""
    if t.tilt_deg >= RACKED_TILT or not FLOOR - 0.003 < t.bottom[2] < FLOOR + RACKED_Z:
        return False
    p = t.bottom + t.axis * (BOARD_MID - t.bottom[2]) / t.axis[2]
    return abs(p[0] - RACK_X) < RACKED_XY and abs(p[1] - HOLES_Y[k]) < RACKED_XY


def released(r) -> bool:
    """No finger pad or tool point within 3 cm of either tube."""
    return all(surface_gap(p, t.pos, t.axis, RIG.tube_len, RIG.tube_radius) >= CLEAR_M
               for t in r.tubes for pads in r.pads.values() for p in pads)


def score(objects) -> float:
    """0 for the rest of the trial once liquid reaches the mouth of either tube (spilled). Otherwise: 0.25 for each
    tube whose layers are mixed; 0.5 when both are mixed and both stand in their holes but a hand is still on or near
    a tube; 0.75 both mixed, both racked, the hands 3 cm clear and the tubes still (success, 1.0, once held 1 s)."""
    r = objects["tube_rack"]
    if any(t.spilled for t in r.tubes):
        return 0.0
    mixed = sum(t.mixed for t in r.tubes)
    if mixed == len(r.tubes) and all(racked(t, k) for k, t in enumerate(r.tubes)):
        still = all(float(np.linalg.norm(t.lin_vel)) < STILL_V for t in r.tubes)
        return 0.75 if released(r) and still else 0.5
    return 0.25 * mixed


def success(objects) -> bool:
    return score(objects) == 0.75


# -- scripted demo (closed loop; see mixdemo.py) ---------------------------------------------------------------------


def controller(bases, dt):
    """yam_scripted plays this: it sees the camera images and the joint positions only."""
    import mixdemo

    return mixdemo.Controller(bases, dt)
