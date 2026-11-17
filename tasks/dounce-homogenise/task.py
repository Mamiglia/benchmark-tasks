"""On a lab bench: a 7 mL glass Dounce tissue grinder stands in a polypropylene rack for 13 mm tubes, its tight
pestle resting in the mortar with the head on the bore's floor, under 3.0 to 3.4 mL of a cell-suspension stand-in
(water with 0.1% dish detergent). The robot homogenises the sample: ten full strokes of the pestle, the head kept
under the liquid on every upstroke, the mortar held still.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import dounce as D  # noqa: E402
from dkit import DounceGrinder, model  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Homogenise the sample in the Dounce with ten full strokes of the pestle: each stroke down to the "
               "bottom and back up, keeping the pestle's head under the liquid, and hold the mortar still.")

# -- layout (env frame: x forward, y left, z up; m) ------------------------------------------------------------------
XY = (0.30, 0.0)  # the mortar's axis
RACK_L, RACK_W, RACK_H, PLATE = 0.200, 0.102, 0.057, 0.003  # Nalgene Unwire rack for 13 mm tubes
HOLE = 0.014
PP = (0.95, 0.85, 0.20)  # yellow acetal
_x0, _y0 = XY
_c = (RACK_W - HOLE) / 4 + HOLE / 2  # centre of the plate strips beside the hole
_zt = TOP + RACK_H - PLATE / 2
RACK = [
    Box("rack_base", pos=(_x0, _y0, TOP + PLATE / 2), size=(RACK_W, RACK_L, PLATE), color=PP),
    Box("rack_end_l", pos=(_x0, _y0 + RACK_L / 2 - PLATE / 2, TOP + RACK_H / 2), size=(RACK_W, PLATE, RACK_H),
        color=PP),
    Box("rack_end_r", pos=(_x0, _y0 - RACK_L / 2 + PLATE / 2, TOP + RACK_H / 2), size=(RACK_W, PLATE, RACK_H),
        color=PP),
    # the top tier, with the mortar's hole: strips in front of and behind it, and to its left and right
    Box("tier_front", pos=(_x0 + _c, _y0, _zt), size=((RACK_W - HOLE) / 2, RACK_L, PLATE), color=PP),
    Box("tier_back", pos=(_x0 - _c, _y0, _zt), size=((RACK_W - HOLE) / 2, RACK_L, PLATE), color=PP),
    Box("tier_left", pos=(_x0, _y0 + (RACK_L + HOLE) / 4, _zt), size=(HOLE, (RACK_L - HOLE) / 2, PLATE), color=PP),
    Box("tier_right", pos=(_x0, _y0 - (RACK_L + HOLE) / 4, _zt), size=(HOLE, (RACK_L - HOLE) / 2, PLATE), color=PP),
]
SEAT_Z = TOP + PLATE  # the mortar's bottom rests on the rack's base
GRINDER = DounceGrinder("dounce", pos=(*XY, SEAT_Z), size=(0.02, 0.02, 0.11))
OBJECTS = [*RACK, GRINDER]


# -- scoring ---------------------------------------------------------------------------------------------------------


def score(objects) -> float:
    """Latched over the whole trial, any one gives 0: the mortar rose more than 5 mm off its seat, tipped past
    20 degrees, or the pestle's head was pulled up out of the grinding tube. Otherwise 0.08 per full stroke (the
    head within 3 mm of the floor, then at least 25 mm above it, then back to the floor), counted up to ten; plus
    0.2 if the foam on the sample is under 2 mm at the end; minus 0.05 for every upstroke on which the head rose
    above the liquid and drew air in; floored at 0."""
    return D.score(objects["dounce"].model)


def success(objects) -> bool:
    return score(objects) == 1.0


def controller(bases, dt):
    """yam_scripted plays this (see ddemo.py)."""
    import ddemo

    return ddemo.Controller(bases, dt)


__all__ = ["INSTRUCTION", "OBJECTS", "SCENE", "controller", "model", "score", "success"]
