"""On the station table, in front of the arms: a sealed 3.5 x 9 in paper/film peel pouch holding 14 cm stainless
dressing forceps lies flat, paper up, on a 3 cm block at x 24 cm, its length along y and its chevron end at y 0; the
paper tab beyond the chevron is turned up and the film tab hangs over the block's end. Beyond it a blue
18 x 26 in sterile towel drape lies flat, centred at x 52 cm, y 0. Same layout in every trial.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import pouchkit as K  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = K.TOP
INSTRUCTION = ("Open the sterile pouch and drop the forceps onto the sterile field without touching the forceps "
               "or the drape.")

RIG = K.PouchRig("pouch_rig", pos=(0.40, 0.0, TOP + 0.12), size=(0.50, 0.66, 0.24))
OBJECTS = [RIG]

# -- scoring ---------------------------------------------------------------------------------------------------------
ON_DRAPE_M = 0.001  # the forceps' lowest point within 1 mm of the drape's top: lying on the drape itself
STILL_V = 0.02  # m/s
HALF_OPEN_MM = 0.5 * K.N * K.STRIP_L * 1000  # half the seal's length


def forceps_on_drape(s) -> bool:
    """The forceps lies on the drape itself (lowest point within 1 mm of its top, so not on the pouch), every blade
    corner over the drape, and still."""
    pts = s.forceps_pts
    return bool(pts[:, 2].min() <= K.DRAPE_TOP + ON_DRAPE_M and all(K.over_drape(p) for p in pts)
                and float(np.linalg.norm(s.lin_vel)) < STILL_V)


def inside_border(s) -> bool:
    """Every blade corner at least 2.5 cm in from the drape's edges (the field's non-sterile border)."""
    return all(K.over_drape(p, K.BORDER) for p in s.forceps_pts)


def score(objects) -> float:
    """0 for the rest of the trial once a gripper touched the forceps (pad tip or tool point within 8 mm) or the drape
    (within 1 cm of its top, over it). Otherwise, with the forceps lying on the drape itself and still: 0.75 (success,
    1.0 once held 1 s) inside the 2.5 cm border and the pouch never touched the drape; 0.5 partly on the border and the
    pouch never touched the drape; 0.25 the pouch touched the drape. Forceps not on the drape: 0.2 when the whole seal
    is open, 0.1 when at least half of it is open from the chevron down, else 0."""
    s = objects["pouch_rig"]
    if s.touched_forceps or s.touched_drape:
        return 0.0
    if forceps_on_drape(s):
        if s.pouch_on_drape:
            return 0.25
        return 0.75 if inside_border(s) else 0.5
    if all(s.parted):
        return 0.2
    return 0.1 if s.peeled_mm >= HALF_OPEN_MM else 0.0


def success(objects) -> bool:
    return score(objects) == 0.75


# -- scripted demo (see peeldemo.py) ---------------------------------------------------------------------------------


def controller(bases, dt):
    """yam_scripted plays this; it plans from the fixed layout and does not read the simulator."""
    import peeldemo

    return peeldemo.Controller(bases, dt)
