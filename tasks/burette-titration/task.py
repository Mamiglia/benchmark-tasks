"""On a student chemistry bench: a cast-iron retort stand holds a 25 mL glass burette (filled to 0.00 mL with
0.05 M sodium hydroxide) whose PTFE stopcock key has its white winged handle facing the left arm, 17.5 cm above the
table. Under its tip, on a white tile on the stand's base, stands a 125 mL Erlenmeyer flask holding an unknown
aliquot (8 to 12 mL) of 0.05 M acetic acid made up to 30 mL with water and 3 drops of phenolphthalein; the tip is
1 cm inside the flask's neck.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from titration import TitrationParams  # noqa: E402
from titrkit import FLASK_RINGS, IRON, TILE, TitrationRig  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Titrate the acid in the flask with the burette to the first permanent pink: swirl the flask while "
               "you add the titrant, and close the stopcock as soon as the pink colour stays.")

# -- layout (env frame: x forward, y left, z up; m) ------------------------------------------------------------------
BUR_XY = (0.30, 0.0)  # burette axis
PLATE_T, TILE_T = 0.010, 0.006
FLOOR = TOP + PLATE_T + TILE_T  # the tile's top: the flask's bottom stands here
RIM = FLOOR + FLASK_RINGS[-1][1]  # 13.0 cm above the table
TIP_Z = RIM - 0.010  # tip 1 cm inside the neck
COCK_Z = TOP + 0.175  # stopcock axis
KEY_YAW = math.radians(-35.0)  # the key's axis, from its handle into the barrel: toward the front right
KEY_DIR = (math.cos(KEY_YAW), math.sin(KEY_YAW), 0.0)
ROD_XY = (0.44, 0.0)

STAND = (
    ((0.345, 0.0, TOP + PLATE_T / 2), (0.25, 0.16, PLATE_T), IRON),  # cast-iron base plate 25 x 16 cm
    ((*BUR_XY, TOP + PLATE_T + TILE_T / 2), (0.152, 0.152, TILE_T), TILE),  # white ceramic tile 6 x 6 in
)
CLAMP_Z = TOP + 0.45
CLAMP = (
    (((BUR_XY[0] + ROD_XY[0]) / 2, 0.0, CLAMP_Z), (ROD_XY[0] - BUR_XY[0], 0.012, 0.012), (0.55, 0.57, 0.6)),
    ((ROD_XY[0], 0.0, CLAMP_Z), (0.03, 0.03, 0.03), (0.30, 0.32, 0.35)),  # boss head
    ((BUR_XY[0], 0.0, CLAMP_Z), (0.024, 0.030, 0.020), (0.15, 0.15, 0.15)),  # rubber-lined jaws
)

RIG = TitrationRig("titration", pos=(*BUR_XY, TOP + 0.3), size=(0.25, 0.16, 0.6), tip_z=TIP_Z, cock_z=COCK_Z,
                   key_dir=KEY_DIR, top_z=TIP_Z + 0.62, flask_xy=BUR_XY, floor_z=FLOOR, stand_boxes=STAND,
                   rod=(*ROD_XY, TOP + PLATE_T, TOP + 0.70, 0.0064), clamp_boxes=CLAMP,
                   titration=TitrationParams())
OBJECTS = [RIG]

# -- scoring ---------------------------------------------------------------------------------------------------------
FULL_OVER = 0.30  # mL past the equivalence volume: about 6 drops
PART_OVER = 1.00
FAR_OVER = 3.00
NEAR_SHORT = 1.00


def score(objects) -> float:
    """End of the trial. 0 if, at any time, the flask's glass touched the burette tip, the flask tipped past 45
    degrees, any titrant fell outside the flask, or the burette ran empty; 0 if the stopcock is still letting
    titrant through at the end. Otherwise, from the burette reading T against the equivalence volume V_e and the
    flask's colour once mixed: pink and T - V_e <= 0.30 mL: 1.0; pink, <= 1.0 mL over: 0.6; pink, <= 3.0 mL over:
    0.3; further over: 0. Still colourless: 0.3 within 1.0 mL short (stopped on a fading flash), 0.1 with at least
    half of V_e added, else 0."""
    s = objects["titration"]
    if s.struck or s.spilled or s.lost > 0 or s.ran_dry or s.flow > 0:
        return 0.0
    e = s.reading - s.v_e
    if s.final_pink:
        return 1.0 if e <= FULL_OVER else 0.6 if e <= PART_OVER else 0.3 if e <= FAR_OVER else 0.0
    return 0.3 if e >= -NEAR_SHORT else 0.1 if s.reading >= 0.5 * s.v_e else 0.0


def success(objects) -> bool:
    return score(objects) == 1.0


# -- scripted demo (closed loop; see titrdemo.py) ---------------------------------------------------------------------


def controller(bases, dt):
    """yam_scripted plays this: it sees the camera images and the joint positions only."""
    import titrdemo

    return titrdemo.Controller(bases, dt)
