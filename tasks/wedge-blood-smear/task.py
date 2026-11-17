"""On the station table, in front of the right arm: a sheet of white bench paper taped down; on it a plain glass
microscope slide (75 x 25 x 1 mm, frosted end toward the arms) with a 5 uL drop of the blood stand-in 24 mm from that
end. To its right, a black slotted stand holds a second, clean slide upright on its short end: the spreader.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from smearkit import SmearRig  # noqa: E402
from smearmodel import smear_score  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Make a wedge smear of the drop on the slide with the spreader slide: let the drop run along the "
               "spreader's edge, then push forward in one smooth stroke. The smear should be 30-45 mm long and end "
               "in a feathered edge before the slide does.")

RIG = SmearRig("smear_rig", pos=(0.2775, -0.08, TOP), size=(0.075, 0.025, 0.002), table_z=TOP)
OBJECTS = [RIG]


def score(objects) -> float:
    """0: no film 15 mm long, a restart (put down again or moved back over the smear), or the base slide pushed over
    10 mm. 0.3: any film 15 mm or longer, or the drop ploughed ahead of the spreader, or the base slide moved 3-10
    mm. 0.6: one continuous film starting within 5 mm of the drop, 20-55 mm long, ending in a feathered edge on the
    slide. 1.0: that, 30-45 mm long, 15 mm or more wide, no hesitation ridge, its feathered end 5 mm or more before
    the slide's far end."""
    smear, moved = objects["smear_rig"].score_in
    return smear_score(smear, moved)


def success(objects) -> bool:
    """Full score, and the spreader taken away: its edge 3 cm or more above the table and 10 cm or more from the base
    slide's centre (the smear is finished and in view)."""
    r = objects["smear_rig"]
    away = float(np.linalg.norm((r.spreader_pos - r.pos)[:2])) >= 0.10
    return score(objects) >= 1.0 and r.spreader_pos[2] >= TOP + 0.03 and away


def controller(bases, dt):
    """yam_scripted plays this: it sees the joint positions only (smeardemo.py)."""
    import smeardemo

    return smeardemo.Controller(bases, dt, RIG)
