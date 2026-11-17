"""On the station table: a planting bed (plywood under brown landscape fabric). A 1/2 in poly drip line lies across it
along y, held down by two wire tubing stakes, with a white paint-pen ring marking where a shrub's dripper goes; the
shrub stands in its nursery pot just behind the line at the mark. On the bed to the right, a drilled pine block holds
the emitter punch (point down in its hole) and a barbed pressure-compensating dripper (barb down in its hole).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from dripkit import DripBed, score_line  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Install the dripper in the drip line at the white mark, then put the punch back in its block."

BED = DripBed("drip_bed", pos=(0.33, -0.02, TOP + 0.006), size=(0.30, 0.64, 0.012), table_z=TOP)
OBJECTS = [BED]


def score(objects) -> float:
    """0.4 a round hole punched within 10 mm of the mark (along the line); +0.5 the dripper seated in that hole (barb
    through the wall, collar on the tube); +0.1 then the punch standing back in its block hole, let go. Any torn hole
    (punched more than 15 degrees off square, or the dripper forced in more than 15 degrees off square) or a second
    hole anywhere caps the score at 0.2."""
    st = objects["drip_bed"]
    return score_line(st.line, st.punch_home)


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def controller(bases, dt):
    """yam_scripted plays this (see dripdemo.py)."""
    import dripdemo

    return dripdemo.Controller(bases, dt, BED)
