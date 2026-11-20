"""On the station table, right of centre: a blue silicone soldering mat; lying on it, an orange PETG robot
bracket (80 x 40 x 12 mm, long side along y) with four brass M3 heat-set inserts standing in its four 4.0 mm holes,
each about 4.5 mm proud of the top face; beside it, a small aluminium block (19 x 19 x 50 mm) stands on its end. To
its right and further back, a soldering-iron stand clamped to the table: its holder tube slopes down away from the
arms, and the iron (switched on, 245 C, M3 heat-set tip) rests in it, its blue grip sticking up toward the arms.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from insertkit import N_INSERTS, InsertRig, insert_credit  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Set the four inserts flush and square into the bracket with the soldering iron and the flat block, "
               "then put the iron back in its stand.")

RIG = InsertRig("insert_rig", pos=(0.28, -0.08, TOP + 0.014), size=(0.04, 0.08, 0.012), table_z=TOP)
OBJECTS = [RIG]


def credits(r) -> list[float]:
    return [insert_credit(i) for i in r.inserts]


def score(objects) -> float:
    """0 if, at any time, a hot part of the iron (tip to the end of its barrel) touched the robot, or the iron was out
    of its stand and in no grip for 2 s (dropped or put down elsewhere), or if at the end the iron is not resting in its
    stand, released. Otherwise per insert: 0.25 flush (top within 0.5 mm of the bracket's face, either way) and pressed
    flat with the flat block while still soft (within 3 s of the iron leaving it), 0.10 seated but not both (top 0.5 to
    2.5 mm proud, sunk 0.5 to 1 mm below the face, or flush by the iron alone); 0 for an insert that is higher, or that
    was sunk with the iron or the block leaning more than 5 degrees (crooked), pushed in harder than 15 N (forced), or
    next to which the hot iron pressed on the bracket for 0.5 s (melt mark)."""
    r = objects["insert_rig"]
    if r.burned or r.dropped or not r.in_stand or r.held is not None:
        return 0.0
    return round(sum(credits(r)), 4)


def success(objects) -> bool:
    return score(objects) >= 0.25 * N_INSERTS - 1e-9


# -- scripted demo (closed loop; see insertdemo.py) ------------------------------------------------------------------


def controller(bases, dt):
    """yam_scripted plays this: it sees the camera images and the joint positions only."""
    import insertdemo

    return insertdemo.Controller(bases, dt, RIG)
