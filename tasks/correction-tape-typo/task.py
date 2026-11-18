"""On the station table, in front of the right arm: a printed A4 letter taped flat at its corners, with the typo
"teh" in its fifth line; to its right, a correction-tape dispenser lying flat, its applicator lip toward the sheet.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from tapekit import TapeRig  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Cover the typo 'teh' in the letter with the correction tape, without covering the words next to it."

# the sheet's top-left corner: the typo's centre lands at x 0.30, y -0.08
RIG = TapeRig("tape_rig", pos=(0.4639, -0.0313, TOP), size=(0.297, 0.210, 0.0002), table_z=TOP)
OBJECTS = [RIG]


def score(objects) -> float:
    """0 if the tape hides more than 1.5 mm of another word's length (or runs off the letter). Otherwise 0.6 x the
    share of the typo's ink box under tape (full at 95%), + 0.2 if that share is 90% or more and no other word is
    touched, + 0.2 if that share is 90% or more and the tape went on in one stroke without bunching."""
    return float(objects["tape_rig"].result["score"])


def success(objects) -> bool:
    """Full score and the dispenser lifted 3 cm or more off the paper."""
    return score(objects) >= 1.0 and objects["tape_rig"].pos[2] >= TOP + 0.03


def controller(bases, dt):
    """yam_scripted plays this: it sees the joint positions only (tapedemo.py)."""
    import tapedemo

    return tapedemo.Controller(bases, dt, RIG)
