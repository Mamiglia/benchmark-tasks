"""On the table in front of the arms: a 40 in tabletop football (foosball) table, 100 x 50 cm, clamped to the
bench with its long side toward the arms. Eight steel rods run across it (toward the arms) carrying plastic figures;
the robot's team (red) has its handles on the near side: goalie, 2-bar, 5-bar (midfield) and 3-bar (attack). The
left arm's handle is the red 5-bar, the right arm's the red 3-bar. The blue team's four rods are clamped with their
figures hanging straight down. The white ball rests on the marked spot 3.5 cm in front of the red 5-bar. The goal to
score in is at the right-hand end (y -).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import tfmodel as M  # noqa: E402
from tfkit import Ball, FootballTable  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.5
assert abs(TABLE_TOP[SCENE] - M.TOP) < 1e-9
INSTRUCTION = ("Using only the two rod handles in front of you, pass the ball from the midfield rod to the attack rod, "
               "stop it there, and shoot it into the far goal. Do not spin a rod more than one full turn.")

_LO = (M.X_NEAR - M.WALL_T, M.Y_GOAL - M.WALL_T, M.TOP)
_HI = (M.X_FAR + M.WALL_T, M.Y_OWN + M.WALL_T, M.WALL_TOP)
TABLE = FootballTable("football_table", pos=tuple((a + b) / 2 for a, b in zip(_LO, _HI)),
                      size=tuple(b - a for a, b in zip(_LO, _HI)), color=(0.2, 0.2, 0.22))
BALL = Ball("ball", pos=M.BALL_START, size=(0.031, 0.031, 0.031), color=(0.97, 0.97, 0.95))
OBJECTS = [TABLE, BALL]


def score(objects) -> float:
    """0 after a foul (latched for the whole trial: a gripper finger or tool point within 1 cm of the ball or inside
    the table, the ball off the table, an own goal, a rod turned more than one full turn). Otherwise the furthest
    stage reached (latched): 0.1 a 5-bar figure has struck the ball past the blue 5-bar; 0.3 a red 3-bar figure has
    then touched it; 0.5 it has then been at rest against or under a 3-bar figure for 1 s (trapped); 0.7 goal, last
    touched by the 3-bar, not trapped first; 1.0 goal shot by the 3-bar after the trap."""
    return objects["football_table"].score


def success(objects) -> bool:
    return score(objects) == 1.0


def controller(bases, dt):
    """yam_scripted plays this closed-loop controller (tfdemo.py)."""
    import tfdemo

    return tfdemo.Controller(bases, dt)
