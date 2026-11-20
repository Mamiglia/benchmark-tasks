"""On the table in front of the arms: a BRIO Labyrinth tilting maze game (wooden box 35 x 30.5 x 9.4 cm, two knobs)
standing on a 9.5 cm wooden riser so that its knobs are at the arms' working height. The box is turned 45 degrees: the
knob on its long side faces the left arm, the knob on its short end faces the right arm. Inside, the playing board
is BRIO's 8-hole practice board (BRIO Labyrinth Boards, 34030): black partitions, 8 numbered holes and a printed
path from START (an arrow) to FINISH (a star); the steel ball rests at START. Turning a knob tilts the board
about one axis (1 degree per 25 degrees of knob turn, up to 4 degrees each way).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import labmodel as M  # noqa: E402
from labkit import Labyrinth, SteelBall  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.5
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Turn the labyrinth's two knobs to roll the steel ball from START along the numbered path, past every "
               "hole to FINISH, without letting it drop into a hole.")

CENTRE = (0.42, 0.0)
YAW = -3 * math.pi / 4  # the box's long axis (x) points back toward the right arm's side
RISER_H = 0.095
LAB = Labyrinth("labyrinth", pos=(*CENTRE, TOP + RISER_H), size=M.BOX, yaw=YAW, color=(0.8, 0.62, 0.4))
RISER = Box("riser", pos=(*CENTRE, TOP + RISER_H / 2), size=(0.37, 0.32, RISER_H), yaw=YAW, color=(0.55, 0.52, 0.48))
BALL = SteelBall("ball", pos=(*CENTRE, TOP + 0.2), size=(0.02, 0.02, 0.02),
                 start=tuple(LAB.to_world((*M.START, M.BOARD_TOP + M.BALL_R + 0.0005))))
OBJECTS = [RISER, LAB, BALL]


def knob_grip(which: str):
    """(grip point, outward axis) of a knob in the world: 'x' (right arm) or 'y' (left arm)."""
    L, W, _ = M.BOX
    c, s = math.cos(YAW), math.sin(YAW)
    if which == "x":
        p, n = (L / 2 + M.GRIP_OUT, 0.0, M.KNOB_Z), (1.0, 0.0)
    else:
        p, n = (0.0, -W / 2 - M.GRIP_OUT, M.KNOB_Z), (0.0, -1.0)
    return LAB.to_world(p), (c * n[0] - s * n[1], s * n[0] + c * n[1], 0.0)


def score(objects) -> float:
    """0 after a foul (latched for the whole trial: a gripper inside the box or at the ball, the box lifted 1 cm,
    tipped 5 degrees or slid 5 cm). Otherwise (holes the ball has rolled past along the path + 1 at FINISH) / 9
    (latched: the furthest point counts; a ball that drops into a hole keeps the holes it had passed)."""
    st = objects["labyrinth"]
    return M.score_value(st.passed, bool(st.fouls))


def success(objects) -> bool:
    return score(objects) == 1.0


def controller(bases, dt):
    """yam_scripted plays this closed-loop controller (labdemo.py)."""
    import labdemo

    return labdemo.Controller(bases, dt)
