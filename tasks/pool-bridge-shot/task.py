"""On the station table: a 40 in tabletop pool table (blue cloth, black rails, six pockets), its long side along y,
centred in front of the arms. On the cloth, a white cue ball and a yellow object ball (38 mm). The near-left corner
pocket (front left, nearest the left arm) has white tape on its rails. A 36 in cue lies along the near rail, tip to the
left.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from poolkit import Ball, BallParams, Cue, CueParams, Rules, Table, TableGeom, configure, monitor, shot_score  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("Pot the yellow ball in the corner pocket marked with white tape, striking only the white ball, "
               "once, with the cue.")

GEOM = TableGeom(top=TOP)
BALL_P = BallParams()
CUE_P = CueParams()
RULES = Rules(target_pocket="near_left")
R = BALL_P.radius
BALL_Z = GEOM.felt + R

# -- layout (world: x forward from the arms, y left; metres) ---------------------------------------------------------
CB_XY = (0.250, 0.150)  # cue ball (spot sticker)
OB_XY = (0.230, 0.325)  # object ball (spot sticker): a 32 degree cut into the near-left corner pocket
RAIL_X = GEOM.x0 - GEOM.cushion_w - GEOM.rail_w / 2  # near rail centre line
RAIL_TOP = GEOM.felt + GEOM.rail_h
CUE_Y = -0.25  # the cue's balance point, lying on the near rail, tip toward +y
CUE_POS = (RAIL_X, CUE_Y, RAIL_TOP + CUE_P.butt_r + 0.0005)

_g = GEOM
_cx = (_g.x0 + _g.x1) / 2
_ow = _g.depth + 2 * (_g.cushion_w + _g.rail_w)
_ol = _g.length + 2 * (_g.cushion_w + _g.rail_w)
TABLE = Table("pool_table", pos=(_cx, 0.0, TOP + _g.felt_h / 2), size=(_ow, _ol, _g.felt_h + _g.rail_h), geom=GEOM)
CUE_BALL = Ball("cue_ball", pos=(*CB_XY, BALL_Z + 0.0005), radius=R, mass=BALL_P.mass, color=(0.96, 0.96, 0.93),
                params=BALL_P)
OBJECT_BALL = Ball("object_ball", pos=(*OB_XY, BALL_Z + 0.0005), radius=R, mass=BALL_P.mass, color=(0.98, 0.80, 0.05),
                   params=BALL_P)
CUE = Cue("cue", pos=CUE_POS, size=(CUE_P.length, 0.025, 0.025), mass=CUE_P.mass, yaw=math.pi / 2, params=CUE_P)
# white tape on the rails either side of the target pocket (19 mm tape, visual only: it sits on the rail tops)
_t = 0.0015
TAPE = [
    Box("tape_near", pos=(RAIL_X, _g.y1 - 0.02, RAIL_TOP + _t / 2), size=(_g.rail_w, 0.04, _t), color=(1.0, 1.0, 1.0)),
    Box("tape_end", pos=(_g.x0 + 0.02, _g.y1 + _g.cushion_w + _g.rail_w / 2, RAIL_TOP + _t / 2),
        size=(0.04, _g.rail_w, _t), color=(1.0, 1.0, 1.0)),
]
OBJECTS = [TABLE, CUE_BALL, OBJECT_BALL, CUE, *TAPE]

configure(TABLE, CUE_BALL, OBJECT_BALL, CUE, RULES)


# -- scoring ---------------------------------------------------------------------------------------------------------


def score(objects) -> float:
    """See poolkit.shot_score: 1.0 potted in the taped pocket with one clean strike and no scratch; 0.5 potted
    elsewhere or at rest within 10 cm of the taped pocket; 0.25 the object ball hit; scratch caps at 0.25; 0 for no
    strike, two strikes, a push or a foul."""
    ob = objects["object_ball"]
    return shot_score(monitor(), ob.pos, ob.resting())


def success(objects) -> bool:
    cb = objects["cue_ball"]
    return score(objects) == 1.0 and (cb.resting() or bool(getattr(cb, "pocket", "")))


# -- scripted demo -----------------------------------------------------------------------------------------------------


def demo_controller(bases, dt):
    from shot_demo import Controller

    return Controller(bases, dt, cue_pos=CUE_POS, cue=CUE_P, cb_xy=CB_XY, ob_xy=OB_XY, ball_z=BALL_Z, radius=R,
                      pocket=GEOM.pocket_mouth(RULES.target_pocket), felt=GEOM.felt,
                      mode=os.environ.get("DEMO_MODE", ""))


def demo():
    """Placeholder so tools that look for a demo() find one; yam_scripted runs demo_controller instead."""
    return {}
