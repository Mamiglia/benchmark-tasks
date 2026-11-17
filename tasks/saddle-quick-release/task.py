"""On the station table: a red 12-inch children's balance bike stands upright along the table, front wheel away from
the arms, its front wheel in a slot-type floor stand. Its left side faces the left arm. The saddle sits low and is
turned about 12 degrees out of line with the frame. A band of blue tape is wrapped round the seat post just above
the seat clamp. The clamp is a quick-release collar: a red lever on the bike's left side, folded down along the
seat tube when closed. With the lever open the post slides and turns freely in the frame and sinks under its own
weight; closed past its dead point, the lever stays shut and the collar holds the post.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from bikekit import Slide, frame_x, on_physics, on_reset, on_step, part, robot_hands, set_efforts  # noqa: E402
from qrmodel import (A, COLLAR_TOP, E0, LEVER_LEN, MIN_INSERT_M, NOSE, PIVOT, POST_D, POST_LEN, PT, REAR, TOP,  # noqa: E402
                     TWIST0_DEG, Y, Clamp, ClampParams, paddle, post_point, score_state)

SCENE = "station"
SCORE_OVER = "end"
INSTRUCTION = ("Raise the saddle until the bottom of the blue tape on the seat post is 4 cm above the seat clamp, "
               "with the saddle straight, and close the quick-release lever.")

RED, BLACK, GREY, SILVER = (0.78, 0.08, 0.07), (0.06, 0.06, 0.07), (0.45, 0.46, 0.48), (0.72, 0.73, 0.75)
LEVER_RED, TAPE_BLUE, STAND_C = (0.95, 0.20, 0.10), (0.15, 0.45, 0.95), (0.20, 0.21, 0.23)
R_Y = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # a "cyl" spec along y


def seg(p0, p1, d, color, collide=True, origin=(0.0, 0.0, 0.0)):
    """A cylinder of diameter ``d`` from p0 to p1 (world), as a spec relative to ``origin``."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    c = (p0 + p1) / 2 - np.asarray(origin, float)
    return (tuple(c), (float(np.linalg.norm(p1 - p0)), d, d), color, collide, "cyl", frame_x(p1 - p0))


def ybar(x, z, y0, y1, d, color, collide=True):
    return seg((x, y0, z), (x, y1, z), d, color, collide)


# -- the bike frame (fixed root body, world coordinates) -----------------------------------------------------------
R_HUB = np.array([0.0, 0.0, TOP + 0.151])
F_HUB = np.array([0.56, 0.0, TOP + 0.151])
HT_B, HT_T = np.array([0.52, 0.0, TOP + 0.27]), np.array([0.50, 0.0, TOP + 0.36])
BAR = np.array([0.49, 0.0, TOP + 0.42])
ST_BOT = COLLAR_TOP - 0.16 * A
FRAME = [
    seg(ST_BOT, COLLAR_TOP - 0.018 * A, 0.028, RED),  # seat tube
    seg(COLLAR_TOP - 0.018 * A, COLLAR_TOP, 0.034, BLACK),  # quick-release collar
    (tuple(COLLAR_TOP - 0.009 * A + 0.020 * REAR), (0.016, 0.034, 0.014), BLACK, True, "box", frame_x(A)),  # ears
    ybar(PIVOT[0], PIVOT[2], -0.027, -0.019, 0.011, SILVER),  # adjusting nut, right side
    ybar(PIVOT[0], PIVOT[2], -0.019, 0.019, 0.005, SILVER, False),  # bolt through the ears
    seg(COLLAR_TOP - 0.12 * A, HT_B, 0.040, RED),  # main tube
    seg(HT_B, HT_T, 0.036, RED),  # head tube
    *[seg(HT_B + (0, s * 0.03, 0), F_HUB + (0, s * 0.03, 0), 0.018, RED) for s in (-1, 1)],  # fork
    *[seg(ST_BOT + (0, s * 0.03, 0), R_HUB + (0, s * 0.03, 0), 0.018, RED) for s in (-1, 1)],  # rear stays
    seg(HT_T, BAR, 0.025, SILVER),  # stem
    ybar(BAR[0], BAR[2], -0.20, 0.20, 0.022, SILVER),  # handlebar
    *[ybar(BAR[0], BAR[2], s * 0.12, s * 0.21, 0.030, BLACK) for s in (-1, 1)],  # grips
]
for hub in (R_HUB, F_HUB):
    FRAME += [ybar(hub[0], hub[2], -0.025, 0.025, 0.30, BLACK),  # 12 1/2 x 2.25 in tyre
              ybar(hub[0], hub[2], -0.026, 0.026, 0.20, GREY, False),  # rim, drawn only
              ybar(hub[0], hub[2], -0.045, 0.045, 0.03, SILVER, False)]  # hub
STAND = [  # slot-type floor stand round the front wheel's bottom
    ((F_HUB[0], 0.0, TOP + 0.004), (0.30, 0.16, 0.008), STAND_C),
    *[((F_HUB[0], s * 0.034, TOP + 0.06), (0.22, 0.008, 0.11), STAND_C) for s in (-1, 1)],
]
FRAME += STAND

# -- the post and saddle (child bodies; specs relative to COLLAR_TOP at slide 0, turn 0) ----------------------------
O = np.zeros(3)
POST_BOXES = (
    seg(PT - POST_LEN * A, PT, POST_D, SILVER, True, COLLAR_TOP - COLLAR_TOP),  # the post
    seg(-0.0005 * A, 0.012 * A, POST_D + 0.0012, TAPE_BLUE, False),  # tape band, lower edge at the collar
    seg(-MIN_INSERT_M * A - 0.002 * A, -MIN_INSERT_M * A, POST_D + 0.0006, BLACK, False),  # MIN INSERT line
    (tuple(PT + (0.0, 0.0, 0.004)), (0.05, 0.034, 0.016), BLACK),  # saddle's clamp shell
    (tuple(PT + (-0.015, 0.0, 0.0200)), (0.11, 0.125, 0.030), BLACK),  # saddle, rear
    (tuple(PT + (0.075, 0.0, 0.0225)), (0.09, NOSE_W := 0.05, 0.025), BLACK),  # saddle, nose
)
SLIDE = Slide(name="post_slide", origin=tuple(COLLAR_TOP), axis=tuple(A),
              boxes=((tuple(-0.10 * A), (0.005, 0.005, 0.005), GREY),), lower=-0.05, upper=0.12, stiffness=0.0,
              damping=15.0, mass=0.02)
TURN = Slide(name="post_turn", origin=tuple(COLLAR_TOP), axis=tuple(A), boxes=POST_BOXES, lower=-math.radians(170),
             upper=math.radians(170), stiffness=0.0, damping=0.002, mass=0.30, revolute=True, parent="post_slide",
             init=math.radians(TWIST0_DEG), armature=0.002)
LEVER_BOXES = (
    ((0.0, 0.0, 0.0), (0.008, 0.016, 0.016), LEVER_RED, True, "cyl", R_Y),  # cam head
    (tuple(0.026 * -A + (0, 0.002, 0)), (0.046, 0.005, 0.011), LEVER_RED, True, "box", frame_x(A)),  # arm
    (tuple(LEVER_LEN * -A + (0, 0.003, 0)), (0.020, 0.007, 0.016), LEVER_RED, True, "box", frame_x(A)),  # paddle
)
LEVER = Slide(name="lever", origin=tuple(PIVOT), axis=(0.0, 1.0, 0.0), boxes=LEVER_BOXES, lower=0.0,
              upper=math.radians(120), stiffness=0.0, damping=0.002, mass=0.02, revolute=True, armature=2e-4)
BIKE = part("bike", (0.0, 0.0, 0.0), FRAME, mass=3.0, color=RED, friction=0.9, slides=(SLIDE, TURN, LEVER),
            fixed=True)
OBJECTS = [BIKE]

# -- run-time mechanics ------------------------------------------------------------------------------------------
PARAMS = ClampParams()
CLAMP = Clamp(PARAMS)
_RT: dict = {}
DEBUG = bool(os.environ.get("SADDLE_DEBUG"))


def _reset(environment=None) -> None:
    CLAMP.reset()
    _RT.update(steps=0, theta=0.0, slide=0.0, turn=math.radians(TWIST0_DEG), vel=(0.0, 0.0, 0.0), failed="",
               max_rise=0.0, hands={}, clear=False)


on_reset(_reset)
_reset()


def _joints(environment):
    from bikekit import _SPECS, _jids, _np

    asset = environment.unwrapped.scene["bike"]
    ids = _jids(asset, _SPECS["bike"])
    q, qd = _np(asset.data.joint_pos)[0, ids], _np(asset.data.joint_vel)[0, ids]
    return q, qd  # order: post_slide, post_turn, lever


@on_physics
def _mechanics(environment) -> None:
    q, qd = _joints(environment)
    slide, turn, theta = (float(v) for v in q)
    lever, f, t = CLAMP.step(theta, float(qd[2]), slide, float(qd[0]), turn, float(qd[1]))
    set_efforts(environment, "bike", {"post_slide": f, "post_turn": t, "lever": lever})
    _RT.update(theta=theta, slide=slide, turn=turn, vel=tuple(float(v) for v in qd), gripping=CLAMP.gripping)
    _RT["steps"] += 1
    _RT["max_rise"] = max(_RT["max_rise"], slide)
    if slide > MIN_INSERT_M and not _RT["failed"]:
        _RT["failed"] = "post pulled past its MIN INSERT line"


def touch_points(slide: float, turn: float, theta: float) -> np.ndarray:
    """Points on the exposed post, the saddle and the lever (world), for the hands-clear check."""
    pts = [post_point(f * PT, slide, turn) for f in np.linspace(-slide / E0 if slide < 0 else 0.0, 1.0, 5)]
    for dx in (-0.07, -0.015, 0.04, 0.075, 0.115):
        for dy in ((-0.055, 0.0, 0.055) if dx < 0.03 else (-0.02, 0.0, 0.02)):
            pts.append(post_point(PT + (dx, dy, 0.02), slide, turn))
    pts += [PIVOT + s * (paddle(theta) - PIVOT) for s in np.linspace(0.0, 1.2, 6)]
    return np.array(pts)


CLEAR_M = 0.03


@on_step
def _hands(environment) -> None:
    hands = robot_hands(environment)
    pts = touch_points(_RT["slide"], _RT["turn"], _RT["theta"])
    _RT["hands"] = hands
    _RT["clear"] = all(np.min(np.linalg.norm(pts - tcp, axis=1)) >= CLEAR_M for tcp, _, _ in hands.values())
    if DEBUG and _RT["steps"] % 48 < 24:
        print(f"[saddle] t {_RT['steps'] / 240:6.2f} rise {_RT['slide'] * 1000:6.1f} mm turn "
              f"{math.degrees(_RT['turn']):6.1f} lever {math.degrees(_RT['theta']):6.1f} grip {CLAMP.gripping} "
              f"clear {_RT['clear']} score {current():.2f} {_RT['failed']}", flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------


def current() -> float:
    return score_state(_RT["slide"] * 1000.0, math.degrees(_RT["turn"]), math.degrees(_RT["theta"]), _RT["clear"],
                       bool(_RT["failed"]))


def score(objects) -> float:
    """Read at the end. 0 if the post was ever pulled out past its MIN INSERT line (7.5 cm up), or unless the lever
    is closed past its dead point (within 12 degrees of folded), both grippers are 3 cm or more from the post, saddle
    and lever, and the tape is 10 mm or more above the collar. Then 0.3; +0.5 if the tape's lower edge is 40 +- 5 mm
    above the collar; +0.2 if the saddle is within 5 degrees of the frame's line."""
    return current()


def success(objects) -> bool:
    return current() == 1.0 and abs(_RT["vel"][0]) < 0.005


# -- scripted demo -----------------------------------------------------------------------------------------------


def demo_controller(bases, dt):
    from saddledemo import Controller

    return Controller(bases, dt, sim=_RT, cheat=os.environ.get("DEMO_CHEAT", ""))


def demo():
    """Placeholder so tools that look for a demo() find one; yam_scripted runs demo_controller instead."""
    return {}
