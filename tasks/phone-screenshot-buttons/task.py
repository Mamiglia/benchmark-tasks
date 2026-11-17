"""On the desk: an iPhone 12 (146.7 x 71.5 x 7.4 mm, 164 g) stands in portrait in a loose aluminium desk stand, screen
towards the robot, leaning back 20 degrees on the stand's back rest, its bottom edge on the stand's seat behind a
front lip. Its side button is on the right-hand edge as seen from the robot (towards the right arm), its volume-up
and volume-down buttons on the left-hand edge. The phone is unlocked and awake.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys
from types import SimpleNamespace

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from phonekit import DeviceRules, Phone, PhoneParams, SeatRules, Stand, configure, grade, phone_pose_in_stand  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("Take one screenshot on the phone: press its side button and volume-up button together and let go "
               "at once. Leave the phone in its stand.")

PARAMS = PhoneParams()
PITCH = math.radians(20)
STAND_XY = (0.30, 0.0)
STAND = Stand("stand", pos=(STAND_XY[0], STAND_XY[1], TOP + 0.0005), mass=0.25, color=(0.62, 0.63, 0.66),
              pitch=PITCH)
_px, _pz = phone_pose_in_stand(STAND, PARAMS)
PHONE_POS = (STAND_XY[0] + _px, STAND_XY[1], TOP + _pz)
PHONE = Phone("phone", pos=PHONE_POS, mass=PARAMS.mass, params=PARAMS, pitch=PITCH)
OBJECTS = [PHONE, STAND]

RULES = DeviceRules()
SEAT = SeatRules()
configure(PHONE, STAND, RULES, SEAT)


def score(objects) -> float:
    """0 if the phone ever left its seat (moved more than 1 cm up or down, 1.5 cm sideways or turned 10 degrees);
    else 0.1 if the power-off slider came up; 1.0 for exactly one screenshot and nothing else (no volume change, no
    lock), the robot no longer touching the phone; 0.5 for a screenshot with any other effect (or two screenshots, or
    still touching); 0.3 for single-button effects only (volume, lock); 0 if nothing happened."""
    s = objects["phone"]
    return grade(s.events, s.out, s.touching)


def success(objects) -> bool:
    return score(objects) == 1.0


# -- scripted demo -----------------------------------------------------------------------------------------------------


def button_point(z_p: float) -> tuple[float, float]:
    """World (x, z) of the middle of the phone's edge, ``z_p`` metres above the phone's centre along its length."""
    return PHONE_POS[0] + z_p * math.sin(PITCH), PHONE_POS[2] + z_p * math.cos(PITCH)


def demo_controller(bases, dt):
    from phone_demo import Controller

    return Controller(bases, dt, task=SimpleNamespace(PARAMS=PARAMS, button_point=button_point),
                      cheat=os.environ.get("DEMO_CHEAT", ""))


def demo():
    """Placeholder so tools that look for a demo() find one; yam_scripted runs demo_controller instead."""
    return {}
