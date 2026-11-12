"""On the station table: four identical sealed white cups (7 cm across, 9 cm tall, thin straight walls, a red seal
6 mm below the rim) stand in two pairs, one pair in front of each arm. Between the pairs, a dark grey tray with low
rims (21 x 28 cm, rims 2 cm tall). Each cup holds a different, unseen mass of damp sand.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Fill masses come from the seed CUPS_SEED (environment variable, default 0).
"""

from __future__ import annotations

import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from cupkit import Cup, CupParams, Rules, configure, total_mass  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Put all four cups on the tray upright."

PARAMS = CupParams()

# -- hidden fills ----------------------------------------------------------------------------------------------------
# One cup from each band per seed, in a seeded order. Every seed has a cup of 60 g or less (buckles at 3.5 N or
# less) and one of 320 g or more (needs about 5.1 N per finger to lift at the effective friction 0.33), so no single
# grip force lifts all four intact.
FILL_BANDS = ((0.020, 0.060), (0.100, 0.180), (0.200, 0.280), (0.320, 0.400))


def fills_for(seed: int) -> tuple[float, ...]:
    """The four fills (kg, rounded to 5 g) for a seed, in the order of CUP_XY."""
    rng = np.random.default_rng(seed)
    values = [round(round(rng.uniform(lo, hi) / 0.005) * 0.005, 3) for lo, hi in FILL_BANDS]
    return tuple(float(values[i]) for i in rng.permutation(len(values)))


SEED = int(os.environ.get("CUPS_SEED", "0"))
FILLS = fills_for(SEED)

# -- layout (world: x forward from the arms, y left; metres) ---------------------------------------------------------
CUP_XY = ((0.34, 0.21), (0.22, 0.21), (0.34, -0.21), (0.22, -0.21))  # left far, left near, right far, right near
CUP_NAMES = ("cup_lf", "cup_ln", "cup_rf", "cup_rn")
TRAY_XY = (0.30, 0.0)
TRAY_INNER = (0.205, 0.395, -0.13, 0.13)  # x0, x1, y0, y1
RIM = 0.01
TRAY_OUTER = (TRAY_INNER[0] - RIM, TRAY_INNER[1] + RIM, TRAY_INNER[2] - RIM, TRAY_INNER[3] + RIM)
TRAY_FLOOR = 0.005  # floor thickness: its top is 5 mm above the table
RIM_H = 0.02
SLOTS = {"cup_lf": (0.345, 0.07), "cup_ln": (0.255, 0.07), "cup_rf": (0.345, -0.07), "cup_rn": (0.255, -0.07)}

TRAY_GREY = (0.25, 0.26, 0.28)
CUPS = [
    Cup(name, pos=(x, y, TOP + 0.0005), size=(2 * PARAMS.radius, 2 * PARAMS.radius, PARAMS.height),
        mass=total_mass(fill, PARAMS), fill=fill, params=PARAMS)
    for name, (x, y), fill in zip(CUP_NAMES, CUP_XY, FILLS, strict=True)
]
_ix0, _ix1, _iy0, _iy1 = TRAY_INNER
_ox0, _ox1, _oy0, _oy1 = TRAY_OUTER
_lx, _ly = _ox1 - _ox0, _oy1 - _oy0
TRAY = [
    Box("tray_floor", pos=(TRAY_XY[0], TRAY_XY[1], TOP + TRAY_FLOOR / 2), size=(_lx, _ly, TRAY_FLOOR), color=TRAY_GREY),
    Box("tray_rim_far", pos=(_ox1 - RIM / 2, TRAY_XY[1], TOP + RIM_H / 2), size=(RIM, _ly, RIM_H), color=TRAY_GREY),
    Box("tray_rim_near", pos=(_ox0 + RIM / 2, TRAY_XY[1], TOP + RIM_H / 2), size=(RIM, _ly, RIM_H), color=TRAY_GREY),
    Box("tray_rim_left", pos=(TRAY_XY[0], _oy1 - RIM / 2, TOP + RIM_H / 2), size=(_lx - 2 * RIM, RIM, RIM_H),
        color=TRAY_GREY),
    Box("tray_rim_right", pos=(TRAY_XY[0], _oy0 + RIM / 2, TOP + RIM_H / 2), size=(_lx - 2 * RIM, RIM, RIM_H),
        color=TRAY_GREY),
]
OBJECTS = [*CUPS, *TRAY]

RULES = Rules(table_z=TOP, tray_z=TOP + TRAY_FLOOR, tray_inner=TRAY_INNER, tray_outer=TRAY_OUTER)
configure(CUPS, RULES)


# -- scoring ---------------------------------------------------------------------------------------------------------


def cup_ok(s) -> bool:
    """One cup's credit: never failed (wall buckled, robot touched its base or seal, off its support without a wall
    grip, slid more than 1 cm in the grip, tipped past 30 degrees, carried with another cup in one gripper, off the
    table); lifted at least 2 cm in a wall grip; let go on the tray; now standing on the tray within 10 degrees of
    upright, released."""
    return (not s.failed and s.lifted and s.placed and s.on_tray and s.tilt_deg() < RULES.upright_deg
            and s.holder is None)


def score(objects) -> float:
    """0.25 per cup that is on the tray as `cup_ok` says; 1.0 for all four."""
    return sum(cup_ok(objects[n]) for n in CUP_NAMES) / len(CUP_NAMES)


def success(objects) -> bool:
    """All four cups intact, upright and released on the tray; must hold for 1 s."""
    return score(objects) == 1.0


# -- scripted demo -----------------------------------------------------------------------------------------------------
# yam_scripted runs grip_demo.Controller closed loop: it sees only the camera images and the joint positions (gripper
# opening included). It knows the start layout (as every yam_scripted demo does), never the fills.


def demo_controller(bases, dt):
    from grip_demo import Controller

    return Controller(bases, dt, cups={n: CUP_XY[i] for i, n in enumerate(CUP_NAMES)}, slots=SLOTS, table_z=TOP,
                      tray_z=TOP + TRAY_FLOOR, cheat=os.environ.get("DEMO_CHEAT", ""))


def demo():
    """Placeholder so tools that look for a demo() find one; yam_scripted runs demo_controller instead."""
    return {}
