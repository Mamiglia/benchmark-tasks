"""A two-robot machine-tending cell on the station table. The LEFT arm is the cell's other robot: it loads a machine
and runs its own program, so commands sent to the left arm are ignored. The RIGHT arm is yours.

On the table, as seen from behind the arms:
- between the arms, 32 cm out: the infeed nest, a black acetal block with a square pocket 4 cm across and 1.2 cm
  deep. One aluminium blank already stands in it.
- in front of the right arm: a blue-grey tray with five aluminium blanks standing on end (bar stock sawn to
  30 mm diameter x 40 mm long, 76 g each).
- in front of the left arm: the machine's infeed chute, an open-topped steel box 12 cm square inside, 10 cm tall.

Every so often the other arm reaches into the nest, closes its gripper, carries whatever it holds to the chute,
drops it in and goes back to its waiting pose. How long it waits between visits changes from cycle to cycle and from
trial to trial (the trial seed sets it). It visits the nest six times in a trial and does not wait for you.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score --seed S ...
"""

# Idea first proposed in robocurve/benchmark-tasks#441; independent implementation.

from __future__ import annotations

import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from cellkit import Cell, configure, monitor  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("Keep the infeed nest stocked with one blank at a time from your tray, so the other arm finds a blank "
               "there every time it comes for one.")

# -- parts and fixtures (world: x forward from the arms, y left; metres) ---------------------------------------------
BLANK_R, BLANK_H, BLANK_KG = 0.015, 0.040, 0.076  # 6061 aluminium, 30 x 40 mm: 2.70 g/cm3 x 28.3 cm3
ALU = (0.80, 0.81, 0.83)
ACETAL = (0.06, 0.06, 0.07)
TRAY_BLUE = (0.22, 0.27, 0.34)
STEEL = (0.42, 0.47, 0.45)

NEST_XY = (0.32, 0.0)
NEST_IN = 0.040  # pocket inner width
NEST_WALL, NEST_H = 0.015, 0.012

TRAY_XY = (0.33, -0.375)
TRAY_IN = (0.22, 0.18)  # inner x, y
TRAY_FLOOR, TRAY_RIM, TRAY_RIM_H = 0.005, 0.010, 0.012
SUPPLY_XY = ((0.26, -0.33), (0.33, -0.33), (0.40, -0.33), (0.295, -0.42), (0.365, -0.42))
BLANKS = tuple(f"blank_{i + 1}" for i in range(len(SUPPLY_XY)))
PRELOAD = "blank_0"

CHUTE_XY = (0.27, 0.40)
CHUTE_IN, CHUTE_WALL, CHUTE_H = 0.12, 0.010, 0.100


def _frame(prefix, xy, inner_x, inner_y, wall, height, color, z0=TOP):
    """Four walls round an inner rectangle, standing on z0."""
    x, y = xy
    ox, oy = inner_x + 2 * wall, inner_y + 2 * wall
    zc = z0 + height / 2
    return [
        Box(f"{prefix}_far", pos=(x + (inner_x + wall) / 2, y, zc), size=(wall, oy, height), color=color),
        Box(f"{prefix}_near", pos=(x - (inner_x + wall) / 2, y, zc), size=(wall, oy, height), color=color),
        Box(f"{prefix}_left", pos=(x, y + (inner_y + wall) / 2, zc), size=(inner_x, wall, height), color=color),
        Box(f"{prefix}_right", pos=(x, y - (inner_y + wall) / 2, zc), size=(inner_x, wall, height), color=color),
    ]


NEST = _frame("nest", NEST_XY, NEST_IN, NEST_IN, NEST_WALL, NEST_H, ACETAL)
TRAY = [Box("tray_floor", pos=(*TRAY_XY, TOP + TRAY_FLOOR / 2),
            size=(TRAY_IN[0] + 2 * TRAY_RIM, TRAY_IN[1] + 2 * TRAY_RIM, TRAY_FLOOR), color=TRAY_BLUE),
        *_frame("tray", TRAY_XY, *TRAY_IN, TRAY_RIM, TRAY_RIM_H - TRAY_FLOOR, TRAY_BLUE, z0=TOP + TRAY_FLOOR)]
CHUTE = _frame("chute", CHUTE_XY, CHUTE_IN, CHUTE_IN, CHUTE_WALL, CHUTE_H, STEEL)
PARTS = [Cylinder(PRELOAD, pos=(*NEST_XY, TOP + BLANK_H / 2 + 0.0005), radius=BLANK_R, height=BLANK_H,
                  mass=BLANK_KG, color=ALU)]
PARTS += [Cylinder(n, pos=(x, y, TOP + TRAY_FLOOR + BLANK_H / 2 + 0.0005), radius=BLANK_R, height=BLANK_H,
                   mass=BLANK_KG, color=ALU) for n, (x, y) in zip(BLANKS, SUPPLY_XY, strict=True)]
OBJECTS = [*NEST, *TRAY, *CHUTE, *PARTS]

CELL = Cell(
    table_z=TOP, nest_xy=NEST_XY, nest_half=NEST_IN / 2,
    chute_inner=(CHUTE_XY[0] - CHUTE_IN / 2, CHUTE_XY[0] + CHUTE_IN / 2, CHUTE_XY[1] - CHUTE_IN / 2,
                 CHUTE_XY[1] + CHUTE_IN / 2),
    chute_wall_h=CHUTE_H, blank_h=BLANK_H, blanks=BLANKS, preload=PRELOAD,
    home=(0.20, 0.28, TOP + 0.15), first_close=(5.0, 6.5), gap=(10.0, 16.0),
)
configure(CELL)


# -- scoring ---------------------------------------------------------------------------------------------------------


def _blanks(objects) -> dict:
    return {n: objects[n].pos for n in (*BLANKS, PRELOAD)}


def score(objects) -> float:
    """1/5 for each of the five tray blanks that the other arm found standing in the nest at one of its visits
    (the right gripper at least 8 cm from the nest when it closed) and that is now in the chute. The blank that
    started in the nest earns nothing. 0 if the two arms ever touched, or if two blanks ever lay in the nest at once.
    """
    return monitor().score(_blanks(objects))


def success(objects) -> bool:
    """All five supplied blanks loaded; must hold for 1 s."""
    return score(objects) == 1.0


# -- scripted demo -----------------------------------------------------------------------------------------------------
# yam_scripted runs feed_demo.Controller closed loop on each observation: the top camera image (is a blank standing
# in the nest?) and the 14-D joint positions (where is the other arm's gripper?). It knows the start layout, as every
# yam_scripted demo does, and never the other arm's timing.


def demo_controller(bases, dt):
    from feed_demo import Controller

    return Controller(bases, dt, table_z=TOP, nest_xy=NEST_XY, supply=SUPPLY_XY, tray_z=TOP + TRAY_FLOOR,
                      blank_h=BLANK_H, cheat=os.environ.get("DEMO_CHEAT", ""))


def demo():
    """Placeholder so tools that look for a demo() find one; yam_scripted runs demo_controller instead."""
    return {}
