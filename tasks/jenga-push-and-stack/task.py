"""On the station table: a freshly built Jenga Classic tower (54 hardwood blocks, 1.5 x 2.5 x 7.5 cm, 18 levels of
three, each level crosswise to the one below, 27 cm tall), standing on the bare table 30 cm in front of the midpoint
between the arms. Level 1 (on the table) has its blocks pointing away from the robot (along x); the even levels,
including the top level 18, run left-right (along y). Nothing else on the table.

As in every real Jenga set, the blocks are not all exactly the same thickness, so some blocks carry the levels above
and are pinched tight, and others are loose. Which ones are loose is not visible and changes with the seed
(JENGA_SEED, environment variable, default 0).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import jengakit  # noqa: E402,F401  (hooks yam_arena on import)
from jengakit import ON_RESET, Block, quat_matrix, tilt_deg  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 5.0  # official rule: the turn ends five seconds after the block is stacked
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("Play one turn of Jenga: using one hand at a time, take a block out from below the top level and "
               "stack it on top, crosswise to the top level, without knocking the tower over.")

# -- the tower ---------------------------------------------------------------------------------------------------------
LENGTH, WIDTH, HEIGHT = 0.075, 0.025, 0.015  # m, Hasbro Jenga Classic block
MASS = 0.015  # kg
LEVELS = 18
PITCH = WIDTH + 0.0003  # 0.3 mm play between neighbours in a level
TOL = 0.0003  # m: each block's thickness is HEIGHT + uniform(-TOL, +TOL)
TOWER_XY = (0.30, 0.0)
HALF = 1.5 * PITCH  # half the tower's footprint (m)
SEED = int(os.environ.get("JENGA_SEED", "0"))
WOOD = (0.86, 0.70, 0.47)


def thicknesses(seed: int) -> np.ndarray:
    """(LEVELS, 3) block thicknesses (m) for a seed; column 1 is the middle block."""
    return HEIGHT + np.random.default_rng(seed).uniform(-TOL, TOL, size=(LEVELS, 3))


def along_x(level: int) -> bool:
    """Odd levels (1 = on the table) point along x, even levels along y."""
    return level % 2 == 1


def name(level: int, i: int) -> str:
    return f"b{level:02d}_{i}"


def build(seed: int):
    h = thicknesses(seed)
    rng = np.random.default_rng(seed + 1000)
    blocks, z0 = [], TOP
    for k in range(1, LEVELS + 1):
        for i in range(3):
            off = (i - 1) * PITCH
            if along_x(k):
                pos, size = (TOWER_XY[0], TOWER_XY[1] + off), (LENGTH, WIDTH, h[k - 1, i])
            else:
                pos, size = (TOWER_XY[0] + off, TOWER_XY[1]), (WIDTH, LENGTH, h[k - 1, i])
            shade = float(rng.uniform(-0.05, 0.05))
            blocks.append(Block(name(k, i), pos=tuple(float(v) for v in (*pos, z0 + h[k - 1, i] / 2 + 1e-5)),
                                size=tuple(float(v) for v in size), mass=MASS,
                                color=tuple(float(np.clip(c + shade, 0, 1)) for c in WOOD)))
        z0 += float(h[k - 1].max()) + 2e-5
    return blocks, z0


BLOCKS, TOWER_TOP = build(SEED)
OBJECTS = list(BLOCKS)
NAMES = [b.name for b in BLOCKS]
LEVEL = {b.name: int(b.name[1:3]) for b in BLOCKS}
TAKEABLE = [n for n in NAMES if LEVEL[n] < LEVELS]  # any block below the highest completed level

# -- scoring -----------------------------------------------------------------------------------------------------------
FELL_M = 0.025  # another block more than this from its start, or tilted past FELL_DEG: the tower fell
FELL_DEG = 15.0
DISTURB_M = 0.010  # another block ever this far from its start
END_OTHER_M = 0.005  # ... or this far at the end
PUSHED_M = 0.005  # the taken block slid this far along its length
HALF_M = LENGTH / 2
ON_TOP_GAP = 0.003  # its underside within this of the top level's upper face
CROSS_DEG = 20.0
CENTRE_M = 0.030  # its centre within this of the tower's axis (x and y)
LEVEL_DEG = 10.0

STATE: dict = {}


def _reset():
    STATE.clear()


ON_RESET.append(_reset)


def _axis(n: str) -> np.ndarray:
    return np.array([1.0, 0.0, 0.0]) if along_x(LEVEL[n]) else np.array([0.0, 1.0, 0.0])


def _in_tower(s) -> bool:
    """Any part of the block inside the tower's footprint, below the top level's upper face."""
    (x0, y0), z_top = TOWER_XY, STATE["top_z"]
    return bool(s.low[0] < x0 + HALF and s.high[0] > x0 - HALF and s.low[1] < y0 + HALF and s.high[1] > y0 - HALF
                and s.low[2] < z_top - 0.002)


def _update(objects) -> dict:
    if "ref" not in STATE:
        STATE.update(ref={n: objects[n].pos.copy() for n in NAMES},
                     top_z=max(float(objects[n].high[2]) for n in NAMES if LEVEL[n] == LEVELS),
                     fell=False, disturbed=False, two_hands=False, pushed=0.0, steps=0)
    ref = STATE["ref"]
    disp = {n: float(np.linalg.norm(objects[n].pos - ref[n])) for n in NAMES}
    taken = max(TAKEABLE, key=disp.get)
    t = objects[taken]
    STATE["pushed"] = max(STATE["pushed"], abs(float(np.dot(t.pos - ref[taken], _axis(taken)))))
    others = [n for n in NAMES if n != taken]
    if any(disp[n] > FELL_M or tilt_deg(objects[n].quat_xyzw) > FELL_DEG for n in others):
        STATE["fell"] = True
    if any(disp[n] > DISTURB_M for n in others):
        STATE["disturbed"] = True
    tower = others if not _in_tower(t) else NAMES
    if any(objects[n].left for n in tower) and any(objects[n].right for n in tower):
        STATE["two_hands"] = True
    STATE["steps"] += 1
    return dict(disp=disp, taken=taken, others=others)


def on_top(s, n: str) -> bool:
    """Lying flat on the top level, crosswise to it (its length within 20 deg of x), centred over the tower, let go
    and still."""
    long_axis = quat_matrix(s.quat_xyzw)[:, 0 if along_x(LEVEL[n]) else 1]
    cross = math.degrees(math.acos(min(1.0, abs(float(long_axis[0])) / max(1e-9, float(np.linalg.norm(long_axis[:2]))))))
    return bool(abs(float(s.low[2]) - STATE["top_z"]) < ON_TOP_GAP and tilt_deg(s.quat_xyzw) < LEVEL_DEG
                and cross < CROSS_DEG and abs(float(s.pos[0]) - TOWER_XY[0]) < CENTRE_M
                and abs(float(s.pos[1]) - TOWER_XY[1]) < CENTRE_M and not (s.left or s.right) and s.resting())


def score(objects) -> float:
    """0 if the tower fell (another block moved over 2.5 cm or tilted past 15 deg) or both hands touched the tower at
    once. Else, for the block taken (the one below the top level that moved most): 1.0 on top as `on_top` says;
    0.6 out of the tower; 0.3 slid half its length (3.75 cm) or more; 0.1 slid 5 mm or more; else 0. Capped at 0.1
    if another block was ever moved 1 cm or more, or is 5 mm or more from its start at the end."""
    u = _update(objects)
    if STATE["fell"] or STATE["two_hands"]:
        if os.environ.get("JENGA_DEBUG") and not STATE.get("said"):
            STATE["said"] = True
            print(f"[jenga] step {STATE['steps']} score 0: fell={STATE['fell']} 2h={STATE['two_hands']}", flush=True)
        return 0.0
    t = objects[u["taken"]]
    if on_top(t, u["taken"]):
        s = 1.0
    elif not _in_tower(t):
        s = 0.6
    elif STATE["pushed"] >= HALF_M:
        s = 0.3
    elif STATE["pushed"] >= PUSHED_M:
        s = 0.1
    else:
        s = 0.0
    if STATE["disturbed"] or any(u["disp"][n] > END_OTHER_M for n in u["others"]):
        s = min(s, 0.1)
    if os.environ.get("JENGA_DEBUG"):
        touched = [n + ("L" if objects[n].left else "") + ("R" if objects[n].right else "") for n in NAMES
                   if objects[n].left or objects[n].right]
        if touched and touched != STATE.get("touched"):
            print(f"[jenga] step {STATE['steps']} touched {touched}", flush=True)
        STATE["touched"] = touched
    if os.environ.get("JENGA_DEBUG") and STATE["steps"] % 20 == 1:
        d = u["disp"]
        worst = max(u["others"], key=d.get)
        tilt = max(tilt_deg(objects[n].quat_xyzw) for n in NAMES)
        print(f"[jenga] step {STATE['steps']} score {s} taken {u['taken']} {d[u['taken']]*1e3:.1f}mm "
              f"worst other {worst} {d[worst]*1e3:.2f}mm max tilt {tilt:.2f} deg latches "
              f"fell={STATE['fell']} dist={STATE['disturbed']} 2h={STATE['two_hands']}", flush=True)
    return s


def success(objects) -> bool:
    return score(objects) == 1.0


# -- scripted demo -----------------------------------------------------------------------------------------------------
# yam_scripted runs jenga_demo.Controller closed loop: it sees only the joint positions (gripper opening included) and
# the commands it sent. It knows the start layout of the tower, as every yam_scripted demo does, never which blocks are
# loose.


def demo_controller(bases, dt):
    from jenga_demo import Controller

    return Controller(bases, dt, tower_xy=TOWER_XY, table_z=TOP, top_z=TOWER_TOP, pitch=PITCH, level_h=HEIGHT,
                      levels=LEVELS, mode=os.environ.get("DEMO_MODE", ""))


def demo():
    """Placeholder so tools that look for a demo() find one; yam_scripted runs demo_controller instead."""
    return {}
