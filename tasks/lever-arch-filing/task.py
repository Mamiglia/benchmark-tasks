"""On the station table: an empty blue A4 lever arch file (80 mm spine) lies open, its spine along the far side, the
front cover laid flat beyond it and the back board, with the two-ring mechanism, toward the arms. The rings are
shut and the lever lies folded flat at the file's left end. In front of the file, on two wrapped reams of copier
paper laid crosswise, lies a stack of 20 punched A4 sheets, holed edge toward the file; the stack's two short ends
overhang the reams. Throwing the lever up past its centre opens both rings (the two moving halves swing back toward
the spine) and it stays open; pushing it back down past its centre snaps both rings shut.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from binderkit import (Slide, _part_state, _SPECS, _jids, _np, frame_x, on_physics, on_reset, on_step, part,  # noqa: E402
                       robot_hands, set_efforts, set_slide_targets)
from bindermodel import (ARCH_OPEN, ARCH_PTS, ARCH_SHUT_DEG, BASE_H, BASE_X, BOARD_HALF_Y, BOARD_T, BOARD_X, BZ,  # noqa: E402
                         CLEAR_M, COVER_X, CREASE, FORCED_DEG, HINGE1_X, HINGE2_X, HINGE_K, LEVER_ARM, LEVER_OPEN,
                         LEVER_PIVOT, LEVER_PLATE, LEVER_SHUT_DEG, OPENED_DEG, PIVOT, PLATE_W, POST_D, POST_H, POST_X,
                         POST_Y, RATIO, SHEET_L, SPINE_X, STACK_MASS, STACK_T, STRIP_W, TIP_D, TIP_LEN, TOP, LeverParams,
                         arch_point, arch_target, lever_effort, lever_point, score_state, strip_boxes, strip_points,
                         threaded)

SCENE = "station"
SCORE_OVER = "end"
INSTRUCTION = "File the punched sheets in the lever arch file and lock its rings shut."

BLUE, BLUE_D, METAL, WHITE, PAPER = (0.10, 0.25, 0.62), (0.07, 0.17, 0.45), (0.72, 0.73, 0.75), (0.93, 0.93, 0.92), \
    (0.96, 0.96, 0.94)
REAM, REAM_BAND, HOLE_C = (0.52, 0.62, 0.74), (0.95, 0.75, 0.15), (0.35, 0.35, 0.38)


def seg(p0, p1, d, color, collide=True, origin=(0.0, 0.0, 0.0)):
    """A cylinder of diameter ``d`` from p0 to p1 (world), as a box spec relative to ``origin``."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    c = (p0 + p1) / 2 - np.asarray(origin, float)
    return (tuple(c), (float(np.linalg.norm(p1 - p0)), d, d), color, collide, "cyl", frame_x(p1 - p0))


def slab(x0, x1, y0, y1, z0, z1, color, collide=True):
    return (((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color, collide)


# -- the file (fixed root: boards, base plate, fixed posts) ------------------------------------------------------
Y0, Y1 = -BOARD_HALF_Y, BOARD_HALF_Y
FILE = [
    slab(*BOARD_X, Y0, Y1, TOP, BZ, BLUE),  # back board
    slab(*SPINE_X, Y0, Y1, TOP, BZ, BLUE_D),  # spine, laid flat
    slab(*COVER_X, Y0, Y1, TOP, BZ, BLUE),  # front cover, laid flat
    slab(SPINE_X[0] + 0.015, SPINE_X[1] - 0.015, -0.12, 0.05, BZ, BZ + 0.0002, WHITE, False),  # spine label
    slab(SPINE_X[0] + 0.030, SPINE_X[1] - 0.030, 0.085, 0.125, BZ, BZ + 0.0003, BLUE_D, False),  # finger hole
    slab(*BASE_X, -0.140, 0.140, BZ, BZ + BASE_H, METAL),  # mechanism base plate
    slab(BASE_X[0], PIVOT[0] + 0.006, 0.140, 0.172, BZ, BZ + BASE_H, METAL),  # lever bracket
]
for y in POST_Y:
    FILE += [seg((POST_X, y, BZ), (POST_X, y, BZ + POST_H - TIP_LEN), POST_D, METAL),
             seg((POST_X, y, BZ + POST_H - TIP_LEN), (POST_X, y, BZ + POST_H), TIP_D, METAL),
             seg((POST_X - 0.004, y, BZ + 0.002), (BASE_X[0] + 0.004, y, BZ + 0.002), 0.004, METAL)]  # post foot

ARCH = []
for y in POST_Y:
    for (x0, z0), (x1, z1) in zip(ARCH_PTS[:-1], ARCH_PTS[1:], strict=True):
        ARCH.append(seg((x0, y, z0), (x1, y, z1), POST_D, METAL))
ARCH.append(seg((0.0, -0.140, 0.0), (0.0, 0.140, 0.0), 0.006, METAL))  # the bar carrying both halves
LEVER = [seg((*LEVER_ARM[0][:1], 0.0, LEVER_ARM[0][1]), (LEVER_ARM[1][0], 0.0, LEVER_ARM[1][1]), 0.006, METAL),
         (((LEVER_PLATE[0][0] + LEVER_PLATE[1][0]) / 2, 0.0, (LEVER_PLATE[0][1] + LEVER_PLATE[1][1]) / 2),
          (math.dist(*LEVER_PLATE), PLATE_W, 0.003), METAL, True, "box",
          frame_x((LEVER_PLATE[1][0] - LEVER_PLATE[0][0], 0.0, LEVER_PLATE[1][1] - LEVER_PLATE[0][1])))]
P = LeverParams()
ARCH_SLIDE = Slide(name="arch", origin=tuple(PIVOT), axis=(0.0, 1.0, 0.0), boxes=tuple(ARCH), lower=0.0,
                   upper=ARCH_OPEN, stiffness=P.arch_k, damping=P.arch_d, mass=0.03, revolute=True, armature=2e-4)
LEVER_SLIDE = Slide(name="lever", origin=tuple(LEVER_PIVOT), axis=(0.0, 1.0, 0.0), boxes=tuple(LEVER), lower=0.0,
                    upper=LEVER_OPEN, stiffness=0.0, damping=0.001, mass=0.02, revolute=True, armature=2e-4)
BINDER = part("binder", (0.0, 0.0, 0.0), FILE, mass=0.49, color=BLUE, friction=0.6,
              slides=(ARCH_SLIDE, LEVER_SLIDE), fixed=True)

# -- the stack: punched strip (root) + two hinged parts ---------------------------------------------------------
STACK_EDGE_X = 0.150  # holed edge at the start
STACK_XY = (STACK_EDGE_X - STRIP_W[0] / 2, 0.0)
REAM_X = (STACK_EDGE_X - 0.2565, STACK_EDGE_X + 0.0405)  # 297 mm along x, 210 mm along y
REAM_H = 0.100
STACK_Z = TOP + REAM_H + STACK_T / 2 + 0.0005
MID = Slide(name="mid", origin=(HINGE1_X, 0.0, 0.0), axis=(0.0, 1.0, 0.0),
            boxes=(((-STRIP_W[1] / 2, 0.0, 0.0), (STRIP_W[1], SHEET_L, STACK_T), PAPER),), lower=-math.radians(150),
            upper=math.radians(150), stiffness=HINGE_K[0], damping=0.002, mass=STACK_MASS * STRIP_W[1] / 0.21,
            revolute=True, armature=1e-5)
FREE = Slide(name="free", origin=(HINGE2_X, 0.0, 0.0), axis=(0.0, 1.0, 0.0),
             boxes=(((-STRIP_W[2] / 2, 0.0, 0.0), (STRIP_W[2], SHEET_L, STACK_T), PAPER),), lower=-math.radians(150),
             upper=math.radians(150), stiffness=HINGE_K[1], damping=0.001, mass=STACK_MASS * STRIP_W[2] / 0.21,
             revolute=True, armature=1e-5, parent="mid")
STACK = part("stack", (*STACK_XY, STACK_Z), strip_boxes(PAPER, HOLE_C), mass=STACK_MASS * STRIP_W[0] / 0.21,
             color=PAPER, friction=0.5, slides=(MID, FREE), contact_offset=0.0008)
REAMS = part("reams", (0.0, 0.0, 0.0), [slab(*REAM_X, -0.105, 0.105, TOP, TOP + REAM_H, REAM),
                                        slab(REAM_X[0] - 0.0005, REAM_X[1] + 0.0005, -0.0306, 0.0306, TOP + 0.002,
                                             TOP + REAM_H - 0.002, REAM_BAND, False)],
             mass=5.0, color=REAM, friction=0.5, fixed=True)
OBJECTS = [BINDER, STACK, REAMS]

# -- run-time mechanics and what happened ------------------------------------------------------------------------
_RT: dict = {}
DEBUG = bool(os.environ.get("BINDER_DEBUG"))
ARCH_SAMPLES = [(x0 + s * (x1 - x0), z0 + s * (z1 - z0))
                for (x0, z0), (x1, z1) in zip(ARCH_PTS[:-1], ARCH_PTS[1:], strict=True) for s in (0.0, 0.5, 1.0)]


def _reset(environment=None) -> None:
    _RT.update(steps=0, lever=0.0, arch=0.0, failed="", ever_opened=False, n_threaded=0, shut=True, clear=True,
               hinges=(0.0, 0.0), stack_pos=np.array([*STACK_XY, STACK_Z]), stack_R=np.eye(3), hands={},
               forced_steps=0, score=0.0)


on_reset(_reset)
_reset()


def _binder_joints(environment):
    asset = environment.unwrapped.scene["binder"]
    ids = _jids(asset, _SPECS["binder"])
    return _np(asset.data.joint_pos)[0, ids], _np(asset.data.joint_vel)[0, ids]  # arch, lever


@on_physics
def _mechanics(environment) -> None:
    q, qd = _binder_joints(environment)
    arch, lever = float(q[0]), float(q[1])
    set_efforts(environment, "binder", {"lever": lever_effort(lever, float(qd[1]), arch, P)})
    set_slide_targets(environment, "binder", {"arch": arch_target(lever)})
    _RT.update(arch=arch, lever=lever)


def _fail(why: str) -> None:
    if not _RT["failed"]:
        _RT["failed"] = why
        if DEBUG:
            print(f"[binder] FAILED: {why}", flush=True)


@on_step
def _watch(environment) -> None:
    rt = _RT
    rt["steps"] += 1
    st = _part_state(environment, _SPECS["stack"])
    pos, R = st.pos, st.rotation()
    rt["stack_pos"], rt["stack_R"] = pos, R
    rt["hinges"] = tuple(float(v) for v in st.slides)
    hands = robot_hands(environment)
    rt["hands"] = hands
    lever, arch = rt["lever"], rt["arch"]
    if math.degrees(lever) > OPENED_DEG:
        rt["ever_opened"] = True
    if rt["steps"] > 5:
        if max(abs(h) for h in rt["hinges"]) > CREASE:
            _fail("stack creased")
        if pos[2] < TOP - 0.02:
            _fail("stack fell off the table")
        arch_pts = np.array([arch_point(p, arch, y) for p in ARCH_SAMPLES for y in POST_Y])
        hand_at_arch = any(np.min(np.linalg.norm(arch_pts - tcp, axis=1)) < 0.04 for tcp, _, _ in hands.values())
        over = math.degrees(arch - RATIO * lever) > FORCED_DEG
        rt["forced_steps"] = rt["forced_steps"] + 1 if (over and hand_at_arch) else 0
        if rt["forced_steps"] >= 3:
            _fail("rings forced open by hand")
    rt["n_threaded"] = sum(threaded(pos, R, i) for i in range(2))
    rt["shut"] = math.degrees(lever) < LEVER_SHUT_DEG and math.degrees(arch) < ARCH_SHUT_DEG
    pts = np.vstack([strip_points(pos, R), [lever_point(p, lever) for p in (*LEVER_ARM, *LEVER_PLATE)]])
    rt["clear"] = all(np.min(np.linalg.norm(pts - tcp, axis=1)) >= CLEAR_M for tcp, _, _ in hands.values())
    rt["score"] = current()
    if DEBUG and rt["steps"] % 10 == 0:
        h = [hole for hole in (pos + R @ np.array([STRIP_W[0] / 2 - 0.012, y, 0.0]) for y in POST_Y)]
        print(f"[binder] t {rt['steps'] / 20:6.2f} lever {math.degrees(lever):6.1f} arch {math.degrees(arch):5.1f} "
              f"hinges {np.round(np.degrees(rt['hinges']), 0)} holes {np.round(np.array(h) * 1000, 1).tolist()} "
              f"thr {rt['n_threaded']} shut {rt['shut']} clear {rt['clear']} score {rt['score']:.2f} {rt['failed']}",
              flush=True)


# -- scoring -------------------------------------------------------------------------------------------------------


def current() -> float:
    return score_state(bool(_RT["failed"]), _RT["n_threaded"], _RT["shut"], _RT["clear"], _RT["ever_opened"])


def score(objects) -> float:
    """Read at the end. 0 if at any time the stack was creased (bent over 120 degrees), fell off the table, or the
    rings were pushed open by hand beyond what the lever allows. Otherwise 1.0 with both posts through both holes, the
    lever folded shut (within 5 degrees) so both rings are shut, and both grippers 3 cm or more from the stack's holed
    strip and the lever; 0.5 with both posts through both holes; 0.2 with one post through its hole; 0.1 if the lever
    was ever thrown past its centre (rings opened); else 0."""
    return current()


def success(objects) -> bool:
    return current() == 1.0


# -- scripted demo -----------------------------------------------------------------------------------------------


def demo_controller(bases, dt):
    from binderdemo import Controller

    return Controller(bases, dt, sim=_RT, cheat=os.environ.get("DEMO_CHEAT", ""))


def demo():
    """Placeholder so tools that look for a demo() find one; yam_scripted runs demo_controller instead."""
    return {}
