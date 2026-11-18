"""On the station table, in front of the right arm: a Logitech Driving Force Shifter (6 speeds + push-down reverse,
H-pattern, black leather knob 45 mm across, 50 mm tall) bolted through its two M6 mounting holes to a 12 mm plywood
board taped to the table. The lever starts in neutral, centred on the 3-4 plane. 1-2 is the left plane, 5-6 the
right; reverse is the 6 slot entered with the knob pushed down.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import shiftmodel as M  # noqa: E402
from bikekit import (_SPECS, Slide, _jids, _np, frame_x, on_physics, on_reset, on_step, part, robot_hands,  # noqa: E402
                     set_efforts)
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Change gear from neutral into first, then second, then third, then reverse. "
               "Leave the lever in reverse.")

# -- layout (world: x forward from the arms, y left; m) --------------------------------------------------------------
SHIFTER_XY = (0.27, -0.21)  # centre of the board, under the lever's pivot
BOARD_T = 0.012
PIVOT = np.array([0.0, 0.0, BOARD_T + M.PIVOT_Z])  # in the part frame (origin on the table top under the pivot)
KNOB_R, KNOB_H = 0.0225, 0.050

BLACK, GREY, LEATHER, SILVER = (0.05, 0.05, 0.055), (0.20, 0.21, 0.22), (0.09, 0.08, 0.075), (0.70, 0.71, 0.73)
PLY, WHITE, RED = (0.80, 0.68, 0.50), (0.92, 0.92, 0.90), (0.75, 0.10, 0.08)
VERT = frame_x((0.0, 0.0, 1.0))  # a "cyl" spec standing upright


def _vcyl(centre, d, h, color, collide=True):
    return (tuple(centre), (h, d, d), color, collide, "cyl", VERT)


ROOT = [
    ((0.0, 0.0, BOARD_T / 2), (0.30, 0.25, BOARD_T), PLY),  # plywood mounting board
    ((0.0, 0.0, BOARD_T + 0.030), (0.146, 0.150, 0.060), BLACK),  # shifter body
    ((0.083, 0.0, BOARD_T + 0.018), (0.020, 0.110, 0.036), GREY),  # front: the desk clamp's jaw housing (unused)
    ((0.0, 0.0, BOARD_T + 0.064), (0.110, 0.110, 0.008), GREY),  # top bezel
    ((0.0, 0.0, BOARD_T + 0.072), (0.075, 0.075, 0.010), LEATHER, False),  # leather boot (drawn; hides the gate)
]
YOKE = Slide(name="select", origin=tuple(PIVOT), axis=(1.0, 0.0, 0.0),
             boxes=(((0.0, 0.0, 0.0), (0.02, 0.02, 0.01), GREY),), lower=-(M.P + M.C) / M.L,
             upper=(M.P + M.C) / M.L, stiffness=0.0, damping=0.01, mass=0.03, revolute=True, armature=2e-4)
ROD_LEN = M.L - KNOB_H / 2
LEVER = Slide(name="shift", origin=tuple(PIVOT), axis=(0.0, 1.0, 0.0),
              boxes=(_vcyl((0.0, 0.0, ROD_LEN / 2), 0.012, ROD_LEN, SILVER),), lower=-M.U / M.L, upper=M.U / M.L,
              stiffness=0.0, damping=0.01, mass=0.08, revolute=True, parent="select", armature=2e-4)
# the knob's top carries the gear pattern, as the real knob does
_T = KNOB_H / 2 + 0.0004
PATTERN = [((0.0, s * 0.011, _T), (0.022, 0.0016, 0.0008), WHITE, False) for s in (-1, 0, 1)]
PATTERN += [((0.0, 0.0, _T), (0.0016, 0.024, 0.0008), WHITE, False),
            ((-0.013, -0.011, _T), (0.004, 0.004, 0.0008), RED, False)]  # R mark at the 6 slot
KNOB = Slide(name="press", origin=tuple(PIVOT + (0.0, 0.0, M.L)), axis=(0.0, 0.0, 1.0),
             boxes=(_vcyl((0.0, 0.0, 0.0), 2 * KNOB_R, KNOB_H, LEATHER), *PATTERN), lower=-M.PRESS_MAX, upper=0.0,
             stiffness=0.0, damping=2.0, mass=0.12, parent="shift", armature=0.01)
SHIFTER = part("shifter", (*SHIFTER_XY, TOP), ROOT, mass=1.2, color=BLACK, friction=1.0, slides=(YOKE, LEVER, KNOB),
               fixed=True)
OBJECTS = [SHIFTER]

# -- run-time mechanics ---------------------------------------------------------------------------------------------
DT = 1.0 / 240.0
DEBUG = bool(os.environ.get("SHIFT_DEBUG"))
_RT: dict = {}


def _reset(environment=None) -> None:
    _RT.update(steps=0, a=0.0, b=0.0, press=0.0, u=0.0, v=0.0, gear=None, box=M.Gearbox(), seq=M.Sequence(),
               limits=None, hands={}, max_press=0.0)
    if environment is not None:
        _write_limits(environment, M.gate_limits(0.0, 0.0))


on_reset(_reset)
_reset()


def _write_limits(environment, lim) -> None:
    import torch

    asset = environment.unwrapped.scene["shifter"]
    ids = _jids(asset, _SPECS["shifter"])[:2]
    t = torch.tensor([[list(lim[0]), list(lim[1])]], dtype=torch.float32, device=asset.device)
    asset.write_joint_position_limit_to_sim_index(limits=t, joint_ids=ids, warn_limit_violation=False)
    _RT["limits"] = lim


@on_physics
def _mechanics(environment) -> None:
    asset = environment.unwrapped.scene["shifter"]
    q = _np(asset.data.joint_pos)[0, _jids(asset, _SPECS["shifter"])]
    a, b, press = (float(x) for x in q)
    u, v = M.uv(a, b)
    lim = M.gate_limits(u, v)
    if lim != _RT["limits"]:
        _write_limits(environment, lim)
    ta, tb, fp = M.efforts(a, b, press)
    set_efforts(environment, "shifter", {"select": ta, "shift": tb, "press": fp})
    gear = _RT["box"].read(u, v, press)
    _RT["steps"] += 1
    t = _RT["steps"] * DT
    _RT["seq"].feed(gear, DT, t)
    _RT.update(a=a, b=b, press=press, u=u, v=v, gear=gear, max_press=max(_RT["max_press"], -press))
    if DEBUG and _RT["steps"] % 24 == 0:
        print(f"[shift] t {t:6.2f} u {u * 1000:6.1f} v {v * 1000:6.1f} press {press * 1000:5.1f} gear {gear} "
              f"done {_RT['seq'].done} wrong {_RT['seq'].wrong}", flush=True)


@on_step
def _hands(environment) -> None:
    _RT["hands"] = robot_hands(environment)


def knob_world(a: float, b: float, press: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """The knob centre (world) and the lever's rotation for joint values (a, b, press)."""
    off, r = M.knob_offset(a, b, press)
    return np.array([SHIFTER_XY[0], SHIFTER_XY[1], TOP]) + PIVOT + off, r


# -- scoring --------------------------------------------------------------------------------------------------------


def current() -> float:
    return _RT["seq"].score(_RT["gear"])


def score(objects) -> float:
    """0.25 each for first, second and third reached in that order, each held engaged (90 % into its slot) for 1 s;
    0.25 more for reverse reached after them and still engaged at the end. Any other gear engaged for 0.25 s (4th,
    5th, 6th instead of reverse, or going back to an earlier gear) stops all further credit; what was earned before
    it stays. Reverse is the 6 slot entered with the knob pushed down 5 mm or more."""
    return current()


def success(objects) -> bool:
    return current() == 1.0


# -- scripted demo -----------------------------------------------------------------------------------------------


def demo_controller(bases, dt):
    from shiftdemo import Controller

    return Controller(bases, dt, sim=_RT, cheat=os.environ.get("DEMO_CHEAT", ""))


def demo():
    """Placeholder so tools that look for a demo() find one; yam_scripted runs demo_controller instead."""
    return {}

