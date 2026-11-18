"""On the station table: a trailer's front wall (an upright grey-painted plywood panel) across the far middle of
the table, its 7-way electrical socket (round zinc socket, SAE J560) facing the robot 13 cm above the table, with its
sprung lid shut over the mouth. The lid's thumb tab sticks out at its lower edge. To the right, in front of the right
arm, the tractor's 7-way plug (zinc, about 5.4 cm across, a key rib along the top of its insert) lies in a wooden
V-block with its cable trailing to a hook on the right; it lies rolled so its key is not at 12 o'clock.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
The plug's start roll comes from the seed PLUG_SEED (environment variable, default 0).
"""

from __future__ import annotations

import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import plugmodel as M  # noqa: E402
from partkit import Slide, matrix_to_quat, on_physics, on_reset, on_step, part, robot_hands  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
assert abs(TOP - M.TOP) < 1e-9
INSTRUCTION = "Plug the trailer cable into the 7-way socket on the trailer front and leave it latched. Then take both grippers away."

SEED = int(os.environ.get("PLUG_SEED", "0"))
ZINC, DARK, WALL, WOOD, BLACK = (0.62, 0.63, 0.62), (0.16, 0.16, 0.17), (0.55, 0.57, 0.58), (0.72, 0.58, 0.40), (0.05, 0.05, 0.05)
LID = (0.70, 0.71, 0.70)
Q = matrix_to_quat
TAN15 = float(np.tan(np.radians(15.0)))


def ngon(apothem, x0, x1, color, centre=(0.0, 0.0)):
    """A 12-sided prism along x from x0 to x1 (its faces at apothem), as six crossed slabs."""
    out = []
    for k in range(6):
        a = np.radians(30.0 * k)
        out.append((((x0 + x1) / 2, centre[0], centre[1]), (x1 - x0, 2 * apothem, 2 * apothem * TAN15), color, True,
                    Q(M.rot_x(np.degrees(a)))))
    return out


def ring(apothem_in, apothem_out, x0, x1, color, skip=()):
    """A 12-sided tube wall along x round the local x axis; segment k faces 30k degrees from +z (k = 0 at 12
    o'clock). Segments in ``skip`` are left out (a slot)."""
    t, mid = apothem_out - apothem_in, (apothem_in + apothem_out) / 2
    out = []
    for k in range(12):
        if k in skip:
            continue
        a = np.radians(30.0 * k)
        r = M.rot_x(np.degrees(a))
        c = r @ np.array([0.0, 0.0, mid])
        out.append((((x0 + x1) / 2, c[1], c[2]), (x1 - x0, 2 * apothem_out * TAN15 + 0.001, t), color, True, Q(r)))
    return out


# -- socket on the trailer wall: barrel (key slot at 12 o'clock), pin face, flange, hinge boss; sprung lid ------------
L = M.BARREL_LEN
SOCKET_BOXES = (
    *ring(M.BARREL_IN, M.BARREL_OUT, 0.0, L, ZINC, skip=(0,)),
    ((L + 0.003, 0.0, 0.0), (0.006, 0.080, 0.080), ZINC),  # flange and pin face
    ((L - 0.002, 0.0, 0.0), (0.002, 0.040, 0.040), DARK, False),  # pin face insulator (drawn)
    ((0.0155, 0.0, 0.039), (0.029, 0.040, 0.018), ZINC),  # hinge boss over the barrel
)
_h = M.HINGE - np.array([M.XF, M.YS, M.ZC])
LID_SLIDE = Slide(name="lid", origin=tuple(_h), axis=(0.0, 1.0, 0.0), boxes=(
    ((0.002, 0.0, -0.0355), (0.004, 0.064, 0.079), LID),  # lid plate
    ((0.006, 0.0, -0.072), (0.004, 0.024, 0.006), LID),  # catch on its inner face
    ((-0.0125, 0.0, -0.0725), (0.025, 0.030, 0.005), LID),  # thumb tab
), lower=0.0, upper=M.LID_MAX, stiffness=M.LID_K, damping=0.004, mass=0.06, revolute=True, target=M.LID_T,
    armature=0.0002)
SOCKET = part("socket", (M.XF, M.YS, M.ZC), SOCKET_BOXES, mass=1.0, friction=0.25, slides=(LID_SLIDE,), fixed=True)

# -- plug: insert with key rib, body with the catch ridge on top, cable boot (drawn) ----------------------------------
PLUG_BOXES = (
    *ngon(M.NOSE_R, -M.NOSE_LEN, 0.0, DARK),
    ((-M.NOSE_LEN / 2, 0.0, sum(M.RIB[2]) / 2), (M.NOSE_LEN, M.RIB[1], M.RIB[2][1] - M.RIB[2][0]), DARK),
    *ngon(M.BODY_R, M.BODY_END, -M.NOSE_LEN, ZINC),
    ((sum(M.RIDGE_X) / 2, 0.0, (M.RIDGE_TOP + M.BODY_R - 0.0005) / 2),
     (M.RIDGE_X[1] - M.RIDGE_X[0], M.RIDGE_W, M.RIDGE_TOP - M.BODY_R + 0.0005), ZINC),
    ((M.BODY_END - 0.045, 0.0, 0.0), (0.090, 0.020, 0.020), BLACK, False),  # spring cable guard and cable (drawn)
)
P0, R0 = M.start_pose(SEED)
PLUG = part("plug", P0, PLUG_BOXES, mass=M.PLUG_MASS, quat=Q(R0), friction=0.8, com=(-0.12, 0.0, 0.0),
            inertia=(0.00013, 0.0011, 0.0011))

# -- trailer wall (18 mm plywood on a foot), V-block, cable hook post -------------------------------------------------
WX = M.XF + L + 0.006


def _box(name, lo, hi, color):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return Box(name, pos=tuple((lo + hi) / 2), size=tuple(hi - lo), color=color)


WALL_BOXES = [
    _box("wall", (WX, M.YS - 0.15, TOP), (WX + 0.018, M.YS + 0.15, TOP + 0.30), WALL),
    _box("wall_foot", (WX + 0.018, M.YS - 0.15, TOP), (WX + 0.12, M.YS + 0.15, TOP + 0.018), WALL),
]
VX, VY = M.VBLOCK_XY
_vs = 0.045  # slab thickness
_vl = 0.12
VBLOCK = part("vblock", (VX, VY, M.V_FLOOR), (
    ((0.0, -0.011 - 0.5 * _vs * np.sqrt(0.5), 0.011 - 0.5 * _vs * np.sqrt(0.5)), (_vl, 0.0311, _vs), WOOD, True,
     Q(M.rot_x(-45.0))),
    ((0.0, 0.011 + 0.5 * _vs * np.sqrt(0.5), 0.011 - 0.5 * _vs * np.sqrt(0.5)), (_vl, 0.0311, _vs), WOOD, True,
     Q(M.rot_x(45.0))),
    ((0.0, 0.0, -0.0075), (_vl, 0.100, 0.015), WOOD),
    ((0.0, -0.039, 0.011), (_vl, 0.022, 0.022), WOOD), ((0.0, 0.039, 0.011), (_vl, 0.022, 0.022), WOOD),
), mass=1.0, fixed=True, friction=0.6)
HOOK = [_box("hook_post", (M.CORD_HOOK[0] - 0.015, M.CORD_HOOK[1] - 0.015, TOP), (M.CORD_HOOK[0] + 0.015,
             M.CORD_HOOK[1] + 0.015, M.CORD_HOOK[2] + 0.02), DARK)]

OBJECTS = [*WALL_BOXES, SOCKET, VBLOCK, *HOOK, PLUG]

# -- run-time mechanics: the split pins' grip and the cable's pull on the plug (each physics step) --------------------
PHYS_DT = 1 / 240
DT = 1 / 20  # the step hook runs once per control step (-E control_hz=20)
MON = M.Monitor(dt=DT)
_RT: dict = {}
FREE_OPEN = 0.080  # a gripper opened wider than this (m) holds nothing
HOLD_R = 0.040  # tool point this close to a part's grip zone, fingers not wide open: holding it


def _reset(environment=None) -> None:
    MON.reset()
    _RT.clear()
    _RT.update(steps=0, states={}, hands={})


on_reset(_reset)
_reset()


@on_physics
def _pins(environment) -> None:
    import torch
    from partkit import _np, quat_to_matrix

    base = environment.unwrapped
    plug = base.scene["plug"]
    pose = _np(plug.data.root_link_pose_w)[0]
    vel = _np(plug.data.root_lin_vel_w)[0]
    pos = pose[:3] - _np(base.scene.env_origins)[0]
    rot = quat_to_matrix(pose[3:7])
    depth, off, _ = M.depth_of(pos, rot)
    cord = M.cord_force(pos)
    f = cord.copy()
    if off < 0.006:
        f[0] += M.axial_friction(depth, float(vel[0]), float(cord[0]), M.PLUG_MASS, PHYS_DT)
    _RT["pin_force"] = float(f[0] - cord[0])
    forces = torch.tensor([[f.tolist()]], dtype=torch.float32, device=plug.device)
    plug.permanent_wrench_composer.set_forces_and_torques_index(forces=forces, torques=torch.zeros_like(forces),
                                                                 is_global=True)


def _held(hands, point, radius) -> str:
    for side, (tcp, _, opening) in hands.items():
        if opening < FREE_OPEN and np.linalg.norm(tcp - point) < radius:
            return side
    return ""


def tab_point(lid: float) -> np.ndarray:
    return M.HINGE + M.rot_y(lid) @ np.array([-0.0125, 0.0, -0.073])


@on_step
def _track(environment) -> None:
    from yam_arena.scene import object_states

    s = object_states(environment, ["plug", "socket"])
    hands = robot_hands(environment)
    p = s["plug"]
    R = p.rotation()
    depth, off, roll = M.depth_of(p.pos, R)
    lid = float(s["socket"].slides[0])
    _RT["steps"] += 1
    MON.step(depth, off, float(p.low[2]), lid)
    plug_held = _held(hands, p.pos + R @ np.array([-0.120, 0.0, 0.0]), 0.06)
    lid_held = _held(hands, tab_point(lid), HOLD_R)
    _RT.update(states=s, hands=hands, R=R, depth=depth, off=off, roll=roll, lid=lid, plug_held=plug_held,
               lid_held=lid_held)
    if os.environ.get("PLUG_DEBUG") and _RT["steps"] % 4 == 0:
        print(f"[plug] t {_RT['steps'] * DT:6.2f} pos {np.round(p.pos, 3)} depth {depth * 1000:6.1f} off "
              f"{off * 1000:5.1f} roll {roll:6.1f} lid {np.degrees(lid):6.1f} pins {_RT.get('pin_force', 0):6.1f} "
              f"held P {plug_held or '-'} L {lid_held or '-'} stage {stage():.2f} open L {hands['left'][2] * 1000:.0f} "
              f"R {hands['right'][2] * 1000:.0f}", flush=True)


# -- scoring ---------------------------------------------------------------------------------------------------------


def stage() -> float:
    if not _RT.get("states"):
        return 0.0
    return MON.score(_RT["depth"], _RT["lid"], bool(_RT["plug_held"]), bool(_RT["lid_held"]))


def score(objects) -> float:
    """0 if the plug fell back to the table after it was lifted. Otherwise, the highest that holds: 1.0 the plug is
    seated (insert within 3 mm of the pin face), nothing holds it or the lid, and the lid is down in the notch behind
    the plug's catch ridge; 0.6 seated and released but the lid not caught; 0.3 at some point the insert went 15 mm
    into the barrel (its key in the slot); 0.1 at some point the lid was held 100 degrees or more open while the
    plug's insert face was on the axis within 1 cm of the mouth."""
    return stage()


def success(objects) -> bool:
    return stage() == 1.0


# -- scripted demo ---------------------------------------------------------------------------------------------------


def demo_controller(bases, dt):
    from plugdemo import Controller

    return Controller(bases, dt, sim=_RT, cheat=os.environ.get("DEMO_CHEAT", ""))


def demo():
    """Placeholder so tools that look for a demo() find one; yam_scripted runs demo_controller instead."""
    return {}
