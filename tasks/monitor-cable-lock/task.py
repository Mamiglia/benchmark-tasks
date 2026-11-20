"""On the station table: a 24-inch office monitor (black, Dell E2422H) on its stand, its screen facing away from the
robot, so its back is toward the arms. The security slot is on the back's lower housing, 13 cm above the desk, a
little left of the arms' centre line. In front of the monitor a keyed cable lock (Kensington MicroSaver 2.0) lies on
the desk, its T-bar toward the monitor and its key in it, in the unlocked position. Its steel cable is already looped
round the desk's leg and leads off over the desk's left edge.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 --scorer task_score ...
The lock's start position and yaw come from the seed LOCK_SEED (environment variable, default 0).
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import lockmodel as M  # noqa: E402
import numpy as np  # noqa: E402
from partkit import (Link, Slide, matrix_to_quat, on_physics, on_reset, on_step, part, robot_hands,  # noqa: E402
                     set_link, set_link_drive, set_slide_targets)
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 2.0  # locked, key out, hands off, held 2 s with the cable pulling on the head
TOP = TABLE_TOP[SCENE]
assert abs(TOP - M.TOP) < 1e-9
INSTRUCTION = "Lock the monitor with the cable lock and take the key out."

SEED = int(os.environ.get("LOCK_SEED", "0"))
BLACK, DARK, GREY, HOLE = (0.07, 0.07, 0.08), (0.12, 0.12, 0.13), (0.30, 0.30, 0.32), (0.01, 0.01, 0.01)
STEEL, BRASS, KEYCAP = (0.70, 0.71, 0.73), (0.78, 0.66, 0.38), (0.10, 0.10, 0.11)


def _b(lo, hi, color, origin, collide=True):
    """A box spec from world corners, relative to a part's origin."""
    lo, hi, o = np.asarray(lo, float), np.asarray(hi, float), np.asarray(origin, float)
    return (tuple((lo + hi) / 2 - o), tuple(hi - lo), color, collide)


# -- monitor: thin panel, lower rear housing with the security slot and the pocket behind it, upper housing, stand ----
O = (M.BACK_X, M.MON_Y, TOP)
Y0, Y1, HZ0, HZ1 = M.HOUSING
SY, SZ = M.SLOT_Y, M.SLOT_Z
CY, CZ, CD = M.CAVITY[0] / 2, M.CAVITY[1] / 2, M.CAVITY[2]
X0, XW, XC = M.BACK_X, M.BACK_X + M.WALL, M.BACK_X + M.WALL + CD  # back face, wall's back, pocket's end
W2, H2 = M.SLOT_W / 2, M.SLOT_H / 2
MONITOR_BOXES = (
    _b((M.SLAB_X0, M.MON_Y - M.PANEL_W / 2, M.PB), (M.SCREEN_X, M.MON_Y + M.PANEL_W / 2, M.PB + M.PANEL_H), BLACK, O),
    # lower rear housing, around the pocket behind the slot
    _b((X0, Y0, HZ0), (M.SLAB_X0, SY - CY, HZ1), BLACK, O),
    _b((X0, SY + CY, HZ0), (M.SLAB_X0, Y1, HZ1), BLACK, O),
    _b((X0, SY - CY, HZ0), (M.SLAB_X0, SY + CY, SZ - CZ), BLACK, O),
    _b((X0, SY - CY, SZ + CZ), (M.SLAB_X0, SY + CY, HZ1), BLACK, O),
    _b((XC, SY - CY, SZ - CZ), (M.SLAB_X0, SY + CY, SZ + CZ), BLACK, O),
    # the wall with the 7 x 3 mm slot
    _b((X0, SY - CY, SZ - CZ), (XW, SY - W2, SZ + CZ), BLACK, O),
    _b((X0, SY + W2, SZ - CZ), (XW, SY + CY, SZ + CZ), BLACK, O),
    _b((X0, SY - W2, SZ + H2), (XW, SY + W2, SZ + CZ), BLACK, O),
    _b((X0, SY - W2, SZ - CZ), (XW, SY + W2, SZ - H2), BLACK, O),
    _b((XC - 0.0005, SY - CY, SZ - CZ), (XC, SY + CY, SZ + CZ), HOLE, O, False),  # dark inside of the pocket
    # small padlock mark beside the slot (moulded on the real housing)
    _b((X0 - 0.0003, SY + 0.007, SZ - 0.002), (X0, SY + 0.010, SZ + 0.002), GREY, O, False),
    # upper rear housing (VESA mount, stand attachment)
    _b((X0 + 0.005, M.MON_Y - 0.10, HZ1), (M.SLAB_X0, M.MON_Y + 0.10, M.PB + 0.25), BLACK, O),
    # stand: neck behind the housing, base plate
    _b((X0 - 0.030, M.MON_Y - 0.025, TOP + 0.010), (X0 + 0.005, M.MON_Y + 0.025, M.PB + 0.22), DARK, O),
    _b((X0 - 0.120, M.MON_Y - 0.111, TOP), (X0 + 0.051, M.MON_Y + 0.111, TOP + 0.010), DARK, O),
)
# centre of mass: panel (3.55 kg) at its middle, stand (0.35 kg) low behind it
_COM = (np.array([M.BACK_X + 0.030, M.MON_Y, M.PB + M.PANEL_H / 2]) * 3.55
        + np.array([M.BACK_X - 0.030, M.MON_Y, TOP + 0.04]) * 0.35) / 3.9 - np.asarray(O)
MONITOR = part("monitor", O, MONITOR_BOXES, mass=M.MASS, friction=0.6, com=tuple(_COM),
               inertia=(0.13, 0.045, 0.095))

# -- lock: head (nose, barrel, fixed side pins, cable collar), the T-bar turning with the key, the key ---------------
_q45 = (math.sin(math.pi / 8), 0.0, 0.0, math.cos(math.pi / 8))
_q90 = (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4))
_q135 = (math.sin(3 * math.pi / 8), 0.0, 0.0, math.cos(3 * math.pi / 8))
_bc = (-(M.NOSE + M.BARREL_L / 2), 0.0, 0.0)
_bs = (M.BARREL_L, M.BARREL_D, 0.0075)
_py = (M.PIN_Y0 + M.PIN_Y1) / 2
HEAD_BOXES = (
    ((-M.NOSE / 2, 0.0, 0.0), (M.NOSE, 0.010, 0.010), STEEL),
    (_bc, _bs, STEEL, True, None), (_bc, _bs, STEEL, True, _q45), (_bc, _bs, STEEL, True, _q90),
    (_bc, _bs, STEEL, True, _q135),  # barrel: about 18 mm across
    ((M.PIN_X / 2, _py, 0.0), (M.PIN_X, M.PIN_Y1 - M.PIN_Y0, M.PIN_Z), STEEL),
    ((M.PIN_X / 2, -_py, 0.0), (M.PIN_X, M.PIN_Y1 - M.PIN_Y0, M.PIN_Z), STEEL),
    ((M.BACK + 0.0035, 0.0, 0.0), (0.007, 0.021, 0.021), BLACK),  # cable collar
    ((M.BACK + 0.0035, 0.032, 0.0), (0.0045, 0.042, 0.0045), STEEL, False),  # the cable, leading off (drawn)
)
TBAR = Slide(name="tbar", origin=(0.0, 0.0, 0.0), axis=(1.0, 0.0, 0.0), revolute=True, lower=0.0,
             upper=M.LOCKED_ANGLE, stiffness=M.DETENT_K, damping=M.DETENT_D, mass=M.TBAR_MASS, armature=2e-6,
             boxes=(((M.BAR_X0 / 2 - 0.001, 0.0, 0.0), (M.BAR_X0 + 0.002, M.STEM, M.STEM), STEEL),
                    (((M.BAR_X0 + M.BAR_TIP) / 2, 0.0, 0.0), (M.BAR_TIP - M.BAR_X0, M.BAR_L, M.BAR_T), STEEL),
                    (((M.BAR_TIP + M.BAR_X1) / 2, 0.0, 0.0),
                     (M.BAR_X1 - M.BAR_TIP, M.BAR_L - 0.0016, M.BAR_T - 0.0008), STEEL)))  # the crossbar's rounded tip, drawn as a slightly smaller box
P0, R0 = M.start_pose(SEED)
LOCK = part("lock", P0, HEAD_BOXES, mass=M.HEAD_MASS, quat=matrix_to_quat(R0), friction=0.6, slides=(TBAR,),
            contact_offset=0.0005, com=(-0.020, 0.0, 0.0), inertia=(1.6e-6, 6e-6, 6e-6))

# The key sits in the lock's cylinder: a joint that turns it with the T-bar and lets it slide out along the lock's
# axis only in the locked position (stiff drive while unlocked: the key is captive), off once it is out.
_FLIP = ((-1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, 1.0))  # frame x = out of the lock
KEYWAY = Link(name="keyway", to="lock/tbar", pos0=(0.0, 0.0, 0.0), rot0=_FLIP, rot1=_FLIP, slide=True, lower=0.0,
              upper=0.03, stiffness=1e5, damping=50.0, max_force=1e4, enabled=True)
KEY_BOXES = (
    # the shank, outside and inside the lock (drawn: it never touches anything but the lock's cylinder)
    (((M.SHANK[0] + M.BACK + 0.012) / 2, 0.0, 0.0), (M.BACK + 0.012 - M.SHANK[0], M.SHANK[1], M.SHANK[1]), STEEL,
     False),
    (((M.BOW[0] + M.BOW[1]) / 2, 0.0, 0.0), (M.BOW[1] - M.BOW[0], M.BOW[2], M.BOW[3]), KEYCAP),
)
KEY = part("key", P0, KEY_BOXES, mass=M.KEY_MASS, quat=matrix_to_quat(R0), friction=0.8, links=(KEYWAY,),
           contact_offset=0.0005, com=(M.BOW[0] + 0.012, 0.0, 0.0), inertia=(3e-7, 4e-7, 6e-7))
OBJECTS = [MONITOR, LOCK, KEY]

# -- run-time mechanics: detents, the captive key, the cable's pull (each physics step) -------------------------------
TRACK = M.Tracker()
_RT: dict = {}


def _reset(environment=None) -> None:
    TRACK.reset()
    _RT.clear()
    _RT.update(steps=0, states={}, hands={}, key_in=True, key_free=False, target=0.0, mon0=None)
    if environment is not None:
        set_link("keyway", True)
        set_link_drive("keyway", target=0.0, stiffness=1e5, damping=50.0, max_force=1e4)


on_reset(_reset)
_reset()


@on_physics
def _mechanics(environment) -> None:
    import torch
    from partkit import _np, quat_to_matrix

    base = environment.unwrapped
    lock = base.scene["lock"]
    key = base.scene["key"]
    origin = _np(base.scene.env_origins)[0]
    angle = float(_np(lock.data.joint_pos)[0, 0])
    target = M.detent_target(angle)
    if target != _RT["target"]:
        set_slide_targets(environment, "lock", {"tbar": target})
        _RT["target"] = target
    if _RT["key_in"]:
        free = angle > M.KEY_FREE_ANGLE
        if free != _RT["key_free"]:  # locked: the key sits on a stiff spring; a 3 N pull stretches it KEY_GIVE
            set_link_drive("keyway", stiffness=M.KEY_K if free else 1e5, damping=20.0 if free else 50.0,
                           max_force=1e4)
            _RT["key_free"] = free
        kp = _np(key.data.root_link_pose_w)[0]
        hp = _np(lock.data.root_link_pose_w)[0]
        out = float(-(quat_to_matrix(hp[3:7])[:, 0] @ (kp[:3] - hp[:3])))
        if free and out > M.KEY_GIVE:  # pulled with 3 N or more: the key comes out, and follows the hand
            set_link("keyway", False)
            _RT["key_in"] = False
    pose = _np(lock.data.root_link_pose_w)[0]
    pos = pose[:3] - origin
    f = M.cord_force(pos)
    at = pose[:3] + quat_to_matrix(pose[3:7]) @ np.array([M.BACK + 0.0035, 0.0, 0.0])
    forces = torch.tensor([[f.tolist()]], dtype=torch.float32, device=lock.device)
    lock.permanent_wrench_composer.set_forces_and_torques_index(
        forces=forces, torques=torch.zeros_like(forces), positions=torch.tensor([[at.tolist()]], dtype=torch.float32,
                                                                               device=lock.device),
        body_ids=[int(lock.find_bodies("body")[0][0])], is_global=True)


@on_step
def _track(environment) -> None:
    from yam_arena.scene import object_states

    s = object_states(environment, ["monitor", "lock", "key"])
    hands = robot_hands(environment)
    mon, lk = s["monitor"], s["lock"]
    if _RT["mon0"] is None:
        _RT["mon0"] = mon.pos.copy()
    mon_pos0 = np.asarray(O)
    depth, off, tilt = M.seat_of(lk.pos, lk.rotation(), mon.pos, mon.rotation(), mon_pos0)
    moved = M.monitor_moved(mon.pos, mon.rotation(), _RT["mon0"])
    angle = math.degrees(float(lk.slides[0]))
    centre = lk.to_world((M.BACK / 2, 0.0, 0.0))
    clear = all(float(np.linalg.norm(tcp - centre)) >= M.CLEAR for tcp, _, _ in hands.values())
    TRACK.step(depth, off, tilt, moved)
    _RT["steps"] += 1
    _RT.update(states=s, hands=hands, depth=depth, off=off, tilt=tilt, angle=angle, clear=clear)
    if os.environ.get("LOCK_DEBUG") and _RT["steps"] % 4 == 0:
        print(f"[lock] t {_RT['steps'] / 20:6.2f} head {np.round(lk.pos, 4)} depth {depth * 1000:6.2f} off "
              f"{off * 1000:5.2f} tilt {tilt:5.1f} angle {angle:5.1f} key_in {_RT['key_in']} mon "
              f"{np.round((mon.pos - _RT['mon0']) * 1000, 1)} clear {clear} stage {stage():.2f}", flush=True)


# -- scoring ---------------------------------------------------------------------------------------------------------


def stage() -> float:
    if not _RT.get("states"):
        return 0.0
    return TRACK.score(_RT["depth"], _RT["off"], _RT["tilt"], _RT["angle"], not _RT["key_in"], _RT["clear"])


def score(objects) -> float:
    """0 once the monitor slid more than 3 cm, tipped more than 10 degrees or left the desk (for the rest of the
    trial). Otherwise at the end: 1.0 locked (head flush on the monitor's back, T-bar through the slot and turned
    crosswise), the key out and both hands 4 cm or more from the head; 0.8 the same with a hand still at the head; 0.6 locked, key still in; 0.3 T-bar through the slot, head flush, not turned; 0.1 the
    T-bar went through the slot at some point but is not there at the end; else 0."""
    return stage()


def success(objects) -> bool:
    return stage() == 1.0


# -- scripted demo ---------------------------------------------------------------------------------------------------


def demo_controller(bases, dt):
    from lockdemo import Controller

    return Controller(bases, dt, sim=_RT, cheat=os.environ.get("DEMO_CHEAT", ""))


def demo():
    """Placeholder so tools that look for a demo() find one; yam_scripted runs demo_controller instead."""
    return {}
