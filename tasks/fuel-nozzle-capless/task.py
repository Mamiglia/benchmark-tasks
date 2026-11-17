"""On the station table, a car's rear quarter panel (painted steel, silver blue) stands across the far left of the
table, its fuel door open: behind it, in a recessed pocket, the round face of a capless filler, whose bore runs into
the car and down. To its right a dark grey pump post holds a dry automatic fuel nozzle (aluminium, black handle and
trigger lever, steel spout) spout-down in its boot; a black hose runs from the nozzle's back end to the pump's hose
outlet on a mast at the right front corner. The nozzle weighs 1.3 kg. Its trigger lever sits under the handle inside
a guard; a small hold-open clip on the back of the guard can be flipped up under the squeezed lever to hold it open.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import nozzlemodel as M  # noqa: E402
from nozzlekit import Link, Slide, matrix_to_quat, on_reset, on_step, part, robot_hands  # noqa: E402
from nozzlekit import set_slide_targets  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
assert abs(TOP - M.TOP) < 1e-9
INSTRUCTION = ("Fill the car: put the nozzle in the filler, squeeze the trigger and set the hold-open clip, then "
               "release the trigger and hang the nozzle back up.")

PAINT, BLACK, ALU, STEEL = (0.55, 0.62, 0.70), (0.06, 0.06, 0.07), (0.70, 0.71, 0.73), (0.50, 0.51, 0.53)
PUMP, RED, POCKET = (0.25, 0.26, 0.28), (0.75, 0.12, 0.10), (0.12, 0.12, 0.13)
Q = lambda r: matrix_to_quat(r)  # noqa: E731
RY = M.rot_y


def RZ(deg):
    c, s_ = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])

# -- nozzle --------------------------------------------------------------------------------------------------------
NOZZLE_BOXES = (
    (tuple(M.SPOUT_DIR * M.SPOUT_LEN / 2), (M.SPOUT_LEN, M.SPOUT_W, M.SPOUT_W), STEEL, True, Q(RY(M.SPOUT_DEG))),
    ((-0.050, 0.0, 0.005), (0.100, 0.045, 0.060), ALU),  # valve housing
    ((-0.165, 0.0, 0.020), (0.130, 0.032, 0.025), BLACK),  # handle tube (insulated grip)
    ((-0.245, 0.0, 0.020), (0.034, 0.034, 0.034), ALU),  # inlet swivel
    ((-0.163, 0.0, -0.085), (0.150, 0.030, 0.008), BLACK),  # guard, bottom bar
    ((-0.236, 0.0, -0.035), (0.008, 0.030, 0.092), BLACK),  # guard, rear post
    ((-0.094, 0.0, -0.055), (0.008, 0.030, 0.060), BLACK),  # guard, front post
)
LEVER = Slide(name="lever", origin=tuple(M.LEVER_PIVOT), axis=(0.0, 1.0, 0.0),
              boxes=(((-0.065, 0.0, -0.025), (0.120, 0.022, 0.010), ALU),), lower=0.0, upper=M.LEVER_MAX,
              stiffness=M.LEVER_REST_K, damping=0.02, mass=0.05, revolute=True, target=M.LEVER_REST_T,
              armature=0.0005)
CLIP = Slide(name="clip", origin=tuple(M.CLIP_PIVOT), axis=(0.0, 1.0, 0.0),
             boxes=(((-0.028, 0.0, 0.003), (0.050, 0.012, 0.006), RED), ((0.0, 0.0, -0.020), (0.010, 0.014, 0.040), RED)),
             lower=0.0, upper=M.CLIP_MAX, stiffness=M.CLIP_K, damping=0.002, mass=0.01, revolute=True,
             target=M.CLIP_T, armature=0.0005)

# -- hose: 8 rigid links on ball joints from the pump's hose outlet to the nozzle's swivel ----------------------------
R_N0 = M.R_BOOT
SWIVEL0 = M.BOOT_N + R_N0 @ M.SWIVEL
HOSE_PTS = M.hose_path(M.HOSE_ANCHOR, SWIVEL0)
HOSE_R = [M.frame_along(HOSE_PTS[i + 1] - HOSE_PTS[i]) for i in range(M.HOSE_N)]
HOSE_K, HOSE_C = 0.25, 0.01  # N m/rad and N m s/rad per joint (bending stiffness of a 3/4 in dispenser hose, approx.)


def _rows(m):
    return tuple(tuple(float(v) for v in row) for row in m)


def _hose_link(i):
    if i == 0:
        return Link(name="hose_link_0", to="hose_outlet", pos0=(0.0, 0.0, 0.0), rot0=_rows(HOSE_R[0]),
                    pos1=(-M.HOSE_SEG / 2, 0.0, 0.0), ball=True, stiffness=HOSE_K, damping=HOSE_C, enabled=True)
    return Link(name=f"hose_link_{i}", to=f"hose_{i - 1}", pos0=(M.HOSE_SEG / 2, 0.0, 0.0),
                rot0=_rows(HOSE_R[i - 1].T @ HOSE_R[i]), pos1=(-M.HOSE_SEG / 2, 0.0, 0.0), ball=True,
                stiffness=HOSE_K, damping=HOSE_C, enabled=True)


HOSE = [part(f"hose_{i}", (HOSE_PTS[i] + HOSE_PTS[i + 1]) / 2, (((0.0, 0.0, 0.0), (M.HOSE_SEG + 0.004, M.HOSE_W, M.HOSE_W),
             BLACK),), mass=M.HOSE_SEG_MASS, quat=Q(HOSE_R[i]), friction=0.6, links=(_hose_link(i),))
        for i in range(M.HOSE_N)]
NOZZLE = part("nozzle", M.BOOT_N, NOZZLE_BOXES, mass=M.NOZZLE_MASS, quat=Q(R_N0), friction=0.9, slides=(LEVER, CLIP),
              links=(Link(name="hose_to_nozzle", to=f"hose_{M.HOSE_N - 1}", pos0=(M.HOSE_SEG / 2, 0.0, 0.0),
                          rot0=_rows(HOSE_R[-1].T @ R_N0), pos1=tuple(M.SWIVEL), ball=True, stiffness=HOSE_K,
                          damping=HOSE_C, enabled=True),),
              com=(-0.08, 0.0, 0.0), inertia=(0.0014, 0.0144, 0.0136))
_mast_h = M.HOSE_ANCHOR[2] - TOP
OUTLET = part("hose_outlet", M.HOSE_ANCHOR, (((0.0, 0.0, 0.0), (0.05, 0.05, 0.05), PUMP),
                                              ((0.03, 0.0, -_mast_h / 2), (0.03, 0.03, _mast_h), PUMP)),
              mass=1.0, fixed=True)

# -- filler: capless head and bore (filler frame F), sprung flap inside -----------------------------------------------
W, IN, L = 0.006, M.BORE_IN, M.BORE_LEN
FILLER_BOXES = (
    ((L / 2, 0.0, IN / 2 + W / 2), (L, IN + 2 * W, W), POCKET),
    ((L / 2, 0.0, -IN / 2 - W / 2), (L, IN + 2 * W, W), POCKET),
    ((L / 2, IN / 2 + W / 2, 0.0), (L, W, IN), POCKET),
    ((L / 2, -IN / 2 - W / 2, 0.0), (L, W, IN), POCKET),
    # the head's face: a ring 66 mm across, 40 mm opening, 12 mm in front of the bore
    ((-0.014, 0.0, 0.0265), (0.004, 0.066, 0.013), ALU), ((-0.014, 0.0, -0.0265), (0.004, 0.066, 0.013), ALU),
    ((-0.014, 0.0265, 0.0), (0.004, 0.013, 0.040), ALU), ((-0.014, -0.0265, 0.0), (0.004, 0.013, 0.040), ALU),
    # its tapered lead-in, from the 40 mm opening down to the 24 mm bore
    ((-0.0049, 0.0, 0.0177), (0.0144, 0.040, 0.004), ALU, True, Q(RY(33.7))),
    ((-0.0049, 0.0, -0.0177), (0.0144, 0.040, 0.004), ALU, True, Q(RY(-33.7))),
    ((-0.0049, 0.0177, 0.0), (0.0144, 0.004, 0.040), ALU, True, Q(RZ(-33.7))),
    ((-0.0049, -0.0177, 0.0), (0.0144, 0.004, 0.040), ALU, True, Q(RZ(33.7))),
)
FLAP = Slide(name="flap", origin=(M.FLAP_AT, 0.0, IN / 2), axis=(0.0, -1.0, 0.0),
             boxes=(((0.0, 0.0, -IN / 2 + 0.001), (0.003, IN - 0.002, IN - 0.002), BLACK),), lower=0.0, upper=1.5,
             stiffness=0.08, damping=0.002, mass=0.005, revolute=True, target=-0.6, armature=0.0002)
FILLER = part("filler", M.LIP, FILLER_BOXES, mass=1.0, quat=Q(M.R_F), friction=0.3, slides=(FLAP,), fixed=True)

# -- quarter panel with the fuel-door pocket, door open; pump post and boot -------------------------------------------
PY0, PY1, PZ1, T, H = 0.02, 0.52, TOP + 0.42, 0.012, 0.05  # panel span, top, thickness, pocket half-size
FX, FY, FZ = M.PANEL_X, M.FILLER_Y, M.FILLER_Z


def _box(name, lo, hi, color, **kw):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return Box(name, pos=tuple((lo + hi) / 2), size=tuple(hi - lo), color=color, **kw)


BACK = M.LIP[0]
PANEL = [
    _box("panel_left", (FX, FY + H, TOP), (FX + T, PY1, PZ1), PAINT),
    _box("panel_right", (FX, PY0, TOP), (FX + T, FY - H, PZ1), PAINT),
    _box("panel_above", (FX, FY - H, FZ + H), (FX + T, FY + H, PZ1), PAINT),
    _box("panel_below", (FX, FY - H, TOP), (FX + T, FY + H, FZ - H), PAINT),
    _box("pocket_top", (FX, FY - H, FZ + H - 0.004), (BACK, FY + H, FZ + H), POCKET),
    _box("pocket_bottom", (FX, FY - H, FZ - H), (BACK, FY + H, FZ - H + 0.004), POCKET),
    _box("pocket_left", (FX, FY + H - 0.004, FZ - H), (BACK, FY + H, FZ + H), POCKET),
    _box("pocket_right", (FX, FY - H, FZ - H), (BACK, FY - H + 0.004, FZ + H), POCKET),
    _box("pocket_back_top", (BACK, FY - H, FZ + 0.035), (BACK + 0.004, FY + H, FZ + H), POCKET),
    _box("pocket_back_bottom", (BACK, FY - H, FZ - H), (BACK + 0.004, FY + H, FZ - 0.035), POCKET),
    _box("pocket_back_left", (BACK, FY + 0.035, FZ - 0.035), (BACK + 0.004, FY + H, FZ + 0.035), POCKET),
    _box("pocket_back_right", (BACK, FY - H, FZ - 0.035), (BACK + 0.004, FY - 0.035, FZ + 0.035), POCKET),
    _box("fuel_door", (FX - 0.10, FY + H + 0.002, FZ - H), (FX - 0.002, FY + H + 0.006, FZ + H), PAINT),
]
POST = [_box("pump_post", (0.565, -0.11, TOP), (0.645, 0.012, TOP + 0.52), PUMP)]
BOOT_R = M.frame_along(M.BOOT_AXIS)
BI, BW, BL = M.BOOT_IN, 0.004, M.BOOT_LEN
BOOT = part("boot", M.BOOT_MOUTH, (
    ((BL / 2, 0.0, BI / 2 + BW / 2), (BL, BI + 2 * BW, BW), BLACK), ((BL / 2, 0.0, -BI / 2 - BW / 2), (BL, BI + 2 * BW, BW), BLACK),
    ((BL / 2, BI / 2 + BW / 2, 0.0), (BL, BW, BI), BLACK), ((BL / 2, -BI / 2 - BW / 2, 0.0), (BL, BW, BI), BLACK),
    ((BL + BW / 2, 0.0, 0.0), (BW, BI + 2 * BW, BI + 2 * BW), BLACK),
    # bracket from the sleeve's back to the post (drawn only)
    ((0.10, 0.0, BI / 2 + BW + 0.035), (0.03, 0.03, 0.07), PUMP, False),
), mass=1.0, quat=Q(BOOT_R), friction=0.6, fixed=True)

OBJECTS = [*PANEL, *POST, FILLER, BOOT, OUTLET, *HOSE, NOZZLE]

# -- run-time mechanics -------------------------------------------------------------------------------------------
DT = 1 / 20  # the hook runs once per control step (-E control_hz=20)
MON = M.Monitor(dt=DT)
_RT: dict = {}
FREE_OPEN = 0.080  # a gripper opened wider than this (m) neither squeezes nor holds anything here
TOUCH_R = 0.015  # ... otherwise its tool point within this of the squeeze point: on the lever
HOLD_R = 0.050  # ... or within this of the housing or handle: holding the nozzle


def _targets(environment, latched: bool) -> None:
    set_slide_targets(environment, "nozzle", {"lever": M.LATCH_HOLD if latched else M.LEVER_REST_T,
                                              "clip": M.CLIP_HELD if latched else M.CLIP_T})


def _reset(environment=None) -> None:
    MON.reset()
    _RT.clear()
    _RT.update(steps=0, states={}, hands={})
    if environment is not None:
        _targets(environment, False)


on_reset(_reset)
_reset()


def hand_on(hands, pos, R, point, radius) -> str:
    p = pos + R @ point
    for side, (tcp, _, opening) in hands.items():
        if opening < FREE_OPEN and np.linalg.norm(tcp - p) < radius:
            return side
    return ""


@on_step
def _mechanics(environment) -> None:
    from yam_arena.scene import object_states

    s = object_states(environment, ["nozzle", "filler"])
    hands = robot_hands(environment)
    n = s["nozzle"]
    R = n.rotation()
    lever, clip = float(n.slides[0]), float(n.slides[1])
    depth, off = M.filler_depth(n.pos, R)
    bdepth, boff = M.boot_depth(n.pos, R)
    housing = n.pos + R @ M.HOUSING_GRIP
    touched = bool(hand_on(hands, n.pos, R, M.HANDLE_SQUEEZE, TOUCH_R))
    _RT["steps"] += 1
    t = _RT["steps"] * DT
    for cmd in MON.step(t, lever, clip, depth, off, float(housing[2]), float(n.lin_vel[2]), touched):
        _targets(environment, cmd == "latch")
    held = hand_on(hands, n.pos, R, M.HOUSING_GRIP, HOLD_R) or hand_on(hands, n.pos, R, M.HANDLE_CARRY, HOLD_R)
    _RT.update(states=s, hands=hands, R=R, lever=lever, clip=clip, depth=depth, off=off, bdepth=bdepth, boff=boff,
               flap=float(s["filler"].slides[0]), held=held, touched=touched)
    if os.environ.get("FUEL_DEBUG") and _RT["steps"] % 4 == 0:
        import time

        now = time.time()
        _RT["wall"], last = now, _RT.get("wall", now)
        print(f"[fuel] wall +{now - last:5.1f} s t {t:6.2f} pos {np.round(n.pos, 3)} lever {lever:.2f} clip {clip:.2f} latched {MON.latched} "
              f"depth {depth * 1000:6.1f} off {off * 1000:5.1f} flap {_RT['flap']:.2f} boot {bdepth * 1000:6.1f}/"
              f"{boff * 1000:4.1f} held {held or '-'} touch {touched} hold {MON.held_s:.1f} fail {MON.failed!r} "
              f"stage {stage()} open L {hands['left'][2] * 1000:.0f} R {hands['right'][2] * 1000:.0f} "
              f"tcpL {np.round(R.T @ (hands['left'][0] - n.pos), 3)} tcpR {np.round(R.T @ (hands['right'][0] - n.pos), 3)}",
              flush=True)


# -- scoring ---------------------------------------------------------------------------------------------------------


def holstered() -> bool:
    n = _RT.get("states", {}).get("nozzle")
    if n is None:
        return False
    return bool(_RT["bdepth"] >= M.HOLSTERED and _RT["boff"] < 0.02 and _RT["lever"] < M.REST and not _RT["held"]
                and np.linalg.norm(n.lin_vel) < 0.02)


def stage() -> float:
    return MON.score(holstered())


def score(objects) -> float:
    """0 if at any time the trigger lever was opened (a third of its travel) while the spout was less than 4 cm into
    the filler, or the nozzle fell (falling faster than 0.7 m/s, or its housing within 4 cm of the table). Otherwise:
    0.15 the spout went 8 cm or more into the filler; 0.5 and, with it that far in, the lever stayed held open by the
    hold-open clip for 3 s in a row with no gripper on the lever; 0.7 and afterwards the clip released and the lever
    back at rest; 1.0 and now the nozzle hangs in its boot (spout 13 cm or more in), lever at rest, no gripper holding
    it (success, held 1 s)."""
    return stage()


def success(objects) -> bool:
    return stage() == 1.0


# -- scripted demo ---------------------------------------------------------------------------------------------------


def demo_controller(bases, dt):
    from nozzledemo import Controller

    return Controller(bases, dt, sim=_RT, mon=MON, cheat=os.environ.get("DEMO_CHEAT", ""))


def demo():
    """Placeholder so tools that look for a demo() find one; yam_scripted runs demo_controller instead."""
    return {}
