"""On the station table: a 24-inch office monitor (black, Dell P2422H) on its stand, facing the robot, its screen's
front 40 cm ahead of the arms and its lower edge 13 cm above the desk. Its connector bay is on the bottom of its back,
and the ports in it face straight down; the HDMI port is 27 mm right of the monitor's centre line. In front of the
right arm an HDMI cable's plug lies flat on the desk, pointing at the monitor, its cable leading off to the right to a
small black HDMI signal generator.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
The plug's start position and yaw come from the seed PLUG_SEED (environment variable, default 0).
"""

from __future__ import annotations

import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import hdmimodel as M  # noqa: E402
import numpy as np  # noqa: E402
from partkit import Link, frame_x, matrix_to_quat, on_physics, on_reset, on_step, part, robot_hands, set_link  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
assert abs(TOP - M.TOP) < 1e-9
INSTRUCTION = "Plug the HDMI cable into the HDMI port on the back of the monitor."

SEED = int(os.environ.get("PLUG_SEED", "0"))
BLACK, DARK, SCREEN, METAL, PORT = (0.07, 0.07, 0.08), (0.13, 0.13, 0.14), (0.03, 0.04, 0.06), (0.72, 0.73, 0.75), \
    (0.02, 0.02, 0.02)


def _b(lo, hi, color, origin, collide=True):
    """A box spec from world corners, relative to a part's origin."""
    lo, hi, o = np.asarray(lo, float), np.asarray(hi, float), np.asarray(origin, float)
    return (tuple((lo + hi) / 2 - o), tuple(hi - lo), color, collide)


# -- monitor: thin outer panel, rear housing with the connector bay, the HDMI receptacle, stand ----------------------
O = (M.PX, M.PY, M.ZM)
HW = M.HOUSING_W / 2
CX0, CX1 = M.PX - M.SHELL_T / 2 - M.CLR, M.PX + M.SHELL_T / 2 + M.CLR  # receptacle opening (x)
CY0, CY1 = M.PY - M.SHELL_W / 2 - M.CLR, M.PY + M.SHELL_W / 2 + M.CLR  # (y)
Z0, Z1 = M.ZM, M.ZM + M.CAV_DEPTH  # receptacle layer
KX0 = M.PX + M.SHELL_T / 2 - M.NARROW_T + M.CLR  # key: the corners on the receptacle's back side are filled
KW = M.CHAMFER_W
PW, PH = M.PANEL_W / 2, M.PANEL_H
MONITOR_BOXES = (
    _b((M.FRONT_X, -PW, M.PB), (M.SLAB_X1, PW, M.PB + PH), BLACK, O),  # outer panel
    _b((M.FRONT_X - 0.001, -PW + 0.008, M.PB + 0.022), (M.FRONT_X, PW - 0.008, M.PB + PH - 0.008), SCREEN, O, False),
    _b((M.SLAB_X1, -HW, Z1), (M.BACK_X, HW, M.PB + 0.25), BLACK, O),  # rear housing above the receptacle layer
    # receptacle layer, leaving the HDMI opening (14.8 x 5.35 mm) with its two keyed corners
    _b((M.SLAB_X1, -HW, Z0), (CX0, HW, Z1), DARK, O),
    _b((CX1, -HW, Z0), (M.BACK_X, HW, Z1), DARK, O),
    _b((CX0, CY1, Z0), (CX1, HW, Z1), DARK, O),
    _b((CX0, -HW, Z0), (CX1, CY0, Z1), DARK, O),
    _b((KX0, CY0, Z0), (CX1, CY0 + KW, Z1), DARK, O),
    _b((KX0, CY1 - KW, Z0), (CX1, CY1, Z1), DARK, O),
    # the bay's front and back lips (the bay is an open channel along y)
    _b((M.SLAB_X1, -HW, M.PB), (M.BAY[0], HW, Z0), BLACK, O),
    _b((M.BAY[1], -HW, M.PB), (M.BACK_X, HW, Z0), BLACK, O),
    # the other ports in the bay ceiling (drawn: power, DisplayPort, VGA, USB)
    *(_b((M.PX - h / 2, y - w / 2, Z0 - 0.0006), (M.PX + h / 2, y + w / 2, Z0 - 0.0001), PORT, O, False)
      for y, w, h in ((M.PY + 0.045, 0.018, 0.012), (M.PY - 0.032, 0.0165, 0.005), (M.PY - 0.068, 0.031, 0.011),
                      (M.PY - 0.100, 0.0125, 0.005), (M.PY + 0.150, 0.0125, 0.005), (M.PY + 0.170, 0.0125, 0.005))),
    _b((CX0, CY0, Z1 - 0.0005), (CX1, CY1, Z1), PORT, O, False),  # dark inside of the receptacle
    # stand: neck behind the housing, base plate
    _b((M.BACK_X, -0.04, TOP + 0.012), (M.BACK_X + 0.03, 0.04, M.PB + 0.22), DARK, O),
    _b((0.395, -0.1225, TOP), (0.568, 0.1225, TOP + 0.012), DARK, O),
)
MONITOR = part("monitor", O, MONITOR_BOXES, mass=5.6, friction=0.4, fixed=True)

# -- plug: keyed shell (wide side toward plug +z), overmold, cable stub (drawn) ---------------------------------------
_wide_t = M.SHELL_T - M.NARROW_T
PLUG_BOXES = (
    ((M.SHELL_LEN / 2, 0.0, M.SHELL_T / 2 - _wide_t / 2), (M.SHELL_LEN, M.SHELL_W, _wide_t), METAL),
    ((M.SHELL_LEN / 2, 0.0, -M.SHELL_T / 2 + M.NARROW_T / 2),
     (M.SHELL_LEN, M.SHELL_W - 2 * M.CHAMFER_W, M.NARROW_T), METAL),
    ((-M.OVER_L / 2, 0.0, 0.0), (M.OVER_L, M.OVER_W, M.OVER_T), BLACK),
    ((-M.OVER_L - 0.03, 0.0, 0.0), (0.06, 0.0065, 0.0065), BLACK, False),  # cable (drawn)
)
P0, R0 = M.start_pose(SEED)
# The contact springs: a joint sliding along the port axis (its position is the shell's depth), the plug's lateral
# motion and turning locked, a damping drive capped at F_INS. Off until the shell is in past BITE on the axis.
_F0 = frame_x((0.0, 0.0, 1.0))
SEAT = Link(name="seat", to="monitor", pos0=(0.0, 0.0, -M.SHELL_LEN), rot0=tuple(map(tuple, _F0)),
            rot1=tuple(map(tuple, M.R_PLUGGED.T @ _F0)), slide=True, lower=0.0, upper=M.SHELL_LEN - 0.0003,
            stiffness=0.0, damping=M.SEAT_DAMPING, max_force=M.F_INS)
PLUG = part("plug", P0, PLUG_BOXES, mass=M.PLUG_MASS, quat=matrix_to_quat(R0), friction=0.8, links=(SEAT,),
            com=(-0.015, 0.0, 0.0), inertia=(1.2e-6, 4.0e-6, 4.5e-6))

GEN = Box("signal_generator", pos=(M.CORD_ANCHOR[0], M.CORD_ANCHOR[1], TOP + 0.011), size=(0.09, 0.06, 0.022),
          color=DARK)
OBJECTS = [MONITOR, GEN, PLUG]

# -- run-time mechanics: the contact springs' grip and the cable's pull on the plug (each physics step) ---------------
MON = M.Monitor()
_RT: dict = {}


def _reset(environment=None) -> None:
    MON.reset()
    _RT.clear()
    _RT.update(steps=0, states={}, hands={}, seat=False)
    if environment is not None:
        set_link("seat", False)


on_reset(_reset)
_reset()


@on_physics
def _springs(environment) -> None:
    import torch
    from partkit import _np, quat_to_matrix

    base = environment.unwrapped
    plug = base.scene["plug"]
    pose = _np(plug.data.root_link_pose_w)[0]
    pos = pose[:3] - _np(base.scene.env_origins)[0]
    depth, off, tilt = M.depth_of(pos, quat_to_matrix(pose[3:7]))
    if not _RT["seat"] and M.grips(depth, off, tilt):
        set_link("seat", True)
        _RT["seat"] = True
    elif _RT["seat"] and M.lets_go(depth):
        set_link("seat", False)
        _RT["seat"] = False
    f = M.cord_force(pos)
    forces = torch.tensor([[f.tolist()]], dtype=torch.float32, device=plug.device)
    plug.permanent_wrench_composer.set_forces_and_torques_index(forces=forces, torques=torch.zeros_like(forces),
                                                                 is_global=True)


@on_step
def _track(environment) -> None:
    from yam_arena.scene import object_states

    s = object_states(environment, ["plug"])
    hands = robot_hands(environment)
    p = s["plug"]
    R = p.rotation()
    depth, off, tilt = M.depth_of(p.pos, R)
    grip = p.pos + R @ np.array([M.GRIP_X, 0.0, 0.0])
    dist = {k: float(np.linalg.norm(tcp - grip)) for k, (tcp, _, _) in hands.items()}
    held = any(dist[k] < M.HOLD_R and hands[k][2] < M.HOLD_OPEN for k in hands)
    clear = all(d >= M.CLEAR_R for d in dist.values())
    _RT["steps"] += 1
    MON.step(depth, off)
    _RT.update(states=s, hands=hands, R=R, depth=depth, off=off, tilt=tilt, held=held, clear=clear)
    if os.environ.get("PLUG_DEBUG") and _RT["steps"] % 4 == 0:
        print(f"[hdmi] t {_RT['steps'] / 20:6.2f} pos {np.round(p.pos, 4)} depth {depth * 1000:6.1f} off "
              f"{off * 1000:5.2f} tilt {tilt:5.1f} hand z {hands['right'][0][2]:.4f} held {held} clear {clear} "
              f"seat {_RT['seat']} stage {stage():.2f}", flush=True)


# -- scoring ---------------------------------------------------------------------------------------------------------


def stage() -> float:
    if not _RT.get("states"):
        return 0.0
    return MON.score(_RT["depth"], _RT["off"], bool(_RT["clear"]))


def score(objects) -> float:
    """1.0 the plug is seated in the HDMI port (overmold within 1 mm of the port face) and no gripper is within 7 cm
    of it; 0.5 at the end the shell is 3 mm or more into the port, but not seated and let go; 0.25 at some point the
    shell went 3 mm into the port, but it is out at the end; else 0."""
    return stage()


def success(objects) -> bool:
    return stage() == 1.0


# -- scripted demo ---------------------------------------------------------------------------------------------------


def demo_controller(bases, dt):
    from hdmidemo import Controller

    return Controller(bases, dt, sim=_RT, cheat=os.environ.get("DEMO_CHEAT", ""))


def demo():
    """Placeholder so tools that look for a demo() find one; yam_scripted runs demo_controller instead."""
    return {}
