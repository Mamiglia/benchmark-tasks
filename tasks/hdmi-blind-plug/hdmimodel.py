"""hdmimodel: geometry, the contact-spring friction rule and the score bookkeeping of the HDMI task. NumPy only.

Frames (metres). World: x forward from the arms, y left, z up. The monitor faces the robot; its HDMI port is in the
recessed connector bay on the bottom of its back and faces straight down, so the plug goes in upward (+z).

Plug frame P: origin at the centre of the overmold's front face (the base of the metal shell), x along the plug toward
the shell's tip, z toward the shell's wide side (the trapezoid's long edge), y = z x x. Lying on the desk at the start,
x points forward and z up. Plugged in, x points up and z toward the screen (world -x).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

TOP = 0.7475  # station table top

# -- monitor (Dell P2422H, 23.8 in, on its stand; positions in world m) ----------------------------------------------
FRONT_X = 0.400  # screen's front face
SLAB_X1 = 0.420  # back of the thin outer panel
BACK_X = 0.452  # back of the rear housing (52 mm deep in all)
PANEL_W, PANEL_H = 0.538, 0.3185
PB = TOP + 0.130  # bottom edge of the panel and the rear housing: stand set so it is 13 cm above the desk
BAY = (0.422, 0.446)  # the connector bay: an open channel in the housing's bottom (x range), ...
RECESS = 0.006  # ... this deep; the port faces are its ceiling
ZM = PB + RECESS  # HDMI port mouth (z)
PX, PY = 0.432, -0.027  # HDMI port axis: 32 mm behind the screen's face, 27 mm right of the centre line
HOUSING_W = 0.36  # rear housing width (y)

# -- HDMI type A (plug shell outside, receptacle inside) -------------------------------------------------------------
SHELL_W, SHELL_T = 0.0140, 0.00455  # plug shell: width, thickness
CHAMFER_W = 0.0013  # the narrow side's corners are cut this far in (each) ...
NARROW_T = 0.00145  # ... over this much of the thickness: the key
SHELL_LEN = 0.0115  # shell length past the overmold
CLR = 0.0004  # receptacle clearance per side (sim; the real one is about 0.1 mm)
CAV_DEPTH = 0.013  # receptacle depth above the mouth (the overmold reaches the mouth first)
OVER_L, OVER_W, OVER_T = 0.036, 0.021, 0.011  # overmold: length, width (y), thickness (z)
GRIP_X = -0.031  # where the hand holds the overmold (plug frame x): near its back end, so the fingers stay low
PLUG_MASS = 0.030  # plug and the first few cm of cable

# -- the receptacle's contact springs: a sliding joint along the port axis whose damping drive saturates at F_INS, so
# the shell slides only while it is pushed (or pulled) with more than F_INS (HDMI spec: insertion 44.1 N max,
# withdrawal 9.8 N min to 39.2 N max) -----------------------------------------------------------------------------
F_INS = 15.0  # N (design value; measure the real pair with a luggage scale, see README)
BITE = 0.004  # the springs grip the shell from this depth on
SEAT_DAMPING = 1.0e5  # N s/m: the drive saturates above 0.15 mm/s
CORD_PULL = 0.5  # N: the lifted cable's pull toward the generator (0.2 N hanging cable, plus drag on the desk)
CORD_LIFT = TOP + 0.015  # the pull acts once the plug's centre is this high (lying on the desk, the cable is slack)
CORD_ANCHOR = np.array([0.15, -0.50, TOP + 0.01])  # the signal generator's HDMI output, where the cable leads

# -- scoring thresholds ----------------------------------------------------------------------------------------------
ENTERED = 0.003  # shell this deep in the port: in
SEATED = SHELL_LEN - 0.001  # overmold within 1 mm of the port face: fully home
ON_AXIS = 0.002  # plug axis this close to the port axis (anything deeper than ENTERED is inside the receptacle)
HOLD_R, HOLD_OPEN = 0.030, 0.035  # a tool point this close to the grip zone, fingers closer than this: holding it
CLEAR_R = 0.070  # every tool point this far from the grip zone: let go

# -- start: plug lying flat on the desk in front of the right arm, tip forward, wide side up -------------------------
START_XY = (0.30, -0.13)


def rot_z(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_y(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


R_PLUGGED = rot_y(-90.0)  # plug x -> world +z, plug z -> world -x


def start_pose(seed: int) -> tuple[np.ndarray, np.ndarray]:
    """Plug origin and rotation at the start for a seed: up to 2 cm off START_XY each way, yawed -20 to 20 degrees
    (whole mm, whole degrees)."""
    rng = np.random.default_rng(seed)
    dx, dy = (round(v * 1000) / 1000 for v in rng.uniform(-0.02, 0.02, 2))
    yaw = float(round(rng.uniform(-20.0, 20.0)))
    return np.array([START_XY[0] + dx, START_XY[1] + dy, TOP + OVER_T / 2 + 0.0005]), rot_z(yaw)


def plugged_pose(depth: float) -> tuple[np.ndarray, np.ndarray]:
    """Plug origin and rotation on the port axis, right way round, shell tip ``depth`` past the mouth."""
    return np.array([PX, PY, ZM + depth - SHELL_LEN]), R_PLUGGED.copy()


def grips(depth: float, off: float, tilt: float) -> bool:
    """The contact springs take hold of the shell: in past BITE, on the axis, nearly straight."""
    return depth > BITE and off < 0.001 and tilt < 3.0


def lets_go(depth: float) -> bool:
    """... and release it once it is pulled back out past BITE."""
    return depth < BITE - 0.0005


def depth_of(pos, rot) -> tuple[float, float, float]:
    """(shell tip's height past the mouth, distance of the shell's tip centre off the port axis, angle in degrees of
    the plug axis off vertical) for a plug pose."""
    pos, rot = np.asarray(pos, float), np.asarray(rot, float)
    tip = pos + SHELL_LEN * rot[:, 0]
    tilt = math.degrees(math.acos(min(1.0, max(-1.0, float(rot[2, 0])))))
    return float(tip[2] - ZM), float(math.hypot(tip[0] - PX, tip[1] - PY)), tilt


def cord_force(pos) -> np.ndarray:
    """The cable's pull on the plug (N, world): CORD_PULL toward where it leads, once the plug is off the desk."""
    if pos[2] < CORD_LIFT:
        return np.zeros(3)
    d = CORD_ANCHOR - np.asarray(pos, float)
    return CORD_PULL * d / np.linalg.norm(d)


# -- score bookkeeping -----------------------------------------------------------------------------------------------


@dataclass
class Monitor:
    """``step`` once per control step with the plug's depth and offset; ``score`` with the end state.

    1.0: the plug is seated (overmold within 1 mm of the port face) and nothing holds it. 0.5: at the end the shell is
    in the port 3 mm or more, but not seated and released. 0.25: at some point the shell went 3 mm into the port
    (latched), but it is out at the end. Otherwise 0."""

    entered: bool = False
    max_depth: float = -1.0

    def reset(self) -> None:
        self.entered = False
        self.max_depth = -1.0

    def step(self, depth: float, off: float) -> None:
        if depth >= ENTERED and off < ON_AXIS:
            self.entered = True
            self.max_depth = max(self.max_depth, depth)

    def score(self, depth: float, off: float, clear: bool) -> float:
        inside = depth >= ENTERED and off < ON_AXIS
        if inside and depth >= SEATED and clear:
            return 1.0
        if inside:
            return 0.5
        if self.entered:
            return 0.25
        return 0.0
