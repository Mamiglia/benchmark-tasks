"""plugmodel: geometry, the split-pin resistance rule and the score bookkeeping of the 7-way coupling task.
NumPy only (no Isaac Sim).

Frames (metres). World: x forward from the arms, y left, z up. The socket's axis runs along world +x (into the
trailer wall). Plug frame P: origin at the centre of the plug's front face (the insert face), x along the plug toward
that face (into the socket when coupled), z toward the key rib. Lid frame: origin on the hinge pin, axis world +y;
lid angle 0 = shut over the socket mouth, positive = swung out and up.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

TOP = 0.7475  # station table top

# -- socket on the trailer's front wall (Phillips 15-720, SAE J560, split pins) --------------------------------------
XF = 0.500  # barrel mouth (world x)
YS = 0.020  # socket axis (world y)
ZC = TOP + 0.130  # socket axis (world z): 13 cm above the table
BARREL_IN = 0.022  # bore, 12-sided stand-in for the round barrel (apothem)
BARREL_OUT = 0.026
BARREL_LEN = 0.030  # mouth to the pin face
HINGE = np.array([XF - 0.005, YS, ZC + 0.040])  # lid hinge pin
LID_LEN = 0.075  # hinge to the lid's lower edge
LID_K, LID_T = 0.15, -0.80  # torsion spring: N m/rad toward -0.8 rad (0.12 N m shut, 0.36 N m at 90 degrees)
LID_MAX = 2.10  # rad, the lid's stop (120 degrees)

# -- plug (Phillips 15-710 zinc die-cast J560 plug on a 1.2 m length of 7-way cable) ---------------------------------
PLUG_MASS = 0.35
NOSE_R, NOSE_LEN = 0.021, 0.031  # insert (12-sided, apothem) and its length
BODY_R, BODY_END = 0.027, -0.160  # body (12-sided, apothem), from the nose back to x = -0.160
RIB = ((-NOSE_LEN, 0.0), 0.006, (0.0205, 0.026))  # key rib on the insert: x range, width, z range
RIDGE_X = (-0.100, -0.035)  # catch ridge on the body's top: the lid's catch rides on it until it drops off its end
RIDGE_W, RIDGE_TOP = 0.016, 0.033
DEPTH_FULL = BARREL_LEN  # insert face on the pin face
SEATED = 0.027  # "fully home": within 3 mm of the pin face (the depth mark level with the barrel rim)
ENGAGED = 0.015  # the pins start to bite once the insert is this far in
F_PINS = 20.0  # N, split-pin resistance at full engagement (design value; measure the real pair, see README)
CORD_PULL = 1.5  # N, the hanging cable's pull on the plug
CORD_HOOK = np.array([0.20, -0.45, TOP + 0.40])  # where the cable hangs from

# -- start: plug lying in a V-block, its key rolled away from 12 o'clock ---------------------------------------------
VBLOCK_XY = (0.30, -0.13)
V_FLOOR = TOP + 0.015  # bottom of the 90 degree groove
V_DEPTH = 0.022


def roll_for(seed: int) -> float:
    """The plug's start roll for a seed: degrees about its axis (right-handed about world +x, so negative turns the
    key from 12 o'clock toward the robot's left), 15 to 40 degrees, whole degrees."""
    return -float(round(np.random.default_rng(seed).uniform(15.0, 40.0)))


def rot_x(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(rad: float) -> np.ndarray:
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def start_pose(seed: int) -> tuple[np.ndarray, np.ndarray]:
    """Plug origin and rotation at the start: axis along +x, resting in the V-block, rolled by ``roll_for(seed)``."""
    centre_z = V_FLOOR + BODY_R * math.sqrt(2.0) + 0.002
    return np.array([VBLOCK_XY[0] + 0.07, VBLOCK_XY[1], centre_z]), rot_x(roll_for(seed))


def coupled_pose(depth: float) -> tuple[np.ndarray, np.ndarray]:
    """Plug origin and rotation on the socket axis, key at 12 o'clock, insert face ``depth`` past the mouth."""
    return np.array([XF + depth, YS, ZC]), np.eye(3)


def depth_of(pos, rot) -> tuple[float, float, float]:
    """(insert depth past the mouth along the axis, distance of the plug axis off the socket axis, key roll in degrees
    from 12 o'clock) for a plug pose."""
    pos, rot = np.asarray(pos, float), np.asarray(rot, float)
    off = math.hypot(pos[1] - YS, pos[2] - ZC)
    z = rot[:, 2]
    return float(pos[0] - XF), float(off), float(math.degrees(math.atan2(-z[1], z[2])))


def aligned(pos, rot, off_tol: float = 0.004, tilt_tol: float = 0.10) -> bool:
    """Plug on the socket axis: centre within off_tol and its axis within tilt_tol (rad) of the socket's."""
    _, off, _ = depth_of(pos, rot)
    return off < off_tol and math.acos(min(1.0, float(np.asarray(rot)[0, 0]))) < tilt_tol


def pin_resistance(depth: float) -> float:
    """Most axial force the split pins can take (N): 0 until the insert is ENGAGED deep, then rising linearly to
    F_PINS at full depth."""
    return F_PINS * float(np.clip((depth - ENGAGED) / (DEPTH_FULL - ENGAGED), 0.0, 1.0))


def axial_friction(depth: float, v_axial: float, other_axial: float, mass: float, dt: float) -> float:
    """Coulomb friction along the axis from the pins (N, + = into the socket): the force that would stop the plug's
    axial motion within this physics step, net of the other known axial force, capped at ``pin_resistance``."""
    cap = pin_resistance(depth)
    if cap <= 0.0:
        return 0.0
    want = -mass * v_axial / dt - other_axial
    return float(np.clip(want, -cap, cap))


def cord_force(pos) -> np.ndarray:
    """The hanging cable's pull on the plug (N, world)."""
    d = CORD_HOOK - np.asarray(pos, float)
    return CORD_PULL * d / np.linalg.norm(d)


def lid_catch_angle() -> tuple[float, float]:
    """(lid angle with its catch on the ridge, with it down in the notch behind the ridge), radians, from the lid's
    geometry: the catch's inner lower corner on the ridge top / on the body top."""
    a, b = 0.008, -0.075  # catch inner face, lower edge (lid frame x, z)

    def angle_for(z_top):
        lo, hi = 1.0, 2.0  # the corner's height rises with the angle over this range
        for _ in range(60):
            mid = (lo + hi) / 2
            if HINGE[2] - ZC - a * math.sin(mid) + b * math.cos(mid) < z_top:
                lo = mid
            else:
                hi = mid
        return (lo + hi) / 2

    return angle_for(RIDGE_TOP), angle_for(BODY_R)


LID_RIDE, LID_CAUGHT = lid_catch_angle()
LID_CAUGHT_MAX = (LID_RIDE + LID_CAUGHT) / 2  # lid below this with the plug seated: caught in the notch
LID_CLEAR = math.radians(100.0)  # lid held at least this far open: clear of the plug's path


# -- score bookkeeping -----------------------------------------------------------------------------------------------


@dataclass
class Monitor:
    """Tracks the trial. ``step`` once per control step with the plug and lid state; ``score`` at the end.

    Stages (each at some point, latched): 0.1 the lid was held clear (100 degrees or more) while the plug's insert face
    was within 1 cm of the mouth on the axis; 0.3 the insert went ENGAGED deep (key in its groove). End states: 0.6 the
    plug sits seated (SEATED deep) and released; 1.0 seated, released, and the lid down in the notch behind the ridge.
    Any time: the plug dropped (it fell back onto the table, or below 4 cm above it, after it left the V-block) -> 0."""

    dt: float = 0.05
    lifted: bool = False
    dropped: bool = False
    presented: bool = False
    engaged: bool = False
    max_depth: float = -1.0

    def reset(self) -> None:
        self.lifted = self.dropped = self.presented = self.engaged = False
        self.max_depth = -1.0

    def step(self, depth: float, off: float, low_z: float, lid: float) -> None:
        if low_z > V_FLOOR + V_DEPTH + 0.03:
            self.lifted = True
        if self.lifted and depth < 0.0 and low_z < TOP + 0.04:
            self.dropped = True
        if lid >= LID_CLEAR and -0.010 <= depth and off < 0.006:
            self.presented = True
        if depth >= ENGAGED and off < 0.004:
            self.engaged = True
        self.max_depth = max(self.max_depth, depth)

    def score(self, depth: float, lid: float, plug_held: bool, lid_held: bool) -> float:
        if self.dropped:
            return 0.0
        seated = depth >= SEATED and not plug_held
        if seated and not lid_held and lid < LID_CAUGHT_MAX:
            return 1.0
        if seated:
            return 0.6
        if self.engaged:
            return 0.3
        if self.presented:
            return 0.1
        return 0.0
