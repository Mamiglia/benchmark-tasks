"""nozzlemodel: geometry and the latch/score bookkeeping of the refuelling task. NumPy only (no Isaac Sim).

Frames (metres). World: x forward from the arms, y left, z up. Nozzle frame N: origin where the spout leaves the
valve housing, x along the handle toward the spout end, y to the nozzle's left, z up when the handle is level.
Filler frame F: origin on the filler's lip centre, x down the bore (into the car), y = world y.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

TOP = 0.7475  # station table top


def rot_y(deg: float) -> np.ndarray:
    """Rotation about world +y; positive tips +x downward (nose down)."""
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


# -- nozzle (Husky 1A-class automatic unleaded nozzle, dry) ---------------------------------------------------------
SPOUT_DEG = 30.0  # spout bent down from the handle line
SPOUT_DIR = np.array([math.cos(math.radians(SPOUT_DEG)), 0.0, -math.sin(math.radians(SPOUT_DEG))])  # N frame
SPOUT_LEN = 0.20
SPOUT_W = 0.019  # square stand-in for the 21.3 mm round spout
NOZZLE_MASS = 1.30
LEVER_PIVOT = np.array([-0.100, 0.0, -0.020])  # N frame; lever rotates about N +y, positive = squeezed (up)
LEVER_GRIP = np.array([-0.165, 0.0, -0.045])  # where a finger lifts the lever (N frame, rest)
CLIP_PIVOT = np.array([-0.150, 0.0, -0.081])  # on the guard bar; positive = arm (reaching back) up under the lever
TAB_PUSH = np.array([-0.145, 0.0, -0.105])  # front face of the clip's tab, mid-height (N frame, rest); pushed back
SWIVEL = np.array([-0.262, 0.0, 0.020])  # hose end of the swivel (N frame)
HOUSING_GRIP = np.array([-0.050, 0.0, 0.005])  # support-hand tool point (housing middle)
HANDLE_CARRY = np.array([-0.165, 0.0, 0.020])  # tool point around the handle tube alone (lever not touched)
HANDLE_SQUEEZE = np.array([-0.165, 0.0, -0.009])  # tool point spanning handle top and lever bottom

LEVER_REST_K, LEVER_REST_T = 2.0, -0.25  # N m/rad toward -0.25 rad: 8 N at rest, 20 N at full travel (at LEVER_GRIP)
LEVER_MAX = 0.36  # rad, lever against the handle
VALVE_OPEN = 0.12  # rad: the main poppet lifts (about a third of the travel)
LATCH_MIN = 0.20  # lever squeezed this far with the clip up: the clip catches it
LATCH_HOLD = 0.22  # where the clip holds the lever (the middle notch)
UNLATCH = 0.29  # squeezed past the notch: the clip drops away
CLIP_K, CLIP_T, CLIP_MAX, CLIP_SET = 0.05, -0.30, 0.80, 0.50  # clip spring, travel and "set" angle (rad)
CLIP_HELD = 0.65
CLIP_DOWN = 0.20  # after a release the clip must fall back under this before it can catch again
REST = 0.05  # lever "released": below this

# -- car body: rear quarter panel with an Easy Fuel capless filler ---------------------------------------------------
PANEL_X = 0.56  # front face of the panel (world x)
POCKET_D = 0.035  # fuel-door pocket depth
FILLER_TILT = 30.0  # bore axis below horizontal
FILLER_Y, FILLER_Z = 0.15, TOP + 0.17  # filler centre: 17 cm above the table, 92 cm above the floor
LIP = np.array([PANEL_X + POCKET_D, FILLER_Y, FILLER_Z])
R_F = rot_y(FILLER_TILT)  # filler frame in world
BORE_IN = 0.024  # inner square of the bore (restrictor 23.6 mm on the real head)
BORE_LEN = 0.17
FLAP_AT = 0.025  # sprung flap this far down the bore
INSERT_FULL = 0.080  # spout tip this far past the lip: fully in (the 80 mm tape mark inside the filler)
INSERT_OUT = 0.040  # under this the spout is out of the filler (for the "lever open outside the filler" rule)

# -- pump: boot (holster) on a post, hose outlet on a mast -----------------------------------------------------------
BOOT_PITCH = 25.0  # nozzle nose-down in the boot: spout 55 degrees below horizontal
R_BOOT = rot_y(BOOT_PITCH)
BOOT_N = np.array([0.420, -0.05, TOP + 0.185])  # nozzle origin when holstered
BOOT_AXIS = R_BOOT @ SPOUT_DIR
BOOT_MOUTH = BOOT_N + 0.030 * BOOT_AXIS
BOOT_IN, BOOT_LEN = 0.036, 0.175
HOLSTERED = 0.130  # spout tip this deep in the boot
HOSE_ANCHOR = np.array([0.56, -0.40, TOP + 0.52])
HOSE_N, HOSE_SEG = 8, 0.13  # 1.04 m of 3/4 in hose
HOSE_W = 0.026
HOSE_SEG_MASS = 0.08  # 0.62 kg/m


def nozzle_pose_in_filler(depth: float) -> tuple[np.ndarray, np.ndarray]:
    """Nozzle origin and rotation with the handle level and the spout tip ``depth`` past the lip, on the bore axis."""
    d = R_F[:, 0]
    return LIP - (SPOUT_LEN - depth) * d, np.eye(3)


def spout_tip(pos, rot) -> np.ndarray:
    return np.asarray(pos, float) + np.asarray(rot, float) @ (SPOUT_LEN * SPOUT_DIR)


def filler_depth(pos, rot) -> tuple[float, float]:
    """(how far the spout tip is past the lip along the bore, its distance off the bore axis)."""
    v = R_F.T @ (spout_tip(pos, rot) - LIP)
    return float(v[0]), float(math.hypot(v[1], v[2]))


def boot_depth(pos, rot) -> tuple[float, float]:
    v = spout_tip(pos, rot) - BOOT_MOUTH
    along = float(v @ BOOT_AXIS)
    return along, float(np.linalg.norm(v - along * BOOT_AXIS))


def hose_path(a, b, n: int = HOSE_N, seg: float = HOSE_SEG, floor: float = TOP + HOSE_W / 2, iters: int = 4000):
    """n+1 points from a to b, consecutive points ``seg`` apart, hanging under gravity above ``floor``
    (position-based relaxation)."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    if np.linalg.norm(b - a) > n * seg:
        raise ValueError("hose too short")
    p = np.linspace(a, b, n + 1)
    p[1:-1, 2] -= 0.2 * np.sin(np.linspace(0, math.pi, n + 1)[1:-1])
    for _ in range(iters):
        p[1:-1, 2] -= 0.0005
        p[:, 2] = np.maximum(p[:, 2], floor)
        for _ in range(2):
            for i in range(n):
                d = p[i + 1] - p[i]
                L = np.linalg.norm(d)
                corr = (L - seg) / L * d
                if i == 0:
                    p[i + 1] -= corr
                elif i == n - 1:
                    p[i] += corr
                else:
                    p[i] += corr / 2
                    p[i + 1] -= corr / 2
        p[0], p[-1] = a, b
    return p


def frame_along(x) -> np.ndarray:
    """A rotation whose x axis is ``x`` and whose y axis is horizontal."""
    x = np.asarray(x, float) / np.linalg.norm(x)
    y = np.cross([0.0, 0.0, 1.0], x)
    y = y / np.linalg.norm(y) if np.linalg.norm(y) > 1e-6 else np.array([0.0, 1.0, 0.0])
    return np.stack([x, y, np.cross(x, y)], axis=1)


# -- latch and score bookkeeping -------------------------------------------------------------------------------------
HOLD_S = 3.0  # latched open, hands off the lever, spout fully in, this long in a row
DROP_VZ = -0.7  # m/s: the nozzle is falling
DROP_Z = TOP + 0.04  # housing centre this low: it hit the table


@dataclass
class Monitor:
    """Fed once per physics step. ``step`` returns the latch commands ("latch"/"unlatch") for the sim to apply."""

    dt: float = 1 / 240
    latched: bool = False
    seated: bool = False
    armed: bool = True
    failed: str = ""
    inserted: bool = False  # spout fully in at some point
    held_s: float = 0.0
    filled: bool = False  # latched hold done
    released: bool = False  # clip dropped and lever back at rest, after the hold
    peak_lever: float = 0.0
    log: list = field(default_factory=list)

    def reset(self) -> None:
        self.__init__(dt=self.dt)

    def step(self, t: float, lever: float, clip: float, depth: float, off_axis: float, housing_z: float, vz: float,
             lever_touched: bool) -> list[str]:
        cmds = []
        in_bore = off_axis < BORE_IN
        fully_in = in_bore and depth >= INSERT_FULL
        if not self.armed and clip < CLIP_DOWN:
            self.armed = True  # the clip has dropped back: it can be set again
        if not self.latched and self.armed and lever >= LATCH_MIN and clip >= CLIP_SET:
            self.latched, self.seated = True, False
            cmds.append("latch")
        elif self.latched and not self.seated and lever < UNLATCH - 0.03:
            self.seated = True  # the lever has come down onto the clip's notch
        elif self.latched and self.seated and lever >= UNLATCH:
            self.latched, self.armed = False, False
            cmds.append("unlatch")
        if not self.failed:
            if lever >= VALVE_OPEN and not (in_bore and depth >= INSERT_OUT):
                self.failed = "lever open with the spout out of the filler"
            elif housing_z < DROP_Z or vz < DROP_VZ:
                self.failed = "nozzle dropped"
        if self.failed:
            return cmds
        self.inserted |= fully_in
        if not self.filled:
            self.held_s = self.held_s + self.dt if (self.latched and fully_in and not lever_touched) else 0.0
            if self.held_s >= HOLD_S:
                self.filled = True
                self.log.append((t, "filled"))
        elif not self.released and not self.latched and lever < REST:
            self.released = True
            self.log.append((t, "released"))
        return cmds

    def score(self, holstered_now: bool) -> float:
        if self.failed or not self.inserted:
            return 0.0
        if not self.filled:
            return 0.15
        if not self.released:
            return 0.5
        return 1.0 if holstered_now else 0.7
