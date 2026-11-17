"""hookmodel: the wiper arm, its J-hook and the blade adapter's latch, as geometry and a state machine. NumPy only.

Frames (metres, radians):
- G, the glass frame: origin on the windshield's top surface at its lower edge, centre; x up the slope, y along the
  lower edge (world y), z the glass normal (out of the glass).
- The arm frame: origin on the hinge axis at the pivot, axes parallel to G when the arm lies on the glass (angle 0);
  the arm turns about G's x axis, positive lifts it off the glass. The arm runs along +y from the pivot.
- A blade's frame: origin at the middle of the rubber's wiping edge; x along the blade, z up through its frame and
  adapter, y = z cross x. The seat frame is where that frame is when the adapter is latched in the hook.

Each blade's coupling to the hook is one of three states, switched by :class:`HookMonitor` before each physics step:

- ``latched``: adapter clicked into the hook's bend; the blade is welded to the arm (breaks above ``break_n``).
- ``slide``: threaded on the hook but not latched; the blade can only slide along the arm between ``SLIDE_LO`` (adapter
  nose 16 mm back from the bend, clear of the hook) and the seat; the last ``CLICK_M`` before the seat are resisted
  by the latch tab riding over the hook's end (``click_n``) unless the tab is held down.
- ``free``: off the hook.

latched -> slide: the release tab is pressed down (``TAB_PRESSED_M`` of its travel). slide -> latched: the adapter
reaches the seat (within ``SEAT_TOL_M``) with the tab up. slide -> free: the blade is back at the slide's open end
(``OFF_M`` or further) and a gripper holds it to lift it off. free -> slide: the adapter nose is brought under the hook,
behind its bend (between ``SLIDE_LO`` and ``ENTER_HI``), lined up with the arm (``ENTER_TOL_M``, ``ENTER_TOL_DEG``) and
moving toward the bend (never from the tip side through the bend).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

ALPHA = math.radians(27.0)  # windshield slope
U = np.array([math.cos(ALPHA), 0.0, math.sin(ALPHA)])  # G x: up the glass
N = np.array([-math.sin(ALPHA), 0.0, math.cos(ALPHA)])  # G z: glass normal
R_G = np.stack([U, [0.0, 1.0, 0.0], N], axis=1)  # world from G
RZ90 = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # blade axes in the arm frame: x -> arm y

# dimensions (m)
GLASS_T = 0.005
ARM_LEN = 0.36  # pivot to the inside of the hook's bend
PIVOT_Z = 0.049  # hinge axis above the glass
SEAT_Y = ARM_LEN - 0.012  # blade origin along the arm when latched (adapter nose 12 mm ahead of it)
SEAT0 = np.array([0.0, SEAT_Y, -PIVOT_Z])  # blade origin in the arm frame when latched
SLIDE_LO = -0.016  # slide's open end (blade along its own x, from the seat)
OFF_M = -0.013  # the adapter is clear of the hook's return leg from here
ENTER_HI = -0.008
CLICK_M = 0.004
SEAT_TOL_M = 0.0015
ENTER_TOL_M = 0.003
ENTER_TOL_DEG = 6.0
TAB_PRESSED_M = 0.0015
BROKEN_M, BROKEN_DEG = 0.005, 5.0
LOW_DEG = -3.8  # hinge stop: the bare hook resting on the glass
TIP_R = ARM_LEN  # radius used for the hook's speed


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_angle_deg(r: np.ndarray) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, (float(np.trace(r)) - 1.0) / 2.0))))


@dataclass(frozen=True)
class Layout:
    origin: tuple[float, float, float]  # G origin in the world
    pivot: tuple[float, float, float]  # hinge point in G

    def pivot_w(self) -> np.ndarray:
        return np.asarray(self.origin) + R_G @ np.asarray(self.pivot)

    def arm_rot(self, theta: float) -> np.ndarray:
        return R_G @ rot_x(theta)

    def seat(self, theta: float) -> tuple[np.ndarray, np.ndarray]:
        """World pose (origin, rotation) of a latched blade's frame with the arm at ``theta``."""
        ra = self.arm_rot(theta)
        return self.pivot_w() + ra @ SEAT0, ra @ RZ90

    def arm_point(self, theta: float, p) -> np.ndarray:
        return self.pivot_w() + self.arm_rot(theta) @ np.asarray(p, float)

    def rel(self, theta: float, pos, rot) -> tuple[np.ndarray, float]:
        """A blade's offset from the seat in the seat frame (x along the arm) and its angle from the seated pose."""
        so, sr = self.seat(theta)
        return sr.T @ (np.asarray(pos, float) - so), rot_angle_deg(sr.T @ np.asarray(rot, float))


@dataclass(frozen=True)
class LatchParams:
    click_n: float = 7.0  # N: push needed over the last CLICK_M with the tab up
    click_k: float = 2000.0  # N/m: the ramp's stiffness (its force is capped at click_n)
    slide_damping: float = 15.0  # N s/m: adapter sliding in the hook
    break_n: float = 0.0  # N; 0: the weld never breaks (PhysX joint-force spikes broke it during normal tab presses)


class HookMonitor:
    """One blade's coupling to the hook. :meth:`step` takes the blade's offset from the seat (seat frame), its angle
    from the seated pose, whether its tab is pressed and whether a gripper holds it; it returns the commands
    ("lock", on) / ("slide", on) / ("click", on) to apply."""

    def __init__(self, latched: bool, params: LatchParams = LatchParams()):
        self.p = params
        self.start = "latched" if latched else "free"
        self.reset()

    def reset(self) -> None:
        self.state = self.start
        self.prev_x = None
        self.click = False
        self.broken = False
        self.released = False  # was unlatched at some point
        self.engaged_once = False  # was threaded from free at some point

    def step(self, rel, angle_deg: float, tab_pressed: bool, held: bool) -> list[tuple[str, bool]]:
        x, y, z = (float(v) for v in rel)
        out: list[tuple[str, bool]] = []
        moving_in = self.prev_x is None or x - self.prev_x >= -0.0005
        self.prev_x = x
        if self.state == "latched":
            if math.hypot(math.hypot(x, y), z) > BROKEN_M or angle_deg > BROKEN_DEG:
                self.broken = True  # the weld gave: forced off
                self.state = "free"
                out += [("lock", False)]
            elif tab_pressed:
                self.state, self.released = "slide", True
                out += [("lock", False), ("slide", True)]
        elif self.state == "slide":
            if x >= -SEAT_TOL_M and not tab_pressed:
                self.state = "latched"
                out += [("slide", False), ("lock", True)]
            elif x <= OFF_M and held:
                self.state = "free"
                out += [("slide", False)]
        elif (SLIDE_LO <= x <= ENTER_HI and abs(y) <= ENTER_TOL_M and abs(z) <= ENTER_TOL_M
              and angle_deg <= ENTER_TOL_DEG and moving_in):
            self.state, self.engaged_once = "slide", True
            out += [("slide", True)]
        click = self.state == "slide" and not tab_pressed and x > -CLICK_M
        if click != self.click:
            self.click = click
            out += [("click", click)]
        return out


@dataclass(frozen=True)
class ArmRules:
    slam_mps: float = 0.75  # hook speed toward the glass at landing: a fall of about 3 cm on the spring alone (released
    #                         from 14 degrees, 8.7 cm, the hook lands at 1.32 m/s in the simulation; v ~ sqrt(height))
    near_deg: float = 2.0  # landing zone: within this of the glass (blade on) / the stop (bare)
    bare_deg: float = LOW_DEG + 0.5  # bare arm this low: the hook is on the glass


class ArmMonitor:
    def __init__(self, rules: ArmRules = ArmRules()):
        self.r = rules
        self.reset()

    def reset(self) -> None:
        self.slammed = False
        self.bare_hit = False
        self.peak_mps = 0.0
        self.max_deg = 0.0

    def step(self, theta: float, theta_dot: float, blade_on: bool) -> None:
        deg = math.degrees(theta)
        self.max_deg = max(self.max_deg, deg)
        land = 0.0 if blade_on else LOW_DEG
        speed = -theta_dot * TIP_R
        if deg < land + self.r.near_deg and speed > 0:
            self.peak_mps = max(self.peak_mps, speed)
            self.slammed |= speed > self.r.slam_mps
        if not blade_on and deg <= self.r.bare_deg:
            self.bare_hit = True

    @property
    def failed(self) -> str:
        return "arm slammed onto the glass" if self.slammed else "bare hook on the glass" if self.bare_hit else ""
