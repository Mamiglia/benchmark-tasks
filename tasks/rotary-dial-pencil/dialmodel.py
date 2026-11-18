"""dialmodel: the rotary phone's geometry, its dial's spring and governor, and the pulse counting and score.

Pure Python (numpy only): the unit tests run it without Isaac Sim.

Dial frame D: origin at the dial's centre on the face of the phone's sloped front; z = the face's outward normal
(tilted ``TILT`` from vertical toward the robot); x = toward 3 o'clock as the robot sees the dial (world -y);
y = toward 12 o'clock (up the slope). Angles ``psi`` are counter-clockwise from 3 o'clock as seen from the front.

The finger wheel turns clockwise when pulled. ``theta`` (degrees, >= 0) is how far it is turned from rest.
Hole n (n = 1..10 pulses; digit n % 10) brings a pencil in it to the finger stop after ``hole_theta(n)`` degrees.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# -- the phone (GPO 746 rotary; phone frame: origin under the body's centre, x toward the back, z up) -----------------
BODY = (0.160, 0.215, 0.145)  # depth (x), width (y), height (z)
TILT = math.radians(35.0)  # dial face from horizontal
FRONT_X, FRONT_Z = -0.080, 0.040  # front edge of the sloped face
SLOPE_LEN = 0.098  # along the slope, front edge to the top block
DIAL_S = 0.049  # dial centre along the slope from the front edge
DIAL_C = (FRONT_X + DIAL_S * math.cos(TILT), 0.0, FRONT_Z + DIAL_S * math.sin(TILT))  # phone frame
PHONE_MASS = 1.0

# -- the dial ----------------------------------------------------------------------------------------------------------
HOLE_R = 0.037  # hole circle radius
HOLE_D = 0.017  # finger hole diameter (drawn); the pockets are the gaps between radial webs
WHEEL_R = 0.048
HUB_R = 0.0285
PLATE_TOP = 0.001  # number plate surface above the face
WHEEL_Z0, WHEEL_Z1 = 0.0015, 0.0075  # finger wheel underside, top (above the face)
WEB_T = 0.003
STOP_PSI = -50.0  # finger stop, degrees (about 4.30)
STOP_Z0, STOP_Z1 = 0.0085, 0.019
STOP_T = 0.003  # tangential thickness of the stop's end
THETA1 = 55.0  # wheel turn at which a pencil in hole 1, pushing the hole's clockwise edge, meets the stop
HOLE_LAG = 15.0  # deg: hole centre behind that contact (pencil and stop half-widths, 7 mm pencil)
PITCH = 30.0
THETA_MAX = 335.0
WHEEL_MASS = 0.03

# spring and governor (real dials: about 1-2 N at the finger, a run-back of about 10 pulses a second)
SPRING_T0 = 0.025  # N m at rest (preload)
SPRING_K = 0.008  # N m per rad
GOV_SPEED = math.radians(300.0)  # rad/s, run-back speed held by the governor

# -- scoring -----------------------------------------------------------------------------------------------------------
NUMBER = (4, 1, 1)  # directory assistance
PULL_START = 10.0  # deg: a pull starts
REST = 2.0  # deg: back at rest
PULSE_TOL = 15.0  # a pulse counts when the wheel got within half a pitch of bringing that hole to the stop
RELEASE_DROP = 10.0  # deg below the pull's maximum: the run-back has begun
SLOW_FACTOR, SLOW_SLACK = 1.5, 0.25  # run-back slower than 1.5 x free + 0.25 s: held back
REPUSH = 3.0  # deg: turned forward again during the run-back
MOVE_MM, TIP_DEG, LIFT_MM = 30.0, 10.0, 10.0  # phone fouls
HAND_R, HAND_H = 0.052, 0.015  # gripper on the dial: inside this cylinder over the face


def hole_theta(n: int) -> float:
    """Wheel turn (deg) that brings hole n (1..10) to the finger stop."""
    return THETA1 + (n - 1) * PITCH


def hole_psi(n: int, theta: float = 0.0) -> float:
    """Angle (deg) of hole n's centre with the wheel turned ``theta``."""
    return STOP_PSI + HOLE_LAG + hole_theta(n) - theta


def pulses_for(theta_max: float) -> int:
    """Pulses a pull to ``theta_max`` sends: the holes it brought within PULSE_TOL of the stop."""
    return sum(theta_max >= hole_theta(n) - PULSE_TOL for n in range(1, 11))


def hole_for_digit(d: int) -> int:
    return 10 if d == 0 else d


def spring_torque(theta_rad: float) -> float:
    return SPRING_T0 + SPRING_K * max(theta_rad, 0.0)


def dial_axes(tilt: float = TILT, yaw: float = 0.0) -> np.ndarray:
    """Columns x, y, z of the dial frame in the world, for a phone turned ``yaw`` (its front faces -x at yaw 0)."""
    c, s = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    x = np.array([0.0, -1.0, 0.0])
    y = np.array([math.cos(tilt), 0.0, math.sin(tilt)])
    z = np.array([-math.sin(tilt), 0.0, math.cos(tilt)])
    return Rz @ np.c_[x, y, z]


def dial_point(psi_deg: float, r: float = HOLE_R, h: float = 0.0) -> np.ndarray:
    """A point in the dial frame."""
    p = math.radians(psi_deg)
    return np.array([r * math.cos(p), r * math.sin(p), h])


@dataclass
class Pull:
    t0: float
    theta_max: float = 0.0
    t_release: float | None = None
    t_end: float | None = None
    repushed: bool = False
    low: float = 1e9  # lowest angle since the run-back began

    @property
    def pulses(self) -> int:
        return pulses_for(self.theta_max)

    @property
    def free_time(self) -> float:
        return max(self.theta_max - RELEASE_DROP - REST, 0.0) / math.degrees(GOV_SPEED)

    @property
    def held(self) -> bool:
        """Held back, or pushed on again, during its run-back."""
        if self.repushed:
            return True
        if self.t_release is None or self.t_end is None:
            return False
        return self.t_end - self.t_release > SLOW_FACTOR * self.free_time + SLOW_SLACK

    @property
    def digit(self) -> int | None:
        """The digit it dials (None for no pulse). A held pull dials a wrong digit (-1)."""
        if self.pulses == 0:
            return None
        return -1 if self.held else self.pulses % 10


@dataclass
class Dialer:
    """Follows the wheel angle step by step and records each completed pull."""

    pulls: list = field(default_factory=list)
    cur: Pull | None = None

    def step(self, t: float, theta: float) -> None:
        c = self.cur
        if c is None:
            if theta > PULL_START:
                self.cur = c = Pull(t0=t, theta_max=theta)
            return
        if c.t_release is None:
            if theta > c.theta_max:
                c.theta_max = theta
            elif theta < c.theta_max - RELEASE_DROP:
                c.t_release, c.low = t, theta
        else:
            c.low = min(c.low, theta)
            if theta > c.low + REPUSH:
                c.repushed = True
                c.theta_max = max(c.theta_max, theta)
        if theta < REST:
            c.t_end = t
            self.pulls.append(c)
            self.cur = None

    @property
    def digits(self) -> list[int]:
        return [p.digit for p in self.pulls if p.digit is not None]


def score_value(digits, fouled: bool) -> float:
    """Leading digits matching NUMBER, / 3. A fourth digit makes the number wrong (at most 2/3). 0 after a foul."""
    if fouled:
        return 0.0
    k = 0
    for want, got in zip(NUMBER, digits):
        if want != got:
            break
        k += 1
    if k == len(NUMBER) and len(digits) > len(NUMBER):
        k = len(NUMBER) - 1
    return k / len(NUMBER)
