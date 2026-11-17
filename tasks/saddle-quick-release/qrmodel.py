"""qrmodel: geometry of the balance bike's seat post and quick-release collar, the clamp's rules, and the score.
Plain Python + NumPy (no Isaac Sim), so the unit tests and the scripted demo can use it.

World frame (metres): x forward from the arms, y left, z up. The bike stands along x, front wheel away from the
robot, so its left side (where the quick-release lever is) faces +y and its seat tube leans back toward the arms.

Joints (all in one fixed articulation, the bike frame as root):
- ``post_slide``: prismatic along the seat tube axis ``A`` (m, + = post up). 0 = the start height, where the lower
  edge of the blue tape band on the post is level with the collar's top.
- ``post_turn``: revolute about ``A`` (rad, + = nose turned to the bike's left). 0 = saddle in line with the frame.
- ``lever``: revolute about +y through the cam pivot (rad). 0 = closed, folded down along the seat tube; opening
  swings the paddle back and up, toward the arms.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

TOP = 0.7475  # station table top
TILT = math.radians(15.0)  # seat tube leans back 15 degrees from vertical (75 degree seat angle)
A = np.array([-math.sin(TILT), 0.0, math.cos(TILT)])  # seat tube axis, up
REAR = np.array([-math.cos(TILT), 0.0, -math.sin(TILT)])  # perpendicular to A in the bike's plane, toward the rear
Y = np.array([0.0, 1.0, 0.0])
COLLAR_TOP = np.array([0.25, 0.0, TOP + 0.24])  # centre of the collar's top face (post axis)

POST_D = 0.0222  # 22.2 mm (7/8 in) post
POST_LEN = 0.22
E0 = 0.045  # post showing above the collar at the start (along A), up to the saddle's clamp shell
PT = E0 * A  # post top, relative to COLLAR_TOP, at slide 0 and turn 0
NOSE = PT + np.array([0.075, 0.0, 0.0225])  # middle of the saddle nose (grip point), same frame
NOSE_W = 0.05
SADDLE_TOP = PT[2] + 0.035  # saddle top above COLLAR_TOP's height at slide 0

PIVOT = COLLAR_TOP - 0.009 * A + 0.020 * REAR + np.array([0.0, 0.027, 0.0])  # cam pivot (bolt axis along y)
LEVER_LEN = 0.048  # pivot to the middle of the paddle

TAPE_TARGET_MM = 40.0  # instruction: tape's lower edge 4 cm above the collar
MIN_INSERT_M = 0.075  # the post's MIN INSERT line clears the collar beyond this rise
TWIST0_DEG = 12.0  # the saddle starts turned 12 degrees to the bike's left


def rot_axis(axis, angle) -> np.ndarray:
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def post_point(p_rel, slide: float, turn: float) -> np.ndarray:
    """World position of a point of the post/saddle given relative to COLLAR_TOP at slide 0, turn 0."""
    return COLLAR_TOP + slide * A + rot_axis(A, turn) @ np.asarray(p_rel, float)


def lever_dir(theta: float) -> np.ndarray:
    return math.cos(theta) * -A + math.sin(theta) * REAR


def lever_tangent(theta: float) -> np.ndarray:
    """Direction the paddle moves as the lever opens."""
    return -math.sin(theta) * -A + math.cos(theta) * REAR


def paddle(theta: float) -> np.ndarray:
    return PIVOT + LEVER_LEN * lever_dir(theta)


# -- the clamp ----------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class ClampParams:
    contact_deg: float = 90.0  # the cam starts to bear on the collar ears
    peak_deg: float = 22.0  # hardest point of the stroke
    dead_deg: float = 12.0  # dead (centre) point: past it the cam pushes the lever shut
    grip_deg: float = 30.0  # below this the collar grips the post
    peak_n: float = 30.0  # force at the paddle at the hardest point (N)
    hold_n: float = 5.0  # force holding the lever shut past the dead point (N)
    open_deg: float = 120.0  # open stop
    lever_fric_nm: float = 0.01  # pivot friction (keeps an open lever where it is left)
    slide_fric_n: float = 1.5  # loose post: sliding friction in the tube (less than its 2.9 N weight along A)
    turn_fric_nm: float = 0.03  # loose post: turning friction
    lock_k: float = 15000.0  # clamped post: N/m (20 N push: 1.3 mm)
    lock_d: float = 60.0
    lock_kt: float = 60.0  # clamped post: N m/rad against turning
    lock_dt: float = 0.3
    v_slide: float = 0.01  # friction regularisation speeds (m/s, rad/s)
    v_turn: float = 0.05
    v_lever: float = 0.2


def paddle_force(theta_deg: float, p: ClampParams = ClampParams()) -> float:
    """Force (N) the cam pushes the paddle OPEN with; negative past the dead point (pulls it shut)."""
    if theta_deg >= p.contact_deg:
        return 0.0
    if theta_deg >= p.peak_deg:
        return p.peak_n * (p.contact_deg - theta_deg) / (p.contact_deg - p.peak_deg)
    if theta_deg >= p.dead_deg:
        return p.peak_n * (theta_deg - p.dead_deg) / (p.peak_deg - p.dead_deg)
    return -p.hold_n * min(1.0, (p.dead_deg - theta_deg) / 4.0)


class Clamp:
    """``step(theta, theta_dot, slide, slide_vel, turn, turn_vel)`` -> (lever N m, slide N, turn N m) joint efforts."""

    def __init__(self, params: ClampParams = ClampParams()):
        self.p = params
        self.reset()

    def reset(self) -> None:
        self.gripping = True
        self.hold = (0.0, math.radians(TWIST0_DEG))

    def step(self, theta, theta_dot, slide, slide_vel, turn, turn_vel):
        p = self.p
        deg = math.degrees(theta)
        lever = paddle_force(deg, p) * LEVER_LEN - p.lever_fric_nm * float(np.clip(theta_dot / p.v_lever, -1, 1))
        grip = deg < p.grip_deg
        if grip and not self.gripping:
            self.hold = (slide, turn)
        self.gripping = grip
        if grip:
            f = -p.lock_k * (slide - self.hold[0]) - p.lock_d * slide_vel
            t = -p.lock_kt * (turn - self.hold[1]) - p.lock_dt * turn_vel
        else:
            f = -p.slide_fric_n * float(np.clip(slide_vel / p.v_slide, -1, 1))
            t = -p.turn_fric_nm * float(np.clip(turn_vel / p.v_turn, -1, 1))
        return lever, f, t


# -- scoring --------------------------------------------------------------------------------------------------------
HEIGHT_TOL_MM = 5.0
ALIGN_DEG = 5.0
RAISED_MM = 10.0
CLOSED_DEG = 12.0  # lever past its dead point (folded within 12 degrees of the frame)


def score_state(rise_mm: float, twist_deg: float, lever_deg: float, hands_clear: bool, failed: bool) -> float:
    """0 if the post was ever pulled past its MIN INSERT line (failed), unless the lever is closed past its dead
    point, both hands are clear and the post is at least 1 cm up. Then 0.3, +0.5 if the tape is 40 +- 5 mm above
    the collar, +0.2 if the saddle is within 5 degrees of the frame's line."""
    if failed or not hands_clear or lever_deg >= CLOSED_DEG or rise_mm < RAISED_MM:
        return 0.0
    s = 0.3
    if abs(rise_mm - TAPE_TARGET_MM) <= HEIGHT_TOL_MM:
        s += 0.5
    if abs(twist_deg) <= ALIGN_DEG:
        s += 0.2
    return round(s, 3)
