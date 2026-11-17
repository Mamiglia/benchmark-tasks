"""bindermodel: geometry of the open lever arch file and the punched stack, the lever mechanism's rules and the score.
Plain Python + NumPy (no Isaac Sim), so the unit tests and the scripted demo can use it.

World frame (metres): x forward from the arms, y left, z up. The file lies open on the table, spine along y on the far
side, back board (with the mechanism) toward the arms, front cover laid flat beyond the spine.

Mechanism (one fixed articulation, the file as root):
- two fixed posts (the rings' fixed halves) stand on the back board, 80 mm apart along y;
- ``arch``: one bar carrying both rings' moving halves, revolute about +y through the mechanism's pivot; 0 = shut
  (each half's tip on its post's top), + = open (the tips swing up and away toward the spine);
- ``lever``: the finger lever at the file's top end, revolute about +y through the same pivot line; 0 = folded flat,
  + = thrown up toward the spine. A cam gears the arch to the lever (arch = RATIO x lever) and an over-centre spring
  holds the lever either shut or open.

The stack (one free articulation): ``sheets`` (root) is the punched strip along the holed edge; ``mid`` and ``free``
are the rest of the sheets, hinged to it and to each other along y with a bending spring.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

TOP = 0.7475  # station table top
BOARD_T = 0.002  # board thickness (polypropylene-covered board)
BZ = TOP + BOARD_T  # top face of the back board

# -- the file (Leitz 180 lever arch file, A4, 80 mm spine: 285 x 318 mm boards) -------------------------------------
BOARD_X = (0.205, 0.490)  # back board, along x; it is 318 mm along y
BOARD_HALF_Y = 0.159
SPINE_X = (0.490, 0.570)
COVER_X = (0.570, 0.855)
POST_X = 0.440
POST_Y = (0.040, -0.040)  # ISO 838: two holes 80 mm apart
POST_D = 0.0046  # ring wire
POST_H = 0.065  # post top above the board
POST_TOP = BZ + POST_H
TIP_LEN = 0.006  # the last 6 mm of the post narrows (rounded tip)
TIP_D = 0.0030
PIVOT = np.array([0.475, 0.0, BZ + 0.012])  # the arch bar's pivot line (along y)
BASE_X = (0.452, 0.488)  # mechanism base plate
BASE_H = 0.006
# moving half of one ring, in the arch frame (x, z relative to PIVOT): up from the pivot, over, down onto the post
ARCH_PTS = ((0.0, 0.0), (0.0, 0.042), (-0.012, 0.0555), (POST_X - PIVOT[0], 0.0555))
LEVER_Y = 0.160
LEVER_PIVOT = np.array([PIVOT[0], LEVER_Y, PIVOT[2]])
LEVER_ARM = ((0.0, 0.0), (-0.035, 0.004))  # lever frame (x, z)
LEVER_PLATE = ((-0.035, 0.004), (-0.058, 0.012))  # finger plate, tilted up so a finger can get round it
PLATE_W = 0.016
LEVER_GRIP = np.array([-0.050, 0.0, 0.0095])  # plate centre, lever frame

RATIO = 0.60  # arch angle / lever angle (the cam)
LEVER_OPEN = math.radians(80.0)  # open stop: the lever stands up, leaning a little toward the pages
ARCH_OPEN = math.radians(50.0)

# -- the stack (20 sheets A4 80 g/m2, ISO 838 two-hole punched) ------------------------------------------------------
SHEET_L = 0.297  # along y
SHEET_W = 0.210  # along x
STACK_T = 0.002  # 20 x 0.1 mm
STACK_MASS = 0.100  # 20 x 5.0 g
HOLE = 0.0060  # square hole, side (ISO 838: 6 +- 0.5 mm round)
HOLE_EDGE = 0.012  # hole centre from the holed edge
STRIP_W = (0.040, 0.085, 0.085)  # widths (x) of the punched strip, the middle and the free part
HINGE_K = (0.12, 0.04)  # N m/rad bending springs at the two hinges
CREASE = math.radians(120.0)  # bent this far at a hinge: creased


def rot_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def arch_point(p_xz, arch: float, y: float = 0.0) -> np.ndarray:
    """World point of the arch bar given (x, z) relative to the pivot at arch angle 0."""
    return PIVOT + rot_y(arch) @ np.array([p_xz[0], y, p_xz[1]])


def lever_point(p, lever: float) -> np.ndarray:
    """World point of the lever given (x, y, z) or (x, z) relative to its pivot at lever angle 0."""
    p = np.array([p[0], 0.0, p[1]]) if len(p) == 2 else np.asarray(p, float)
    return LEVER_PIVOT + rot_y(lever) @ p


def lever_grip(lever: float) -> np.ndarray:
    return lever_point(LEVER_GRIP, lever)


def arch_tip(i: int, arch: float) -> np.ndarray:
    return arch_point(ARCH_PTS[-1], arch, POST_Y[i])


# -- the mechanism -----------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class LeverParams:
    centre_deg: float = 40.0  # over-centre point
    close_peak_n: float = 10.0  # hardest push to throw it open, at the plate (N)
    close_peak_deg: float = 20.0
    hold_n: float = 3.0  # pulls the shut lever down at 0 degrees (N)
    open_peak_n: float = 6.0  # pushes it open past the centre
    open_peak_deg: float = 60.0
    open_hold_n: float = 2.0  # at the open stop
    arm_m: float = 0.050  # pivot to the plate's centre
    friction_nm: float = 0.004
    v_lever: float = 0.2
    arch_k: float = 4.0  # N m/rad: the cam's stiffness between lever and arch (implicit drive on the arch)
    arch_d: float = 0.02


def plate_force(theta_deg: float, p: LeverParams = LeverParams()) -> float:
    """Force (N) the spring pushes the plate with: negative toward shut, positive toward open."""
    c = p.centre_deg
    if theta_deg <= 0.0:
        return -p.hold_n
    if theta_deg < p.close_peak_deg:
        return -(p.hold_n + (p.close_peak_n - p.hold_n) * theta_deg / p.close_peak_deg)
    if theta_deg < c:
        return -p.close_peak_n * (c - theta_deg) / (c - p.close_peak_deg)
    if theta_deg < p.open_peak_deg:
        return p.open_peak_n * (theta_deg - c) / (p.open_peak_deg - c)
    top = math.degrees(LEVER_OPEN)
    s = min(1.0, (theta_deg - p.open_peak_deg) / (top - p.open_peak_deg))
    return p.open_peak_n + s * (p.open_hold_n - p.open_peak_n)


def lever_effort(theta: float, theta_dot: float, arch: float, p: LeverParams = LeverParams()) -> float:
    """Lever joint effort (N m): over-centre spring, pivot friction, and the cam's reaction from the arch."""
    spring = plate_force(math.degrees(theta), p) * p.arm_m
    fric = -p.friction_nm * float(np.clip(theta_dot / p.v_lever, -1.0, 1.0))
    reaction = -RATIO * p.arch_k * (RATIO * theta - arch)
    return spring + fric + reaction


def arch_target(theta: float) -> float:
    return float(np.clip(RATIO * theta, 0.0, ARCH_OPEN))


# -- the stack ---------------------------------------------------------------------------------------------------
# Stack root frame: origin at the punched strip's centre, x toward the holed edge (+x in the world at rest), y along
# the holed edge, z up. The holed edge is at x = +STRIP_W[0] / 2.
EDGE_X = STRIP_W[0] / 2
HOLE_X = EDGE_X - HOLE_EDGE
HOLES = ((HOLE_X, POST_Y[0], 0.0), (HOLE_X, POST_Y[1], 0.0))
HINGE1_X = -STRIP_W[0] / 2
HINGE2_X = HINGE1_X - STRIP_W[1]


def strip_boxes(color, hole_color):
    """The punched strip as boxes (stack frame): two 6 mm square holes, centred HOLE_EDGE from the holed edge."""
    w, L, t, h = STRIP_W[0], SHEET_L, STACK_T, HOLE / 2
    x0, x1 = -w / 2, w / 2
    ys = sorted(POST_Y)
    cuts = [-L / 2, ys[0] - h, ys[0] + h, ys[1] - h, ys[1] + h, L / 2]
    out = []
    for a, b in ((cuts[0], cuts[1]), (cuts[2], cuts[3]), (cuts[4], cuts[5])):
        out.append((((x0 + x1) / 2, (a + b) / 2, 0.0), (w, b - a, t), color))
    for y in ys:
        for a, b in ((x0, HOLE_X - h), (HOLE_X + h, x1)):
            out.append((((a + b) / 2, y, 0.0), (b - a, HOLE, t), color))
        out.append(((HOLE_X, y, t / 2 + 0.0001), (HOLE + 0.0016, HOLE + 0.0016, 0.0001), hole_color, False))  # rim mark
    return out


def strip_points(pos, R) -> np.ndarray:
    """Points over the punched strip (world), for the hands-clear check."""
    pts = [(x, y, 0.0) for x in (-STRIP_W[0] / 2, 0.0, STRIP_W[0] / 2) for y in np.linspace(-SHEET_L / 2, SHEET_L / 2, 9)]
    return np.asarray(pos) + np.array(pts) @ np.asarray(R).T


def hole_world(pos, R, i: int) -> np.ndarray:
    return np.asarray(pos) + np.asarray(R) @ np.array(HOLES[i])


def threaded(pos, R, i: int, tol: float = 0.003) -> bool:
    """Post i passes through hole i: hole centre within ``tol`` of the post's axis, and 5 mm or more below its top."""
    h = hole_world(pos, R, i)
    return bool(math.hypot(h[0] - POST_X, h[1] - POST_Y[i]) < tol and h[2] < POST_TOP - 0.005 and h[2] > BZ - 0.003)


# -- scoring -------------------------------------------------------------------------------------------------------
LEVER_SHUT_DEG = 5.0
ARCH_SHUT_DEG = 2.0
OPENED_DEG = 40.0  # lever past its centre: the rings were opened
CLEAR_M = 0.03
FORCED_DEG = 6.0  # the arch more open than the lever's cam allows, by this much, with a hand at the arch


def score_state(failed: bool, n_threaded: int, shut: bool, clear: bool, ever_opened: bool) -> float:
    """0 if failed (stack creased, off the table, or the rings forced open by hand). 1.0 with both posts through both
    holes, the rings shut and locked and both hands clear; 0.5 with both posts through; 0.2 with one; 0.1 if the
    lever was ever thrown past its centre; else 0."""
    if failed:
        return 0.0
    if n_threaded == 2:
        return 1.0 if (shut and clear) else 0.5
    if n_threaded == 1:
        return 0.2
    return 0.1 if ever_opened else 0.0
