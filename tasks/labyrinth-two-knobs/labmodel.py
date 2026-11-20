"""labmodel: the labyrinth's geometry, its knob gearing and the path scoring. Pure Python (NumPy), no Isaac Sim.

Frames (metres):
- box frame: origin on the underside of the box at its centre; x along the long side (350 mm), y along the short
  side (305 mm), z up.
- board frame: origin at the centre of the board's top surface; u along the box's x, v along its y, w up. When the
  board is level, board = box + (0, 0, BOARD_TOP).

Gimbal: the frame turns about the box's x axis (``tilt_x``, driven by the knob on the +x end face), and the board
turns
about the frame's y axis (``tilt_y``, driven by the knob on the -y long face). Each knob's turn sets its tilt through
the string-on-shaft gearing: tilt = KNOB_SIGN * knob angle / RATIO, limited to +-TILT_MAX.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# -- the game: BRIO Labyrinth (34000) ----------------------------------------------------------------------------
BOX = (0.350, 0.305, 0.094)  # outside, x y z
WALL_T = 0.015  # box side walls
FLOOR_T = 0.006
BOARD = (0.290, 0.245, 0.008)  # the playing board
BOARD_TOP = 0.080  # board's top surface above the box's underside, level
RIM_T, RIM_H = 0.006, 0.012  # the gimbal frame's raised edge, just outside the board's edges
WALL_H = 0.010  # the board's black plastic partitions
BALL_R, BALL_MASS = 0.00635, 0.0084  # 12.7 mm steel ball
HOLE_R = 0.0072  # 14.4 mm holes (measured on the photo, below)
KNOB_R, KNOB_LEN, KNOB_GAP = 0.0175, 0.022, 0.008  # 35 mm knob, 22 mm deep, 8 mm off the wall on its shaft
KNOB_Z = 0.055  # knob axis above the box's underside
GRIP_OUT = KNOB_GAP + 0.014  # where the fingers grip the knob: this far out from the wall (8 mm in from its end)

# Gearing: each shaft winds a string that pulls the board's edge against a spring. A 8 mm shaft (4 mm radius)
# pulling at about 0.10 m from the tilt axis gives 0.04 rad of tilt per radian of knob: 1 degree per 25 degrees.
RATIO = 25.0
TILT_MAX = math.radians(4.0)  # the board's travel each way
KNOB_LIMIT = math.radians(120.0)  # the knob turns on (string wound further only slips)
KNOB_FRICTION_NM = 0.04  # the knob's turning resistance (string tension on the shaft, felt as friction)

# -- the board: BRIO Labyrinth Boards (34030), the 8-hole practice board -------------------------------------------
# Traced from BRIO's own product photo of 34030 (the 8-hole board, as sold; the same board comes in 34020), rectified
# to the board's 290 x 245 mm outline (README). Board frame, mm, u along the long side, v toward the far long edge
# (away from the side knob): the BRIO logo is at the -u, -v corner. Positions +-3 mm.
_WALLS_MM = (  # (u0, u1, v0, v1): footprints of the 6 mm partitions
    (-105, 26, 100.5, 106.5), (-105, -99, 91, 106), (20.5, 26.5, 78, 122.5), (20.5, 42, 74, 80),
    (-121, -20, 61, 67), (-124, -118, 47.5, 72), (-25.5, -19.5, -37, 61), (-20, 37.5, 40, 46),
    (-145, -50, -4, 2), (-108, -102, 2, 17.5), (-71, -65, 2, 17),
    (-96, -90, -71, -34), (-115, -90, -75, -69), (-91, 22.5, -46, -40), (-37, -31, -75, -40),
    (-67, -61, -122.5, -77), (-61, 56, -106, -100), (0, 6, -100, -84.5),
    (49.5, 55.5, -100, 20), (49.5, 96, 14, 20), (28, 50, -7.5, -1.5), (55, 118, -46, -40),
    (93, 99, -71, 78), (80, 112.5, 74, 80), (80, 86, 80, 103.5), (64, 86, 97.5, 103.5), (99, 112.5, 42, 48),
    (128, 145, 57, 63), (121, 145, -12.5, -6.5), (-145, -126, -35, -29), (79.5, 85.5, -122.5, -92))
_HOLES_MM = ((-110.6, 77.1), (-84.6, 15.1), (-110.9, -55.8), (38.8, -86.9), (14.4, -6.7), (39.0, 57.7),
             (109.3, 28.3), (109.4, -63.5))  # numbered 1..8 as printed
# The printed path, START (the arrow, top middle) to FINISH (the star, bottom middle), kept >= 6.35 mm + 0.8 mm
# (the ball's radius and a margin) from every partition and the board's edge.
_PATH_MM = ((-12.5, 114.5), (-60, 114.5), (-100, 114.5), (-120, 113), (-131, 103), (-134, 70), (-133, 45), (-125, 33),
            (-110, 30), (-95, 35), (-70, 42.5), (-50, 36), (-38, 20), (-37, 0), (-42, -12.5), (-62.5, -20),
            (-100, -25), (-118, -42), (-130, -67.5), (-127.5, -87.5), (-115, -92.5), (-79, -92.5), (-77.5, -65),
            (-70, -62.5), (-52.5, -63.75), (-48.75, -85), (-17.5, -85), (-15, -62.5), (30, -62.5), (35, -30),
            (30, -27.5), (2.5, -25), (-5, -10), (-2.5, 7.5), (10, 17.5), (35, 25), (62.5, 42.5), (67.5, 62.5),
            (65, 77.5), (52.5, 92.5), (53, 106), (62.5, 113), (90, 113), (102.5, 100), (120, 88), (121, 68.5),
            (121, 50), (130, 40), (127.5, 17.5), (112.5, 5), (110, -9), (116, -22), (130, -32.5), (132.5, -52.5),
            (130, -72.5), (117.5, -87.5), (97.5, -90), (80, -77.5), (72.5, -80), (67.5, -95), (66, -108),
            (60, -114), (-36, -114.5))
WALLS = np.array(_WALLS_MM, float) / 1000
START, FINISH = np.array(_PATH_MM[0]) / 1000, np.array(_PATH_MM[-1]) / 1000


def path_points() -> np.ndarray:
    """The printed path's centre line, START to FINISH."""
    return np.array(_PATH_MM, float) / 1000


def holes() -> np.ndarray:
    """(N, 2) hole centres, numbered 1..N as printed (row k is hole k + 1)."""
    return np.array(_HOLES_MM, float) / 1000


@dataclass(frozen=True)
class Path:
    pts: np.ndarray

    @property
    def seg_len(self) -> np.ndarray:
        return np.linalg.norm(np.diff(self.pts, axis=0), axis=1)

    @property
    def length(self) -> float:
        return float(self.seg_len.sum())

    def project(self, p, lo: float = -math.inf, hi: float = math.inf) -> tuple[float, float]:
        """(arc length of the closest point on the path with arc length in [lo, hi], distance to it)."""
        p = np.asarray(p, float)[:2]
        best = (0.0, math.inf)
        s0 = 0.0
        for a, b, ln in zip(self.pts[:-1], self.pts[1:], self.seg_len):
            if s0 > hi or s0 + ln < lo:
                s0 += ln
                continue
            t = float(np.clip((p - a) @ (b - a) / ln**2, (lo - s0) / ln, (hi - s0) / ln))
            t = float(np.clip(t, 0.0, 1.0))
            d = float(np.linalg.norm(a + t * (b - a) - p))
            if d < best[1]:
                best = (s0 + t * ln, d)
            s0 += ln
        return best

    def point(self, s: float) -> np.ndarray:
        s = float(np.clip(s, 0.0, self.length))
        for a, b, ln in zip(self.pts[:-1], self.pts[1:], self.seg_len):
            if s <= ln:
                return a + (b - a) * s / ln
            s -= ln
        return self.pts[-1].copy()


PATH = Path(path_points())
HOLES = holes()
HOLE_S = np.array([PATH.project(h)[0] for h in HOLES])  # where each hole is along the path
N_HOLES = len(HOLES)
N_STEPS = N_HOLES + 1  # the score counts the holes passed and FINISH
FINISH_TOL = 0.010  # FINISH: the ball's centre within 1 cm (along the path) of the path's end at the star


class Progress:
    """How far along the path the ball has come. The ball's place on the path is its closest point on the centre
    line within ``max_jump`` of its last place (so it never jumps to a neighbouring stretch across a partition).
    ``passed`` is the number of holes whose line across the path the ball's centre has crossed, plus 1 at FINISH
    (latched: the furthest point counts)."""

    def __init__(self, max_jump: float = 0.04):
        self.max_jump = max_jump
        self.reset()

    def reset(self) -> None:
        self.s = 0.0
        self.s_max = 0.0
        self.fell: int | None = None  # hole number (1..N) the ball fell into

    def step(self, uv) -> None:
        if self.fell is not None:
            return
        uv = np.asarray(uv, float)[:2]
        d = np.linalg.norm(HOLES - uv, axis=1)
        k = int(np.argmin(d))
        if d[k] < HOLE_R:
            self.fell = k + 1
            return
        self.s, _ = PATH.project(uv, self.s - self.max_jump, self.s + self.max_jump)
        self.s_max = max(self.s_max, self.s)

    @property
    def passed(self) -> int:
        return int(np.sum(HOLE_S < self.s_max)) + int(self.s_max >= PATH.length - FINISH_TOL)


def tilt_from_knob(theta: float) -> float:
    """Board tilt (rad) for a knob angle (rad)."""
    return float(np.clip(theta / RATIO, -TILT_MAX, TILT_MAX))


def ball_accel(tilt_x: float, tilt_y: float) -> np.ndarray:
    """Rolling ball's acceleration (u, v) on the board for the two tilts (5/7 g sin, solid sphere rolling).
    tilt_x turns about +x (raises +v), tilt_y about +y (lowers +u)."""
    g = 9.81 * 5 / 7
    return np.array([g * math.sin(tilt_y), -g * math.sin(tilt_x)])


def score_value(passed: int, foul: bool) -> float:
    """(Holes passed + 1 at FINISH) / (N + 1); 0 after a foul."""
    return 0.0 if foul else passed / N_STEPS
