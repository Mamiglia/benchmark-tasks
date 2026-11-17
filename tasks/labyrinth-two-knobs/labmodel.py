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

# -- the product: BRIO Labyrinth (34000) ----------------------------------------------------------------------------
BOX = (0.350, 0.305, 0.094)  # outside, x y z
WALL_T = 0.015  # box side walls
FLOOR_T = 0.006
BOARD = (0.290, 0.245, 0.008)  # the playing board
BOARD_TOP = 0.080  # board's top surface above the box's underside, level
RIM_T, RIM_H = 0.006, 0.012  # the raised edge round the board
LANE_WALL_T, LANE_WALL_H = 0.005, 0.010  # the path's wooden walls
BALL_R, BALL_MASS = 0.00635, 0.0084  # 12.7 mm steel ball
HOLE_R = 0.009  # 18 mm holes
KNOB_R, KNOB_LEN, KNOB_GAP = 0.0175, 0.022, 0.008  # 35 mm knob, 22 mm deep, 8 mm off the wall on its shaft
KNOB_Z = 0.055  # knob axis above the box's underside
GRIP_OUT = KNOB_GAP + 0.014  # where the fingers grip the knob: this far out from the wall (8 mm in from its end)

# Gearing: each shaft winds a string that pulls the board's edge against a spring. A 8 mm shaft (4 mm radius)
# pulling at about 0.10 m from the tilt axis gives 0.04 rad of tilt per radian of knob: 1 degree per 25 degrees.
RATIO = 25.0
TILT_MAX = math.radians(4.0)  # the board's travel each way
KNOB_LIMIT = math.radians(120.0)  # the knob turns on (string wound further only slips)
KNOB_FRICTION_NM = 0.04  # the knob's turning resistance (string tension on the shaft, felt as friction)

# -- the path (board frame, m) ----------------------------------------------------------------------------------------
PLAY_U = BOARD[0] / 2 - RIM_T  # 0.139: playable half length
PLAY_V = BOARD[1] / 2 - RIM_T  # 0.1165
LANE_W = (2 * PLAY_V - 3 * LANE_WALL_T) / 4  # 0.0545 m between walls
LANES_V = tuple(-PLAY_V + LANE_W / 2 + k * (LANE_W + LANE_WALL_T) for k in range(4))[::-1]  # A, B, C, D: +v to -v
WALLS_V = tuple((LANES_V[k] + LANES_V[k + 1]) / 2 for k in range(3))
GAP = 0.050  # the turn gap at a wall's open end
TURN_U = PLAY_U - GAP / 2  # 0.114
START_U, FINISH_U = -0.120, -0.100  # FINISH: just past the last hole
HOLE_US = (-0.070, -0.035, 0.0, 0.035, 0.070)  # along each lane
HOLE_OFF = 0.012  # holes sit this far either side of the lane's centre line, alternately


def walls() -> list[tuple[float, float, float, float]]:
    """Lane walls as (u0, u1, v, thickness): A|B open at +u, B|C open at -u, C|D open at +u."""
    return [(-PLAY_U, PLAY_U - GAP, WALLS_V[0], LANE_WALL_T),
            (-PLAY_U + GAP, PLAY_U, WALLS_V[1], LANE_WALL_T),
            (-PLAY_U, PLAY_U - GAP, WALLS_V[2], LANE_WALL_T)]


def lane_dirs() -> tuple[int, ...]:
    return (1, -1, 1, -1)  # lane A runs toward +u, B back, ...


def path_points() -> np.ndarray:
    """The drawn path's centre line, START to FINISH."""
    a, b, c, d = LANES_V
    return np.array([(START_U, a), (TURN_U, a), (TURN_U, b), (-TURN_U, b), (-TURN_U, c), (TURN_U, c), (TURN_U, d),
                     (FINISH_U, d)], float)


def holes() -> np.ndarray:
    """(N, 2) hole centres, numbered 1..N in path order (row k is hole k + 1)."""
    out = []
    for lane, (v, s) in enumerate(zip(LANES_V, lane_dirs())):
        us = HOLE_US if s > 0 else HOLE_US[::-1]
        for j, u in enumerate(us):
            side = 1 if (lane * len(us) + j) % 2 == 0 else -1
            out.append((u, v + side * HOLE_OFF))
    return np.array(out, float)


@dataclass(frozen=True)
class Path:
    pts: np.ndarray

    @property
    def seg_len(self) -> np.ndarray:
        return np.linalg.norm(np.diff(self.pts, axis=0), axis=1)

    @property
    def length(self) -> float:
        return float(self.seg_len.sum())

    def project(self, p) -> tuple[float, float]:
        """(arc length of the closest point on the path, distance to it)."""
        p = np.asarray(p, float)[:2]
        best = (0.0, math.inf)
        s0 = 0.0
        for a, b, ln in zip(self.pts[:-1], self.pts[1:], self.seg_len):
            t = float(np.clip((p - a) @ (b - a) / ln**2, 0.0, 1.0))
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


class Progress:
    """How far along the path the ball has come. The ball's place on the path is its closest point on the centre
    line; a jump of more than ``max_jump`` (across a wall's end, say) is not followed. ``passed`` is the number of
    holes whose line across the path the ball's centre has crossed (latched: the furthest point counts)."""

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
        s, _ = PATH.project(uv)
        if abs(s - self.s) <= self.max_jump:
            self.s = s
            self.s_max = max(self.s_max, s)

    @property
    def passed(self) -> int:
        return int(np.sum(HOLE_S < self.s_max))


def tilt_from_knob(theta: float) -> float:
    """Board tilt (rad) for a knob angle (rad)."""
    return float(np.clip(theta / RATIO, -TILT_MAX, TILT_MAX))


def ball_accel(tilt_x: float, tilt_y: float) -> np.ndarray:
    """Rolling ball's acceleration (u, v) on the board for the two tilts (5/7 g sin, solid sphere rolling).
    tilt_x turns about +x (raises +v), tilt_y about +y (lowers +u)."""
    g = 9.81 * 5 / 7
    return np.array([g * math.sin(tilt_y), -g * math.sin(tilt_x)])


def score_value(passed: int, foul: bool) -> float:
    """Holes passed / N (1.0: past the last hole, at FINISH); 0 after a foul."""
    return 0.0 if foul else passed / N_HOLES
