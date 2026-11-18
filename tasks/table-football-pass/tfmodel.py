"""tfmodel: the table-football table's geometry, figure contact, and the trial's progress and score. Pure Python.

Frame: the env frame (x forward from the arms, y left, z up; metres). The table's long side faces the arms; its rods
run along x, handles toward the arms. The robot's team attacks the goal at the -y end.

Rod angle ``theta``: right-handed about +x (the rod's axis, pointing away from the arms). theta = 0 hangs the figures
straight down; theta < 0 swings the feet toward -y (toward the goal being attacked, "forward").
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

TOP = 0.7475  # station table top

# -- the table (Sport Squad FX40-class 40 in tabletop table, as modelled) -------------------------------------------
FLOOR_Z = TOP + 0.050  # playfield surface: 2 cm feet + 3 cm cabinet base
ROD_Z = TOP + 0.130  # rod axes, 8 cm above the playfield
X_NEAR, X_FAR = 0.420, 0.890  # inner faces of the near (handle side) and far side walls
X_C = (X_NEAR + X_FAR) / 2  # 0.655, the playfield's centre line along the rods
WALL_T = 0.015
WALL_TOP = TOP + 0.160
Y_C = 0.0575  # playfield centre (y)
FIELD_L = 0.970  # between the end walls
Y_GOAL = Y_C - FIELD_L / 2  # -0.4275: inner face of the end wall with the goal being attacked
Y_OWN = Y_C + FIELD_L / 2  # +0.5425: the robot team's own goal
GOAL_W, GOAL_H = 0.150, 0.065  # goal mouth, centred on X_C, from the playfield up
POCKET = 0.060  # depth of the goal pocket behind the end wall
PITCH = 0.115  # between rods


def rod_y(k: int) -> float:
    """Rod k = 1..8 from the robot team's goal (+y) to the opponent's goal (-y)."""
    return Y_C + (4.5 - k) * PITCH


# Our rods (handles toward the arms): goalie 1, 2-bar 2, 5-bar 4, 3-bar 6. Opponent: 3-bar 3, 5-bar 5, 2-bar 7,
# goalie 8 (handles on the far side, clamped with the figures hanging straight down).
FIVE_Y, THREE_Y = rod_y(4), rod_y(6)  # 0.115, -0.115
OPP5_Y, OPP2_Y, OPPG_Y = rod_y(5), rod_y(7), rod_y(8)
FIG_X = {  # figure centres along x at zero slide
    1: (0.0,), 2: (-0.12, 0.12), 3: (-0.13, 0.0, 0.13), 4: tuple(k * 0.085 for k in (-2, -1, 0, 1, 2)),
    5: tuple(k * 0.085 for k in (-2, -1, 0, 1, 2)), 6: (-0.13, 0.0, 0.13), 7: (-0.12, 0.12), 8: (0.0,)}
TRAVEL = {4: 0.050, 6: 0.065}  # slide each way (bumpers)

# A figure, in its rod's frame (x along the rod, z down at theta = 0 is -z): (centre, half-size) boxes.
FOOT_BOTTOM = ROD_Z - FLOOR_Z - 0.003  # 0.077: rod axis to the sole; 3 mm above the playfield
FIGURE_BOXES = (
    ((0.0, 0.0, 0.022), (0.009, 0.008, 0.012)),  # head and shoulders above the rod
    ((0.0, 0.0, -0.022), (0.010, 0.007, 0.032)),  # body
    ((0.0, 0.0, -FOOT_BOTTOM + 0.0125), (0.011, 0.012, 0.0125)),  # foot block, 22 x 24 x 25 mm
)
FIG_MASS = 0.012

# Handles: rubber, 28 mm diameter, 80 mm long; centre at x HANDLE_X with the rod at zero slide.
HANDLE_X, HANDLE_R, HANDLE_L = 0.290, 0.014, 0.080
ROD_R = 0.0065

# Ball: the table's hard plastic ball, 31 mm, 12 g.
BALL_R, BALL_MASS = 0.0155, 0.012
BALL_START = (X_C + 0.0425, FIVE_Y - 0.035, FLOOR_Z + BALL_R)  # marked spot 3.5 cm ahead of the 5-bar

# -- scoring constants ------------------------------------------------------------------------------------------------
TOUCH = 0.003  # a figure "touches" the ball within 3 mm of its surface
STILL_D = 0.003  # "at rest": the ball's centre stays within 3 mm
TRAP_S = 1.0  # at rest against or under a 3-bar figure this long
HAND_BALL = 0.01  # a finger or tool point within 1 cm of the ball's surface
SPIN_MAX = 2 * math.pi

STAGE_SCORE = {"none": 0.0, "crossed": 0.1, "received": 0.3, "trapped": 0.5, "goal_untrapped": 0.7, "goal": 1.0}
STAGES = tuple(STAGE_SCORE)


def rot_x(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])


def box_distance(p: np.ndarray, centre, half) -> float:
    """Distance from point p to an axis-aligned box (0 inside)."""
    d = np.abs(np.asarray(p) - np.asarray(centre)) - np.asarray(half)
    return float(np.linalg.norm(np.maximum(d, 0.0)))


def figure_gap(k: int, ball, slide: float = 0.0, theta: float = 0.0) -> float:
    """Gap (m) between the ball's surface and the nearest figure of rod k at this slide and angle."""
    rel = np.asarray(ball, float) - np.array([X_C + slide, rod_y(k), ROD_Z])
    rel = rot_x(theta).T @ rel
    best = 1e9
    for fx in FIG_X[k]:
        q = rel - np.array([fx, 0.0, 0.0])
        for c, h in FIGURE_BOXES:
            best = min(best, box_distance(q, c, h))
    return best - BALL_R


def in_goal(ball) -> bool:
    x, y, z = ball
    return y < Y_GOAL - BALL_R and abs(x - X_C) < GOAL_W / 2 and z < FLOOR_Z + GOAL_H


def in_own_goal(ball) -> bool:
    x, y, z = ball
    return y > Y_OWN + BALL_R and abs(x - X_C) < GOAL_W / 2 and z < FLOOR_Z + GOAL_H


def off_table(ball) -> bool:
    """Out of the playfield and not in a goal: over a wall, or bounced up 5 cm over the wall tops."""
    x, y, z = ball
    if in_goal(ball) or in_own_goal(ball):
        return False
    mouth = abs(x - X_C) < GOAL_W / 2 and z < FLOOR_Z + GOAL_H  # on its way into a goal
    y_lo, y_hi = (Y_GOAL - WALL_T - POCKET, Y_OWN + WALL_T + POCKET) if mouth else (Y_GOAL - 0.005, Y_OWN + 0.005)
    return z > WALL_TOP + 0.05 or z < FLOOR_Z - 0.02 or not (X_NEAR - 0.005 < x < X_FAR + 0.005) or not y_lo < y < y_hi


@dataclass
class Progress:
    """Latched stages of one trial, stepped every physics step.

    - ``crossed``: after one of the 5-bar's figures touched it, the ball's centre has passed the opponent's 5-bar.
    - ``received``: after crossing, a 3-bar figure has touched it.
    - ``trapped``: after receiving, it has stayed within 3 mm of one spot while touching a 3-bar figure for 1 s.
    - goal: the ball's centre is past the goal line inside the mouth. 1.0 when it was trapped first and the last of
      the robot's figures to touch it was a 3-bar figure (the shot); 0.7 when it was received but not trapped and
      last touched by the 3-bar; a goal straight from the 5-bar, or after the 5-bar touched it again, adds nothing.
    - foul (0 for the trial): a hand at the ball or inside the table, the ball off the table, an own goal, a rod
      turned more than one full turn from its start.
    """

    stage: str = "none"
    last_own: str = ""  # "5" or "3": the robot's rod that touched the ball last
    touched5: bool = False
    still_t: float = 0.0
    still_at: tuple = ()
    goal: bool = False
    fouls: set = field(default_factory=set)
    theta0: dict = field(default_factory=dict)

    def _raise(self, stage: str):
        if STAGES.index(stage) > STAGES.index(self.stage):
            self.stage = stage

    def step(self, dt: float, ball, vel, rods: dict, hand_gap: float = 1.0, hand_in_table: bool = False):
        """``rods``: {4: (slide, theta), 6: (slide, theta)}. ``hand_gap``: the nearest finger/tool point's distance
        to the ball's surface."""
        for k, (_, th) in rods.items():
            self.theta0.setdefault(k, th)
            if abs(th - self.theta0[k]) > SPIN_MAX:
                self.fouls.add(f"rod {k} spun past one turn")
        if hand_gap < HAND_BALL:
            self.fouls.add("hand at the ball")
        if hand_in_table:
            self.fouls.add("hand inside the table")
        if self.goal:
            return
        ball = np.asarray(ball, float)
        if in_own_goal(ball):
            self.fouls.add("own goal")
            return
        if off_table(ball):
            self.fouls.add("ball off the table")
            return
        t5 = figure_gap(4, ball, *rods[4]) < TOUCH
        t3 = figure_gap(6, ball, *rods[6]) < TOUCH
        if t5:
            self.touched5, self.last_own = True, "5"
        if t3:
            self.last_own = "3"
        if self.touched5 and ball[1] < OPP5_Y and self.stage == "none":
            self._raise("crossed")
        if t3 and self.stage == "crossed":
            self._raise("received")
        if self.stage == "received":
            moved = not self.still_at or float(np.linalg.norm(ball - np.asarray(self.still_at))) > STILL_D
            if t3 and not moved:
                self.still_t += dt
                if self.still_t >= TRAP_S:
                    self._raise("trapped")
            else:
                self.still_t, self.still_at = 0.0, tuple(ball)
        if in_goal(ball):
            self.goal = True
            if self.last_own == "3" and self.stage == "trapped":
                self._raise("goal")
            elif self.last_own == "3" and self.stage in ("received", "trapped"):
                self._raise("goal_untrapped")

    def score(self) -> float:
        return 0.0 if self.fouls else STAGE_SCORE[self.stage]
