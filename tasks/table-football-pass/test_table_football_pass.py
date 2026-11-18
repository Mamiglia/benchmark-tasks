"""Tests for table-football-pass: table geometry, figure contact, goal and foul checks, and the latched scorer.
No Isaac Sim needed (tfkit is stubbed if yam_arena is missing)."""

from __future__ import annotations

import math
import pathlib
import sys
import types

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import tfmodel as M  # noqa: E402

DT = 1 / 240
REST = {4: (0.0, 0.0), 6: (0.0, 0.0)}


def ball_at(x, y):
    return np.array([x, y, M.FLOOR_Z + M.BALL_R])


def test_rods_and_lanes():
    assert abs(M.FIVE_Y - 0.115) < 1e-9 and abs(M.THREE_Y + 0.115) < 1e-9
    lane = M.BALL_START[0]
    # the ball's lane passes clear of the blue 5-bar, 2-bar and goalie (all hanging straight down)
    for k in (5, 7, 8):
        for y in np.linspace(M.rod_y(k) - 0.03, M.rod_y(k) + 0.03, 13):
            assert M.figure_gap(k, ball_at(lane, y)) > 0.01
    assert abs(lane - M.X_C) < M.GOAL_W / 2 - M.BALL_R
    # a 5-bar and a 3-bar figure can reach the lane within the rods' travel
    assert min(abs(lane - M.X_C - fx) for fx in M.FIG_X[4]) < M.TRAVEL[4]
    assert min(abs(lane - M.X_C - fx) for fx in M.FIG_X[6]) < M.TRAVEL[6]


def test_start_spot_is_clear_of_every_figure():
    b = np.array(M.BALL_START)
    for k in range(1, 9):
        assert M.figure_gap(k, b) > 0.005


def test_raised_figures_let_the_ball_pass_under():
    lane = M.BALL_START[0]
    for y in np.linspace(M.THREE_Y + 0.06, M.THREE_Y - 0.10, 50):
        assert M.figure_gap(6, ball_at(lane, y), lane - M.X_C, -math.radians(64)) > 0
    # hanging straight down, the middle figure blocks the lane
    assert M.figure_gap(6, ball_at(lane, M.THREE_Y + 0.0275), lane - M.X_C, 0.0) < M.TOUCH


def test_goal_and_off_table():
    assert M.in_goal(ball_at(M.X_C, M.Y_GOAL - 0.02))
    assert not M.in_goal(ball_at(M.X_C + 0.1, M.Y_GOAL - 0.02))
    assert not M.off_table(ball_at(M.X_C, M.Y_GOAL - 0.01))  # rolling in through the mouth
    assert M.off_table(ball_at(M.X_C + 0.1, M.Y_GOAL - 0.03))
    assert M.off_table(np.array([M.X_C, 0.0, M.WALL_TOP + 0.1]))
    assert M.in_own_goal(ball_at(M.X_C, M.Y_OWN + 0.02))


def run(path, rods_fn=lambda i, p: REST, **kw):
    """Step a Progress along a list of ball positions (one per physics step)."""
    pr = M.Progress()
    prev = np.asarray(path[0], float)
    for i, p in enumerate(path):
        p = np.asarray(p, float)
        pr.step(DT, p, (p - prev) / DT, rods_fn(i, p), **kw)
        prev = p
    return pr


def line(a, b, n):
    return [np.asarray(a) + (np.asarray(b) - np.asarray(a)) * t for t in np.linspace(0, 1, n)]


LANE = M.BALL_START[0]
S3 = LANE - M.X_C  # 3-bar slide that puts its middle figure on the lane
TOUCH5 = ball_at(LANE, M.FIVE_Y - 0.028)  # against the (aligned) 5-bar figure's foot
TOUCH3 = ball_at(LANE, M.THREE_Y + 0.0275)  # against the back of the 3-bar figure's foot


def rods_aligned(i, p):
    return {4: (S3, 0.0), 6: (S3, 0.0)}


def test_full_sequence_scores_one():
    path = [TOUCH5] * 5 + line(TOUCH5, TOUCH3, 200) + [TOUCH3] * 300 + line(TOUCH3, ball_at(LANE, M.Y_GOAL - 0.03),
                                                                               200)
    pr = run(path, rods_aligned)
    assert pr.stage == "goal" and pr.score() == 1.0


def test_untrapped_goal_and_partial_stages():
    shot = line(TOUCH3, ball_at(LANE, M.Y_GOAL - 0.03), 200)
    pr = run([TOUCH5] * 5 + line(TOUCH5, TOUCH3, 200) + shot, rods_aligned)
    assert pr.score() == 0.7  # received and shot, never still for 1 s
    pr = run([TOUCH5] * 5 + line(TOUCH5, TOUCH3, 200) + [TOUCH3] * 300, rods_aligned)
    assert pr.score() == 0.5  # trapped, no shot
    pr = run([TOUCH5] * 5 + line(TOUCH5, TOUCH3, 200) + [TOUCH3] * 100, rods_aligned)
    assert pr.score() == 0.3  # received, not still for long enough
    away = {4: (S3, 0.0), 6: (-M.TRAVEL[6], math.radians(-80))}  # 3-bar slid away and raised
    pr = run([TOUCH5] * 5 + line(TOUCH5, ball_at(LANE, M.Y_GOAL - 0.03), 400), lambda i, p: away)
    assert pr.score() == 0.1  # goal straight from the 5-bar
    pr = run([ball_at(LANE, 0.08)] * 10)
    assert pr.score() == 0.0


def test_fouls_latch_to_zero():
    full = [TOUCH5] * 5 + line(TOUCH5, TOUCH3, 200) + [TOUCH3] * 300 + line(TOUCH3, ball_at(LANE, M.Y_GOAL - 0.03),
                                                                               200)
    pr = run(full, rods_aligned, hand_gap=0.005)
    assert pr.score() == 0.0 and "hand at the ball" in pr.fouls
    pr = run(full, rods_aligned, hand_in_table=True)
    assert pr.score() == 0.0
    spun = lambda i, p: {4: (S3, 0.0), 6: (S3, -0.02 * i)}  # noqa: E731
    pr = run(full, spun)
    assert "rod 6 spun past one turn" in pr.fouls and pr.score() == 0.0
    pr = run([TOUCH5] * 5 + line(TOUCH5, ball_at(M.X_C, M.Y_OWN + 0.03), 300), rods_aligned)
    assert "own goal" in pr.fouls and pr.score() == 0.0


def test_task_scorer_reads_the_state():
    if "yam_arena" not in sys.modules:
        try:
            import yam_arena  # noqa: F401
        except ImportError:  # stub just enough for task.py and tfkit's dataclasses
            from dataclasses import dataclass, field

            @dataclass(frozen=True)
            class Box:
                name: str
                pos: tuple = (0.0, 0.0, 0.0)
                size: tuple = (0.1, 0.1, 0.1)
                yaw: float = 0.0
                color: tuple = (0.7, 0.7, 0.7)

            @dataclass(frozen=True)
            class ObjectState:
                pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
                quat_xyzw: np.ndarray = field(default_factory=lambda: np.zeros(4))
                lin_vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
                ang_vel: np.ndarray = field(default_factory=lambda: np.zeros(3))
                low: np.ndarray = field(default_factory=lambda: np.zeros(3))
                high: np.ndarray = field(default_factory=lambda: np.zeros(3))

            pkg = types.ModuleType("yam_arena")
            tf = types.ModuleType("yam_arena.task_file")
            tf.Box, tf.ObjectState, tf.TABLE_TOP = Box, ObjectState, {"station": M.TOP, "tabletop": 0.75}
            sys.modules["yam_arena"], sys.modules["yam_arena.task_file"] = pkg, tf
    import task
    from tfkit import TableState

    z = np.zeros(3)
    st = TableState(pos=z, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=z, ang_vel=z, low=z, high=z, score=1.0)
    assert task.score({"football_table": st}) == 1.0 and task.success({"football_table": st})
    assert "pass the ball" in task.INSTRUCTION
