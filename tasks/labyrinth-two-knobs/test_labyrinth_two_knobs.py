"""Tests for labyrinth-two-knobs: path, holes, progress, gearing and the scorer. No Isaac Sim needed."""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import labmodel as M  # noqa: E402
import task  # noqa: E402
from labkit import LabState  # noqa: E402


def state(passed=0, fouls=(), fell=None):
    z = np.zeros(3)
    return {"labyrinth": LabState(pos=z, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=z, ang_vel=z, low=z, high=z,
                                  passed=passed, fouls=tuple(fouls), fell=fell)}


def _rect_dist(p, r):
    u0, u1, v0, v1 = r
    return math.hypot(max(u0 - p[0], 0.0, p[0] - u1), max(v0 - p[1], 0.0, p[1] - v1))


def test_eight_holes_in_path_order():
    assert M.N_HOLES == 8 and M.N_STEPS == 9
    assert np.all(np.diff(M.HOLE_S) > 0)


def test_the_ball_fits_along_the_printed_path():
    """Every point of the path leaves the ball clear of the partitions, the board's edge and the holes."""
    bu, bv = M.BOARD[0] / 2, M.BOARD[1] / 2
    for s in np.linspace(0, M.PATH.length, 3000):
        p = M.PATH.point(s)
        assert min(_rect_dist(p, w) for w in M.WALLS) > M.BALL_R
        assert bu - abs(p[0]) > M.BALL_R and bv - abs(p[1]) > M.BALL_R
        assert np.min(np.linalg.norm(M.HOLES - p, axis=1)) > M.HOLE_R + 0.004


def test_centre_line_reaches_finish():
    p = M.Progress()
    for s in np.linspace(0, M.PATH.length, 3000):
        p.step(M.PATH.point(s))
    assert p.fell is None and p.passed == M.N_STEPS


def test_stopping_short_of_finish_is_not_full_score():
    p = M.Progress()
    for s in np.linspace(0, M.PATH.length - 0.03, 3000):
        p.step(M.PATH.point(s))
    assert p.passed == M.N_HOLES and task.score(state(p.passed)) == 8 / 9


def test_ball_over_a_hole_falls_and_keeps_its_count():
    p = M.Progress()
    for s in np.linspace(0, M.HOLE_S[4] + 0.005, 1000):
        p.step(M.PATH.point(s))
    assert p.passed == 5
    p.step(M.HOLES[5])
    assert p.fell == 6
    for s in np.linspace(M.HOLE_S[5], M.PATH.length, 500):  # nothing counts after the fall
        p.step(M.PATH.point(s))
    assert p.passed == 5


def test_progress_does_not_jump_across_walls():
    p = M.Progress()
    p.step(M.PATH.point(0.02))
    p.step(M.PATH.point(0.9))  # a teleport far along the path is not followed
    assert p.passed == 0


def test_progress_is_latched_when_the_ball_rolls_back():
    p = M.Progress()
    for s in np.linspace(0, M.HOLE_S[2] + 0.01, 1000):
        p.step(M.PATH.point(s))
    for s in np.linspace(M.HOLE_S[2] + 0.01, 0.0, 1000):
        p.step(M.PATH.point(s))
    assert p.passed == 3


def test_gearing():
    assert math.isclose(M.tilt_from_knob(math.radians(25)), math.radians(1), rel_tol=1e-9)
    assert M.tilt_from_knob(math.radians(200)) == M.TILT_MAX
    assert M.tilt_from_knob(-math.radians(200)) == -M.TILT_MAX
    a = M.ball_accel(0.0, math.radians(1))
    assert math.isclose(a[0], 9.81 * 5 / 7 * math.sin(math.radians(1)))


def test_score():
    assert task.score(state(0)) == 0.0
    assert task.score(state(7)) == 7 / 9
    assert task.score(state(9)) == 1.0 and task.success(state(9))
    assert task.score(state(9, fouls=("left hand in the box",))) == 0.0
    assert task.score(state(4, fell=5)) == 4 / 9


def test_knobs_face_the_arms():
    px, nx = task.knob_grip("x")
    py, ny = task.knob_grip("y")
    assert px[1] < 0 and nx[1] < 0  # the end knob is on the right arm's side and faces it
    assert py[1] > 0 and ny[1] > 0
    assert abs(px[2] - py[2]) < 1e-9 and px[2] - task.TOP > 0.13  # level grips the arms can reach
