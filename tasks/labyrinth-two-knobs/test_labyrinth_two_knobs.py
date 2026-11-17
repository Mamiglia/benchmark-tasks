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


def test_twenty_holes_in_path_order():
    assert M.N_HOLES == 20
    assert np.all(np.diff(M.HOLE_S) > 0)


def test_holes_inside_their_lanes_with_a_way_past():
    for u, v in M.HOLES:
        lane = min(M.LANES_V, key=lambda c: abs(c - v))
        off = abs(v - lane)
        assert off + M.HOLE_R < M.LANE_W / 2  # the hole is inside its lane
        band = (M.LANE_W / 2 - M.BALL_R) + off - M.HOLE_R  # where the ball's centre may pass beside it
        assert band > 0.02


def test_centre_line_misses_every_hole():
    p = M.Progress()
    for s in np.linspace(0, M.PATH.length, 2000):
        p.step(M.PATH.point(s))
    assert p.fell is None and p.passed == M.N_HOLES


def test_ball_over_a_hole_falls_and_keeps_its_count():
    p = M.Progress()
    for s in np.linspace(0, M.HOLE_S[6] + 0.005, 400):
        p.step(M.PATH.point(s))
    assert p.passed == 7
    p.step(M.HOLES[7])
    assert p.fell == 8
    for s in np.linspace(M.HOLE_S[7], M.PATH.length, 200):  # nothing counts after the fall
        p.step(M.PATH.point(s))
    assert p.passed == 7


def test_progress_does_not_jump_across_walls():
    p = M.Progress()
    p.step(M.PATH.point(0.02))
    p.step(M.PATH.point(0.9))  # a teleport far along the path is not followed
    assert p.passed == 0


def test_progress_is_latched_when_the_ball_rolls_back():
    p = M.Progress()
    for s in np.linspace(0, M.HOLE_S[2] + 0.01, 200):
        p.step(M.PATH.point(s))
    for s in np.linspace(M.HOLE_S[2] + 0.01, 0.0, 200):
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
    assert task.score(state(7)) == 7 / 20
    assert task.score(state(20)) == 1.0 and task.success(state(20))
    assert task.score(state(20, fouls=("left hand in the box",))) == 0.0
    assert task.score(state(12, fell=13)) == 12 / 20


def test_knobs_face_the_arms():
    px, nx = task.knob_grip("x")
    py, ny = task.knob_grip("y")
    assert px[1] < 0 and nx[1] < 0  # the end knob is on the right arm's side and faces it
    assert py[1] > 0 and ny[1] > 0
    assert abs(px[2] - py[2]) < 1e-9 and px[2] - task.TOP > 0.13  # level grips the arms can reach
