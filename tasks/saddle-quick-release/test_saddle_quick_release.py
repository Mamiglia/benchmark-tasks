"""Unit tests for the clamp rules, the geometry and the score (no Isaac Sim needed): pytest test_saddle_quick_release.py"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from qrmodel import (A, COLLAR_TOP, LEVER_LEN, PIVOT, REAR, Clamp, ClampParams, lever_dir, lever_tangent,  # noqa: E402
                     paddle, paddle_force, post_point, rot_axis, score_state)

P = ClampParams()
R = math.radians


def test_axes():
    assert abs(np.dot(A, REAR)) < 1e-12 and abs(np.linalg.norm(A) - 1) < 1e-12
    assert np.allclose(np.cross([0, 1, 0], lever_dir(0.0)), lever_tangent(0.0))  # +y opens the lever
    assert np.isclose(np.linalg.norm(paddle(R(70)) - PIVOT), LEVER_LEN)


def test_post_point_moves_along_and_about_the_tube():
    p = np.array([0.075, 0.0, 0.06])
    assert np.allclose(post_point(p, 0.04, 0.0) - post_point(p, 0.0, 0.0), 0.04 * A)
    q = post_point(p, 0.0, R(10)) - COLLAR_TOP
    assert np.isclose(np.dot(q, A), np.dot(p, A)) and np.isclose(np.linalg.norm(q), np.linalg.norm(p))
    assert post_point(p, 0.0, R(10))[1] > 0  # + turns the nose to the bike's left
    assert np.allclose(rot_axis(A, R(30)) @ A, A)


def test_paddle_force_is_over_centre():
    assert paddle_force(100) == 0.0
    assert math.isclose(paddle_force(P.peak_deg), P.peak_n)
    assert 0 < paddle_force(60) < P.peak_n and 0 < paddle_force(15) < P.peak_n
    assert paddle_force(P.dead_deg) == 0.0
    assert paddle_force(5) == -P.hold_n  # past the dead point it pulls the lever shut
    assert max(paddle_force(d) for d in np.linspace(0, 120, 1201)) <= P.peak_n + 1e-9  # within a YAM finger (~40 N)


def test_clamp_grips_only_when_nearly_closed():
    c = Clamp(P)
    lever, f, t = c.step(0.0, 0.0, 0.001, 0.0, R(12), 0.0)
    assert c.gripping and f < 0 and lever < 0  # closed: holds the post, lever pulled shut
    c.step(R(60), 0.0, 0.0, 0.0, R(12), 0.0)
    assert not c.gripping
    _, f, t = c.step(R(60), 0.0, 0.03, 0.2, R(5), -1.0)
    assert -P.slide_fric_n <= f < 0 and 0 < t <= P.turn_fric_nm  # loose: friction only
    assert P.slide_fric_n < 0.30 * 9.81 * math.cos(R(15))  # the loose post sinks under its own weight
    lever, _, _ = c.step(R(25), 0.0, 0.04, 0.0, 0.0, 0.0)
    assert c.gripping and lever > 0  # short of the dead point: grips while held, but pushes the lever open
    assert c.hold == (0.04, 0.0)
    _, f, t = c.step(R(25), 0.0, 0.0413, 0.0, R(1), 0.0)
    assert f < -15.0  # a 20 N push moves the clamped post about 1.3 mm
    assert t < -0.9


def test_score():
    assert score_state(40, 0, 0, True, False) == 1.0
    assert score_state(44, -4.9, 11, True, False) == 1.0
    assert score_state(40, 8, 0, True, False) == 0.8
    assert score_state(30, 0, 0, True, False) == 0.5
    assert score_state(30, 8, 0, True, False) == 0.3
    assert score_state(40, 0, 20, True, False) == 0.0  # lever short of its dead point
    assert score_state(40, 0, 0, False, False) == 0.0  # still holding something
    assert score_state(5, 0, 0, True, False) == 0.0  # never raised (straightened only)
    assert score_state(40, 0, 0, True, True) == 0.0  # post pulled past MIN INSERT at some point
