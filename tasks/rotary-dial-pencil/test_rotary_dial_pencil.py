"""Unit tests for the dial model and the score (no Isaac Sim needed): pytest test_rotary_dial_pencil.py"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import dialmodel as M  # noqa: E402

DT = 1 / 240


def run(dialer, theta_path, t0=0.0):
    t = t0
    for th in theta_path:
        t += DT
        dialer.step(t, th)
    return t


def pull(theta_max, speed_out=200.0, back_speed=None, hold_s=0.2):
    """Wheel angles for one pull: out at speed_out deg/s, held at the stop, back at back_speed (governor default)."""
    back = back_speed or math.degrees(M.GOV_SPEED)
    out = list(np.arange(0.0, theta_max, speed_out * DT)) + [theta_max] * int(hold_s / DT)
    ret = list(np.arange(theta_max, 0.0, -back * DT)) + [0.0] * 60
    return out + ret


def test_hole_geometry():
    assert M.hole_psi(1) == M.STOP_PSI + M.HOLE_LAG + M.THETA1
    assert all(abs(M.hole_psi(n + 1) - M.hole_psi(n) - M.PITCH) < 1e-9 for n in range(1, 10))
    for n in range(1, 11):  # a pull that brings hole n to the stop sends n pulses
        assert M.pulses_for(M.hole_theta(n)) == n
        assert M.pulses_for(M.hole_theta(n) - M.PULSE_TOL - 0.5) == n - 1
    R = M.dial_axes()
    assert np.allclose(R.T @ R, np.eye(3)) and abs(np.linalg.det(R) - 1) < 1e-9
    assert R[2, 2] == math.cos(M.TILT) and R[0, 2] < 0  # face tilted toward the arms


def test_dial_411_scores_one():
    d = M.Dialer()
    t = 0.0
    for digit in M.NUMBER:
        t = run(d, pull(M.hole_theta(M.hole_for_digit(digit))), t)
    assert d.digits == [4, 1, 1]
    assert M.score_value(d.digits, False) == 1.0
    assert M.score_value(d.digits, True) == 0.0


def test_short_pull_dials_a_lower_digit():
    d = M.Dialer()
    t = run(d, pull(M.hole_theta(4)))
    run(d, pull(M.hole_theta(1) - 20), t)  # let go 20 degrees before the stop: no pulse at all for "1"
    assert d.digits == [4]
    d = M.Dialer()
    run(d, pull(M.hole_theta(4) - 20))
    assert d.digits == [3]
    assert M.score_value(d.digits, False) == 0.0


def test_held_back_run_back_is_a_wrong_digit():
    d = M.Dialer()
    run(d, pull(M.hole_theta(4), back_speed=60.0))
    assert d.pulls[0].held and d.digits == [-1]
    assert M.score_value(d.digits, False) == 0.0


def test_pushed_again_on_the_way_back():
    d = M.Dialer()
    th = M.hole_theta(4)
    path = list(np.arange(0, th, 1.0)) + list(np.arange(th, 60, -1.0)) + list(np.arange(60, th, 1.0)) + \
        list(np.arange(th, 0, -1.0)) + [0.0] * 10
    run(d, path)
    assert len(d.pulls) == 1 and d.pulls[0].repushed and d.digits == [-1]


def test_partial_and_extra_digits():
    assert M.score_value([4], False) == 1 / 3
    assert M.score_value([4, 1], False) == 2 / 3
    assert M.score_value([4, 2, 1], False) == 1 / 3
    assert M.score_value([4, 1, 2], False) == 2 / 3
    assert M.score_value([4, 1, 1, 1], False) == 2 / 3
    assert M.score_value([], False) == 0.0
    d = M.Dialer()
    run(d, [5.0] * 100 + [0.0] * 10)  # a wiggle under PULL_START is no pull
    run(d, list(np.arange(0, 30, 1.0)) + [0.0] * 10)  # a pull short of hole 1 sends no pulse: dials nothing
    assert d.digits == [] and len(d.pulls) == 1


def test_unfinished_pull_does_not_count():
    d = M.Dialer()
    run(d, list(np.arange(0, M.hole_theta(4), 2.0)))
    assert d.digits == [] and d.cur is not None


def test_spring():
    assert M.spring_torque(0.0) == M.SPRING_T0
    full = M.spring_torque(math.radians(M.hole_theta(10)))
    assert 0.02 < M.SPRING_T0 and full / M.HOLE_R < 2.5  # under 2.5 N at the finger hole, fully wound
