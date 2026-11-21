"""Unit tests for the smear rule and the score (no Isaac Sim needed): python -m pytest test_wedge_blood_smear.py"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from smearmodel import Smear, smear_score  # noqa: E402

DT = 1 / 240


def stroke(angle=35.0, v=0.045, wait=1.5, start=0.030, back_to=0.025, end=0.072, lift_at=None, pause_at=None,
           down_at=None, back_after=None, moved=0.0):
    """A hand-made trial: down ahead of the drop, back into it, wait, one stroke (with optional faults)."""
    s, x = Smear(), start
    for _ in range(24):
        s.step(DT, x, 0.0, False, angle)
    while x > back_to:
        x -= 0.005 * DT
        s.step(DT, x, 0.0, True, angle)
    for _ in range(int(wait / DT)):
        s.step(DT, x, 0.0, True, angle)
    paused = backed = False
    while x < end:
        if pause_at and x > pause_at and not paused:
            for _ in range(120):
                s.step(DT, x, 0.0, True, angle)
            paused = True
        if back_after and x > back_after and not backed:
            for _ in range(120):
                x -= 0.025 * DT
                s.step(DT, x, 0.0, True, angle)
            backed = True
        x += v * DT
        touching = not (lift_at and lift_at < x < lift_at + 0.004) and not (down_at and x < down_at)
        s.step(DT, x, 0.0, touching, angle)
    for _ in range(24):
        s.step(DT, x, 0.0, False, angle)
    return smear_score(s, moved), s


def test_good_smear_full_score():
    sc, s = stroke()
    assert sc == 1.0, s.summary()
    assert s.state == "feathered" and 0.030 <= s.length <= 0.045 and s.width >= 0.015


def test_length_follows_speed_and_angle():
    lengths = [stroke(v=v)[1].length for v in (0.035, 0.045, 0.060)]
    assert lengths[0] > lengths[1] > lengths[2]
    lengths = [stroke(angle=a)[1].length for a in (30, 35, 42)]
    assert lengths[0] > lengths[1] > lengths[2]


def test_too_slow_runs_off_the_end():
    sc, s = stroke(v=0.012, end=0.080)
    assert s.state == "runoff" and sc == 0.3


def test_too_fast_and_steep_is_short():
    sc, s = stroke(angle=55, v=0.09)
    assert s.length < 0.030 and sc <= 0.6


def test_no_wait_is_narrow_and_long():
    sc, s = stroke(wait=0.0)
    assert s.width < 0.015 and sc < 1.0


def test_lift_mid_stroke_is_blunt_and_restart_scores_zero():
    sc, s = stroke(lift_at=0.035)
    assert s.restart and sc == 0.0


def test_pause_leaves_a_ridge():
    sc, s = stroke(pause_at=0.035)
    assert s.ridge and sc == 0.6


def test_moving_back_over_the_smear_is_a_restart():
    sc, s = stroke(back_after=0.035)
    assert s.restart and sc == 0.0


def test_plough_from_behind():
    s, x = Smear(), 0.018
    for _ in range(int(0.1 / DT)):
        s.step(DT, x, 0.0, True, 35.0)
    while x < 0.070:
        x += 0.045 * DT
        s.step(DT, x, 0.0, True, 35.0)
    assert s.ploughed and smear_score(s, 0.0) <= 0.3


def test_never_touching_the_drop_scores_zero():
    s, x = Smear(), 0.030
    while x < 0.072:
        x += 0.045 * DT
        s.step(DT, x, 0.0, True, 35.0)
    assert s.length == 0 and smear_score(s, 0.0) == 0.0


def test_hover_without_contact_scores_zero():
    s, x = Smear(), 0.0245
    for _ in range(480):
        s.step(DT, x, 0.0, False, 35.0)
    while x < 0.072:
        x += 0.045 * DT
        s.step(DT, x, 0.0, False, 35.0)
    assert smear_score(s, 0.0) == 0.0


def test_base_slide_moved():
    assert stroke(moved=0.005)[0] == 0.3
    assert stroke(moved=0.011)[0] == 0.0


def test_film_tapers_to_a_feathered_end():
    _, s = stroke()
    nz = np.nonzero(s.h)[0]
    h = s.h[nz]
    assert h[0] > 3 * h[-1] and math.isclose(len(nz) * s.law.bin_m, s.length, abs_tol=0.001)


def test_ridge_is_judged_from_the_film():
    """A ridge is a band over 1.5x the film 1-4 mm either side, wherever it came from; a slight one is not."""
    _, s = stroke()
    assert not s.ridge
    i = int(0.040 / s.law.bin_m)
    s.h[i] *= 1.3
    assert not s.ridge and smear_score(s, 0.0) == 1.0
    s.h[i] *= 1.5
    assert s.ridge and smear_score(s, 0.0) == 0.6
    _, s = stroke(pause_at=0.035)  # the pause's band sits where the spreader stopped
    assert s.ridge and abs(np.argmax(s.h) * s.law.bin_m - 0.035) < 0.001


def test_restart_is_judged_from_the_film():
    """A restart is a second film edge or a gap band over 1 mm across the film."""
    _, s = stroke()
    assert not s.restart and s.edges == [s.start_x]
    i = int(0.040 / s.law.bin_m)
    s.h[i:i + 3] = 0.0  # 1.5 mm gap
    assert s.restart and smear_score(s, 0.0) == 0.0
    _, s = stroke()
    s.edges.append(0.045)  # a second film edge
    assert s.restart and smear_score(s, 0.0) == 0.0
    _, s = stroke(lift_at=0.035)  # put down again 4 mm on: its line of blood leaves a gap and a second edge
    assert len(s.edges) == 2 and not s.continuous
