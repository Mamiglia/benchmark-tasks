"""Unit tests for the mechanism rules, the geometry and the score (no Isaac Sim needed): pytest test_lever_arch_filing.py"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bindermodel import (ARCH_PTS, HOLE, HOLES, LEVER_OPEN, POST_D, POST_TOP, POST_X, POST_Y, RATIO, LeverParams,  # noqa: E402
                         arch_point, arch_target, arch_tip, lever_effort, plate_force, score_state, strip_boxes,
                         threaded)

P = LeverParams()
D = math.radians


def test_lever_is_bistable_and_within_a_finger():
    assert plate_force(0) < 0 and plate_force(10) < 0 and plate_force(35) < 0  # shut side pulls it shut
    assert plate_force(P.centre_deg) == 0.0
    assert plate_force(50) > 0 and plate_force(79) > 0  # open side holds it open
    assert max(abs(plate_force(d)) for d in np.linspace(0, 80, 801)) <= 10.0 + 1e-9  # well within 40 N


def test_cam_reaction_is_consistent():
    # no reaction when the arch follows the lever; a held-back arch pushes the lever back open
    assert math.isclose(lever_effort(D(30), 0.0, RATIO * D(30)), plate_force(30) * P.arm_m)
    assert lever_effort(D(30), 0.0, RATIO * D(30) + D(10)) > lever_effort(D(30), 0.0, RATIO * D(30))
    assert arch_target(LEVER_OPEN * 2) == arch_target(LEVER_OPEN * 3)


def test_rings_shut_on_the_posts_and_open_clear_of_them():
    for i in range(2):
        tip = arch_tip(i, 0.0)
        assert abs(tip[0] - POST_X) < 1e-9 and abs(tip[1] - POST_Y[i]) < 1e-9
        assert 0 < tip[2] - POST_TOP <= POST_D / 2 + 0.0005  # on the post's top (0.2 mm gap)
        open_tip = arch_tip(i, RATIO * D(80))
        assert open_tip[0] - POST_X > 0.03  # swung back more than 3 cm: the post's tip is free
    # every point of the open moving half stays on the spine side of the posts (the sheets pass)
    for p in ARCH_PTS:
        assert arch_point(p, RATIO * D(80))[0] > POST_X + 0.02


def test_holes_and_clearance():
    assert HOLE - POST_D > 0.001
    assert math.isclose(abs(HOLES[0][1] - HOLES[1][1]), 0.080)
    boxes = strip_boxes((1, 1, 1), (0, 0, 0))
    for (cx, cy, _), (sx, sy, _), *_ in [b for b in boxes if len(b) == 3]:  # no collider covers a hole
        for hx, hy, _ in HOLES:
            assert abs(cx - hx) >= sx / 2 + HOLE / 2 - 1e-9 or abs(cy - hy) >= sy / 2 + HOLE / 2 - 1e-9


def test_threaded_needs_both_axis_and_depth():
    R = np.eye(3)
    pos = np.array([POST_X - HOLES[0][0], 0.0, POST_TOP - 0.03])
    assert threaded(pos, R, 0) and threaded(pos, R, 1)
    assert not threaded(pos + [0.004, 0, 0], R, 0)
    assert not threaded(pos + [0, 0, 0.03], R, 0)  # resting on the tip


def test_score_covers_every_outcome():
    assert score_state(True, 2, True, True, True) == 0.0
    assert score_state(False, 2, True, True, True) == 1.0
    assert score_state(False, 2, True, False, True) == 0.5
    assert score_state(False, 2, False, True, True) == 0.5
    assert score_state(False, 1, True, True, True) == 0.2
    assert score_state(False, 0, True, True, True) == 0.1
    assert score_state(False, 0, True, True, False) == 0.0
