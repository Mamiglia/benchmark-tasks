"""Unit tests for the pin rule, the lid geometry and the score bookkeeping (NumPy only; no Isaac Sim)."""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import plugmodel as M  # noqa: E402

HIGH = M.TOP + 0.13


def test_rolls_off_twelve():
    for seed in range(50):
        r = M.roll_for(seed)
        assert -40.0 <= r <= -15.0 and r == round(r)


def test_pin_resistance_rises_over_the_last_15_mm():
    assert M.pin_resistance(0.0) == 0.0
    assert M.pin_resistance(M.ENGAGED) == 0.0
    assert math.isclose(M.pin_resistance((M.ENGAGED + M.DEPTH_FULL) / 2), M.F_PINS / 2)
    assert math.isclose(M.pin_resistance(M.DEPTH_FULL), M.F_PINS)


def test_friction_stops_slow_motion_and_caps_fast_motion():
    dt, m = 1 / 240, M.PLUG_MASS
    # moving in slowly: just the force that stops it this step
    assert math.isclose(M.axial_friction(M.DEPTH_FULL, 0.01, 0.0, m, dt), -m * 0.01 / dt)
    # driven in hard: capped at the full resistance, outward
    assert math.isclose(M.axial_friction(M.DEPTH_FULL, 0.5, 0.0, m, dt), -M.F_PINS)
    # at rest under the cable's pull: friction balances the pull, no more
    assert math.isclose(M.axial_friction(M.DEPTH_FULL, 0.0, -1.5, m, dt), 1.5)
    # not engaged: no friction at all
    assert M.axial_friction(0.010, 0.05, -1.5, m, dt) == 0.0


def test_depth_and_roll():
    p, r = M.coupled_pose(0.02)
    d, off, roll = M.depth_of(p, r)
    assert math.isclose(d, 0.02) and off < 1e-12 and abs(roll) < 1e-9
    _, r0 = M.start_pose(0)
    assert math.isclose(M.depth_of(p, r0)[2], M.roll_for(0), abs_tol=1e-9)
    assert M.aligned(p, r) and not M.aligned(p + [0, 0.01, 0], r)


def test_lid_angles():
    # riding on the ridge the lid sits higher (more open) than with its catch down in the notch
    assert M.LID_RIDE > M.LID_CAUGHT
    assert math.radians(80) < M.LID_CAUGHT < M.LID_CAUGHT_MAX < M.LID_RIDE < math.radians(95)
    # with the plug seated, the catch lies behind the ridge's end (it can drop into the notch)
    r = M.rot_y(M.LID_CAUGHT)
    near_edge = M.HINGE[0] + (r @ np.array([0.008, 0.0, -0.069]))[0]
    assert near_edge - (M.XF + M.DEPTH_FULL) < M.RIDGE_X[0]
    # ... but 4 mm short of seated it would still sit on the ridge
    assert near_edge - (M.XF + M.DEPTH_FULL - 0.004) > M.RIDGE_X[0]


def run(mon, steps, depth, lid, low=HIGH, off=0.0):
    for _ in range(steps):
        mon.step(depth, off, low, lid)


def test_full_sequence_scores_one():
    mon = M.Monitor()
    run(mon, 5, -0.2, 0.0, low=M.TOP + 0.016)  # in the V-block
    run(mon, 5, -0.2, 0.0)  # lifted
    run(mon, 5, -0.005, math.radians(115))
    assert mon.score(-0.005, math.radians(115), True, True) == 0.1
    run(mon, 5, 0.020, math.radians(115))
    assert mon.score(0.020, math.radians(115), True, True) == 0.3
    run(mon, 5, 0.030, math.radians(115))
    assert mon.score(0.030, math.radians(115), False, True) == 0.6  # lid still held
    assert mon.score(0.030, math.radians(115), True, False) == 0.3  # plug still held
    assert mon.score(0.030, M.LID_CAUGHT, False, False) == 1.0
    assert mon.score(0.030, M.LID_RIDE, False, False) == 0.6  # lid on the ridge: not caught


def test_drop_is_zero_and_latched():
    mon = M.Monitor()
    run(mon, 5, -0.2, 0.0)
    run(mon, 5, -0.2, 0.0, low=M.TOP + 0.005)
    run(mon, 5, 0.030, M.LID_CAUGHT)
    assert mon.score(0.030, M.LID_CAUGHT, False, False) == 0.0


def test_shut_lid_gives_nothing():
    mon = M.Monitor()
    run(mon, 5, -0.2, 0.0)
    run(mon, 5, -0.006, 0.0)  # plug against the shut lid
    assert mon.score(-0.006, 0.0, False, False) == 0.0


def test_off_axis_does_not_engage():
    mon = M.Monitor()
    run(mon, 5, 0.020, math.radians(115), off=0.01)
    assert mon.score(0.020, math.radians(115), True, True) == 0.0
