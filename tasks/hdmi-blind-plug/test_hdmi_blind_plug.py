"""Unit tests for the port geometry, the contact-spring rule and the score bookkeeping (NumPy only; no Isaac Sim)."""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import hdmimodel as M  # noqa: E402


def test_start_poses_lie_flat_near_the_right_arm():
    for seed in range(50):
        p, r = M.start_pose(seed)
        assert abs(p[0] - M.START_XY[0]) <= 0.0201 and abs(p[1] - M.START_XY[1]) <= 0.0201
        assert math.isclose(r[2, 2], 1.0)  # wide side up
        yaw = math.degrees(math.atan2(r[1, 0], r[0, 0]))
        assert -20.0 <= yaw <= 20.0 and abs(yaw - round(yaw)) < 1e-9


def test_plugged_pose_points_up_with_the_wide_side_to_the_screen():
    p, r = M.plugged_pose(M.SHELL_LEN)
    assert np.allclose(r[:, 0], [0, 0, 1]) and np.allclose(r[:, 2], [-1, 0, 0])
    d, off, tilt = M.depth_of(p, r)
    assert math.isclose(d, M.SHELL_LEN) and off < 1e-12 and tilt < 1e-6
    assert math.isclose(p[2], M.ZM)  # overmold face on the port face


def test_port_hidden_under_the_panel():
    # the mouth is up in the bay, above the panel's lower edge, and behind the screen's face
    assert M.ZM > M.PB and M.FRONT_X < M.PX - M.SHELL_T / 2 < M.PX + M.SHELL_T / 2 < M.BACK_X
    assert M.BAY[0] < M.PX - M.OVER_T / 2 and M.PX + M.OVER_T / 2 < M.BAY[1]  # the overmold fits the bay


def test_key_blocks_an_upside_down_plug():
    # right way round, the wide part clears the filled corners; turned over, it overlaps them
    wide_half, narrow_half = M.SHELL_W / 2, M.SHELL_W / 2 - M.CHAMFER_W
    key_inner = narrow_half + M.CLR
    assert wide_half > key_inner + 0.0005
    assert M.SHELL_W - 2 * M.CHAMFER_W < 2 * key_inner


def test_contact_springs_take_hold_only_inside_and_on_axis():
    assert not M.grips(0.003, 0.0, 0.0)  # not in far enough
    assert M.grips(0.005, 0.0004, 1.0)
    assert not M.grips(0.005, 0.002, 1.0) and not M.grips(0.005, 0.0004, 5.0)
    assert not M.lets_go(M.BITE) and M.lets_go(M.BITE - 0.001)
    # a seated plug is held against far more than its weight and the cable's pull
    assert M.F_INS > 10 * (M.PLUG_MASS * 9.81 + M.CORD_PULL)


def test_cord_pulls_toward_the_generator_once_lifted():
    f = M.cord_force(np.array([M.PX, M.PY, M.ZM]))
    assert math.isclose(np.linalg.norm(f), M.CORD_PULL) and f[1] < 0 and f[2] < 0
    p, _ = M.start_pose(0)
    assert not M.cord_force(p).any()  # lying on the desk, the cable is slack


def test_score_bookkeeping():
    m = M.Monitor()
    assert m.score(-0.05, 0.1, True) == 0.0
    m.step(0.002, 0.0)
    assert not m.entered  # 2 mm is not in
    m.step(0.004, 0.003)
    assert not m.entered  # off the port axis: not the HDMI port
    m.step(0.004, 0.0)
    assert m.entered
    assert m.score(-0.02, 0.01, True) == 0.25  # came back out
    assert m.score(0.006, 0.0, True) == 0.5  # in, not home
    assert m.score(M.SHELL_LEN, 0.0, False) == 0.5  # home, still held
    assert m.score(M.SHELL_LEN, 0.0, True) == 1.0
    m.reset()
    assert m.score(-0.02, 0.01, True) == 0.0
