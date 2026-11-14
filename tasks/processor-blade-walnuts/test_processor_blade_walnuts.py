"""Hub friction, scoring and demo vision for processor-blade-walnuts. No Isaac Sim needed.

    pytest tasks/processor-blade-walnuts/test_processor_blade_walnuts.py     (in the yam_arena environment)
"""

import math
import pathlib
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import bladedemo  # noqa: E402
from bladekit import PartState, matrix_to_quat, rot_x  # noqa: E402
from yam_arena.task_file import load  # noqa: E402

T = load(HERE / "task.py").module


def state(pos, R=np.eye(3), v=0.0) -> PartState:
    pos = np.asarray(pos, float)
    return PartState(pos=pos, quat_xyzw=np.array(matrix_to_quat(R)), lin_vel=np.array([v, 0, 0]), ang_vel=np.zeros(3),
                     low=pos - 0.05, high=pos + 0.05, R=R)


def scene(n_in_mix: int, blade=None, bowl=None) -> dict:
    """Bowl back on its place; ``n_in_mix`` nuts resting in the mixing bowl, the rest in the work bowl."""
    out = {"bowl": bowl or state((*T.BOWL_XY, T.TOP)), "mixing_bowl": state((*T.MIX_XY, T.TOP)),
           "board": state((*T.BOARD_XY, T.TOP)), "blade": blade or state((*T.BOWL_XY, T.TOP))}
    for i, n in enumerate(T.NUTS):
        where = (T.MIX_XY if i < n_in_mix else T.BOWL_XY)
        out[n.name] = state((where[0] + 0.01 * (i % 5 - 2), where[1] + 0.01 * (i // 5 % 4 - 1.5), T.TOP + 0.02))
    return out


def blade_on_board(tilt_deg=0.0, v=0.0):
    p = np.array([*T.BOARD_XY, T.TOP + T.BOARD_SIZE[2] - T.HUB_Z0])
    return state(p, rot_x(math.radians(tilt_deg)), v)


@pytest.fixture(autouse=True)
def fresh():
    T._reset_state()
    yield
    T._reset_state()


# -- the hub on the post ---------------------------------------------------------------------------------------------


def test_pull_more_than_8_degrees_off_the_axis_wedges():
    assert 7.5 < T.jam_angle_deg() < 8.5
    for deg, slides in ((5, True), (7, True), (9, False), (20, False)):
        a = math.radians(deg)
        for f in (2.0, 10.0, 40.0):  # any pull force: the wedge grows with the pull
            side, moment = f * math.sin(a), f * math.sin(a) * T.STEM_LEVER
            assert (f * math.cos(a) > T.breakaway(side, moment)) == slides, (deg, f)


def test_tipped_bowl_lets_the_blade_slide_out_between_92_and_100_degrees():
    m, g = T.BLADE_MASS, 9.81
    out = [phi for phi in range(80, 135) if -m * g * math.cos(math.radians(phi))
           > T.breakaway(m * g * math.sin(math.radians(phi)), 0.0)]
    assert 92 <= out[0] <= 100


def test_breakaway_spring_drags_behind_the_blade():
    k, lim = 2000.0, 1.0
    assert T.drag(0.0, 0.0, lim, k) == 0.0  # small motion: held
    assert T.drag(0.01, 0.0, lim, k) == pytest.approx(0.01 - lim / k)  # pulled past: follows, lim/k behind
    assert T.drag(0.01, 0.02, lim, k) == pytest.approx(0.01 + lim / k)


def test_relative_pose_of_blade_on_post():
    Rb = rot_x(0.3)
    pb = np.array([0.1, 0.2, 0.3])
    pl = pb + Rb @ np.array([0.001, 0.0, 0.03])
    travel, off, tilt = T.relative(Rb, pb, Rb, pl)
    assert travel == pytest.approx(0.03) and off == pytest.approx(0.001) and tilt == pytest.approx(0.0, abs=1e-7)


def test_held_needs_both_pads_of_one_hand():
    assert T.held_by({"left_robot": [1.0, 0.5], "right_robot": [0.0, 0.0]}) == "left_robot"
    assert T.held_by({"left_robot": [1.0, 0.0], "right_robot": [0.0, 3.0]}) is None


# -- scoring ---------------------------------------------------------------------------------------------------------


def test_full_transfer_with_blade_on_the_board_succeeds():
    T._S["attached"] = False
    s = scene(len(T.NUTS), blade_on_board())
    assert T.score(s) == 1.0 and T.success(s)


def test_second_method_blade_still_on_the_post_counts():
    s = scene(len(T.NUTS))  # held the hub down while pouring: still attached
    assert T.score(s) == 1.0 and T.success(s)


def test_blade_riding_up_the_post_is_not_stowed():
    T._S["travel"] = 0.04  # tipped without a hand on the hub: the blade is sliding off with the nuts
    s = scene(len(T.NUTS))
    assert T.score(s) == pytest.approx(0.8) and not T.success(s)


def test_partial_transfer_is_proportional_and_90_percent_is_full():
    T._S["attached"] = False
    n = len(T.NUTS)
    s = scene(n // 2, blade_on_board())
    frac = sum(x.mass for x in T.NUTS[: n // 2]) / sum(x.mass for x in T.NUTS)
    assert T.score(s) == pytest.approx(frac / 0.9)
    assert T.score(scene(math.ceil(0.92 * n), blade_on_board())) > 0.99


def test_blade_left_elsewhere_or_held_caps_at_0_8():
    T._S["attached"] = False
    n = len(T.NUTS)
    assert T.score(scene(n, state((0.3, 0.3, T.TOP + 0.04)))) == pytest.approx(0.8)  # on the bare table
    assert T.score(scene(n, blade_on_board(tilt_deg=30))) == pytest.approx(0.8)
    T._S["held"] = "left_robot"
    assert T.score(scene(n, blade_on_board())) == pytest.approx(0.8)  # still in the hand
    assert not T.success(scene(n, blade_on_board()))


@pytest.mark.parametrize("latch", ["released_unheld", "riding", "in_mix", "on_floor", "edge_up"])
def test_every_blade_latch_zeroes_the_trial_for_good(latch):
    T._S["attached"] = False
    T._S[latch] = True
    s = scene(len(T.NUTS), blade_on_board())  # even if it ends perfectly
    assert T.score(s) == 0.0 and not T.success(s)


def test_nuts_still_inside_the_work_bowl_do_not_count_even_over_the_mixing_bowl():
    T._S["attached"] = False
    bowl = state((*T.MIX_XY, T.TOP + 0.01))  # the whole work bowl set into the mixing bowl
    s = scene(0, blade_on_board(), bowl=bowl)
    for n in T.NUTS:
        s[n.name] = state((T.MIX_XY[0], T.MIX_XY[1], T.TOP + 0.03))
    assert T.score(s) == 0.0


def test_location_checks():
    assert T.in_mixing_bowl((T.MIX_XY[0] + 0.05, T.MIX_XY[1], T.TOP + 0.02))
    assert not T.in_mixing_bowl((T.MIX_XY[0] + 0.13, T.MIX_XY[1], T.TOP + 0.02))
    assert not T.in_mixing_bowl((T.MIX_XY[0], T.MIX_XY[1], T.TOP + 0.2))


# -- demo vision -----------------------------------------------------------------------------------------------------


def test_demo_finds_blobs_where_they_are():
    cam = bladedemo.TopCamera(320, 176)
    img = np.full((176, 320, 3), 150, np.uint8)
    truth = {"stem": (np.array([*T.BOWL_XY, T.TOP + bladedemo.STEM_BLOB_Z]), (20, 20, 22)),
             "mix": (np.array([*T.MIX_XY, T.TOP + T.MIX_H / 2]), (40, 80, 180)),
             "board": (np.array([*T.BOARD_XY, T.TOP + T.BOARD_SIZE[2]]), (60, 150, 75))}
    for p, color in truth.values():
        u, v = cam.project(p)
        img[int(round(v)) - 2:int(round(v)) + 3, int(round(u)) - 2:int(round(u)) + 3] = color
    u, v = (int(round(c)) for c in cam.project(truth["stem"][0]))
    ring = img[v - 9:v + 10, u - 9:u + 10]
    ring[...] = (180, 214, 240)  # the clear bowl around the stem
    img[v - 2:v + 3, u - 2:u + 3] = truth["stem"][1]
    du, dv = (int(round(c)) for c in cam.project((0.15, -0.25, T.TOP + 0.15)))
    img[dv - 4:dv + 5, du - 4:du + 5] = (20, 20, 22)  # a parked arm's black link, away from the bowl
    heights = {k: p[2] for k, (p, _) in truth.items()}
    got = bladedemo.find(cam, img, heights)
    for k, (p, _) in truth.items():
        assert np.linalg.norm(got[k][:2] - p[:2]) < 0.01, k
