"""Scoring and latches for glasses-sliding-box. No Isaac Sim needed.

    pytest tasks/glasses-sliding-box/test_glasses_sliding_box.py     (in the yam_arena environment)
"""

import math
import pathlib
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from glasskit import ContactState  # noqa: E402
from yam_arena.task_file import load  # noqa: E402

M = load(HERE / "task.py").module
T = M.T


WEIGHT = M.GLASSES_MASS * 9.81


def st(pos, yaw=0.0, tilt_x=0.0, v=0.0, robot_n=0.0, support_n=WEIGHT, axis=0):
    """An object state: turned ``yaw`` about z, then tipped ``tilt_x`` degrees about its own x (``axis`` 1: y); by default resting on a surface
    (carrying its weight, as the glasses' would be) and not touched by a robot."""
    a, b = yaw / 2, math.radians(tilt_x) / 2
    qz = np.array([0, 0, math.sin(a), math.cos(a)])
    qx = np.array([0.0, 0.0, 0.0, math.cos(b)])
    qx[axis] = math.sin(b)
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    q = np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                  w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
    p = np.asarray(pos, float)
    return ContactState(pos=p, quat_xyzw=q, lin_vel=np.array([v, 0, 0]), ang_vel=np.zeros(3), low=p, high=p,
                          robot_n=robot_n, support_n=support_n)


START = dict(glasses=st((*M.GLASSES_START, T), math.pi), keys=st((*M.KEYS_START, T + 0.085)),
             lid=st(M.START["lid"]), mug=st(M.START["mug"]))
IN_BOX = st((M.BOX_XY[0], M.INNER_Y[1] - 0.025, T + M.FLOOR), math.pi)
IN_DISH = st((*M.DISH_XY, T + M.DISH_BASE))
LID_OPEN = st((M.BOX_XY[0], M.BOX_XY[1] - 0.195, M.LID_Z))


def run(*frames):
    """Score a trial: each frame overrides some objects of the previous one; returns the last score."""
    M.clear_latches()
    o = dict(START)
    s = None
    for f in frames:
        o.update(f)
        s = M.score(o)
    return s


def test_start_and_full_success():
    assert run({}) == 0.0
    assert run({"keys": IN_DISH}, {"lid": LID_OPEN}, {"glasses": IN_BOX}, {"lid": START["lid"]}) == 1.0


def test_partial_credit():
    assert run({"keys": IN_DISH}) == 0.2
    assert run({"keys": IN_DISH}, {"lid": LID_OPEN}, {"glasses": IN_BOX}) == 0.6
    # keys left on the lid, which then rides open and shut with them
    k = M.KEYS_START
    assert run({"lid": LID_OPEN, "keys": st((k[0], k[1] - 0.195, T + 0.085))}, {"glasses": IN_BOX},
               {"lid": START["lid"], "keys": START["keys"]}) == 0.8
    # a shut lid earns nothing without the glasses under it
    assert run({"keys": IN_DISH}, {"lid": LID_OPEN}, {"lid": START["lid"]}) == 0.2


def test_glasses_upside_down_count_and_carrying_sideways_is_not_a_fall():
    upside_down = st((M.BOX_XY[0], -0.035, T + M.FLOOR + M.GLASSES_TOP), math.pi, tilt_x=180)  # arms now point +y
    assert run({"keys": IN_DISH}, {"lid": LID_OPEN}, {"glasses": upside_down}, {"lid": START["lid"]}) == 1.0
    turning = st((0.2, 0.2, T + 0.15), math.pi, tilt_x=90, v=0.1, robot_n=0.4, support_n=0.0)  # carried sideways
    assert run({"keys": IN_DISH}, {"lid": LID_OPEN}, {"glasses": turning}, {"glasses": IN_BOX},
               {"lid": START["lid"]}) == 1.0


@pytest.mark.parametrize("tilt_x,pos", [
    (90, (*M.GLASSES_START, T + 0.07)),  # on the lenses, on the table
    (-90, (*M.GLASSES_START, T + 0.07)),  # the other way over
    (90, (M.BOX_XY[0], 0.04, T + M.FLOOR + 0.07)),  # on the lenses, on the box floor
])
@pytest.mark.parametrize("axis", [0, 1])  # tipped about the glasses' x (onto the lenses) or y (onto a side)
def test_resting_on_lenses_or_side_latches(tilt_x, pos, axis):
    # still, carried by a surface (the default state), nobody holding them: a fall, even if then put right
    lying = st(pos, math.pi, tilt_x=tilt_x, axis=axis)
    assert run({"keys": IN_DISH}, {"glasses": lying}, {"lid": LID_OPEN}, {"glasses": IN_BOX},
               {"lid": START["lid"]}) == 0.0
    assert M.LATCH == {"glasses_on_lenses_or_side": True}


def test_sideways_pause_while_held_is_not_a_fall():
    for held in (st((0.2, 0.2, T + 0.15), math.pi, tilt_x=90, robot_n=0.4, support_n=0.0),  # paused in the air
                 st((0.2, 0.2, T + 0.15), math.pi, tilt_x=90, robot_n=0.4, support_n=-0.1),
                 st((*M.GLASSES_START, T + 0.07), math.pi, tilt_x=90, robot_n=0.3, support_n=0.1)):  # held, grazing
        frames = [{"keys": IN_DISH}, {"lid": LID_OPEN}] + [{"glasses": held}] * 40 + [{"glasses": IN_BOX},
                                                                                       {"lid": START["lid"]}]
        assert run(*frames) == 1.0, held
    # sideways and still with nothing holding or carrying them (a state between frames): not a fall either
    assert run({"keys": IN_DISH}, {"lid": LID_OPEN}, {"glasses": st((0.2, 0.2, T + 0.15), math.pi, tilt_x=90,
               support_n=0.0)}, {"glasses": IN_BOX}, {"lid": START["lid"]}) == 1.0
    # the same grip let go on the table: the moment the table carries them and the fingers are off, it latches
    assert run({"keys": IN_DISH}, {"glasses": st((*M.GLASSES_START, T + 0.07), math.pi, tilt_x=90, robot_n=0.3,
               support_n=0.1)}, {"glasses": st((*M.GLASSES_START, T + 0.07), math.pi, tilt_x=90)}) == 0.0


def test_glasses_must_stand_still_on_floor():
    lying = st((M.BOX_XY[0], 0.05, T + M.FLOOR), math.pi, tilt_x=30)
    assert run({"keys": IN_DISH}, {"lid": LID_OPEN}, {"glasses": lying}) == 0.2
    moving = st(IN_BOX.pos, math.pi, v=0.1)
    assert run({"keys": IN_DISH}, {"lid": LID_OPEN}, {"glasses": moving}) == 0.2
    held_high = st(IN_BOX.pos + [0, 0, 0.03], math.pi)
    assert run({"keys": IN_DISH}, {"lid": LID_OPEN}, {"glasses": held_high}) == 0.2
    overhanging = st((M.INNER_X[0] + 0.03, IN_BOX.pos[1], IN_BOX.pos[2]), math.pi)
    assert run({"keys": IN_DISH}, {"lid": LID_OPEN}, {"glasses": overhanging}) == 0.2


@pytest.mark.parametrize("bad", [
    {"glasses": st((*M.GLASSES_START, T), math.pi, tilt_x=80)},  # knocked onto their lenses
    {"glasses": st((0.2, 0.3, 0.3))},  # on the floor
    {"keys": st((M.BOX_XY[0], 0.05, T + M.FLOOR))},  # keys dropped into the open box
    {"keys": st((0.3, 0.6, 0.2))},  # keys on the floor
    {"lid": st(M.START["lid"] + [0, 0, 0.05])},  # lid lifted off its guides
    {"lid": st(M.START["lid"], tilt_x=20)},
    {"mug": st(M.START["mug"], tilt_x=40)},
    {"mug": st(M.START["mug"] + [0.03, 0, 0])},
])
def test_violations_latch_for_the_whole_trial(bad):
    # a violation, then everything put right: still 0
    assert run({"keys": IN_DISH}, bad, dict(START, keys=IN_DISH), {"lid": LID_OPEN}, {"glasses": IN_BOX},
               {"lid": START["lid"]}) == 0.0


def test_reset_clears_latches():
    run({"mug": st(M.START["mug"], tilt_x=40)})
    assert M.LATCH
    for fn in M.ON_RESET:
        fn()
    assert not M.LATCH


def test_layout_fits():
    # the glasses fit inside the box's inner walls as placed, and the lid clears the guides by 2 mm a side
    pts = M.corners(M.GLASSES, IN_BOX.pos, IN_BOX.quat_xyzw)
    assert all(M.inside_box(p) for p in pts)
    near_guide = M.BOX.parts[-2]
    gap = M.BOX_XY[0] + near_guide[0][0] + M.RAIL_W / 2
    assert math.isclose(M.BOX_XY[0] - M.BOX_X / 2 - gap, 0.002, abs_tol=1e-9)
    assert pts[:, 2].max() < T + M.BOX_H  # the shut lid clears standing glasses
