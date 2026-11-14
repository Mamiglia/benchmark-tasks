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

from yam_arena.task_file import ObjectState, load  # noqa: E402

M = load(HERE / "task.py").module
T = M.T


def st(pos, yaw=0.0, tilt_x=0.0, v=0.0):
    """An object state: turned ``yaw`` about z, then tipped ``tilt_x`` degrees about x."""
    a, b = yaw / 2, math.radians(tilt_x) / 2
    qz = np.array([0, 0, math.sin(a), math.cos(a)])
    qx = np.array([math.sin(b), 0, 0, math.cos(b)])
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    q = np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                  w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=q, lin_vel=np.array([v, 0, 0]), ang_vel=np.zeros(3), low=p, high=p)


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
    guides = [o for o in M.OBJECTS if o.name.startswith("guide_")]
    gap = min(guides, key=lambda g: g.pos[0]).pos[0] + M.RAIL_W / 2
    assert math.isclose(M.BOX_XY[0] - M.BOX_X / 2 - gap, 0.002, abs_tol=1e-9)
    assert pts[:, 2].max() < T + M.BOX_H  # the shut lid clears standing glasses
