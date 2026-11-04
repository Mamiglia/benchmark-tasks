"""Scoring tests on handcrafted states, the geometry the task relies on, and the demo's reachability. No Isaac Sim
needed:

    pytest tasks/hidden-ball-peg      (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.task_file import load

TASK = load(Path(__file__).resolve().parent / "task.py")
T = TASK.module
from pegkit import PartState  # noqa: E402  (task.py put this folder on sys.path)

UP = np.array([0.0, 0.0, 0.0, 1.0])
RZ = T.rot_z(T.YAW)  # block and peg frames: the bore runs along world y, mouth toward -y
LEFT_BASE, RIGHT_BASE = (0.0, 0.305, 0.75), (0.0, -0.305, 0.75)


def st(pos, R=None):
    pos = np.asarray(pos, float)
    return PartState(pos, UP, np.zeros(3), np.zeros(3), pos - 0.05, pos + 0.05, R=RZ if R is None else R)


def scene(depth=T.START_IN, ball=None, block=None):
    """Block at ``block`` (default its place), peg with ``depth`` of shaft in the bore, ball at ``ball``."""
    blk = st(block or T.BLOCK.pos)
    return {"block": blk, "peg": st(blk.to_world((T.MOUTH + depth, 0.0, T.ZC))), "ball": st(ball or T.BALL.pos)}


OUT = (T.TX, T.TY, T.TOP + 0.003 + T.R)  # the ball in the tray


@pytest.fixture(autouse=True)
def fresh():
    T._reset_state()


def test_the_task_loads_with_its_objects_and_hooks():
    assert TASK.scene == "station" and TASK.score_over == "end"
    assert TASK.names == ("block", "peg", "ball", "tray") and set(TASK.free_objects) == set(TASK.names)
    assert all(callable(f) for f in (TASK.success, TASK.score, TASK.demo))


def test_the_start_matches_the_world_layout():
    s = scene()
    assert s["peg"].pos == pytest.approx(T.PEG.pos) and T.ball_out(s["block"], s["ball"]) is False
    assert s["block"].to_world((T.MOUTH, 0, 0))[1] == pytest.approx(T.BY - T.BL / 2)  # mouth toward -y


def test_the_tray_is_clear_of_the_block_and_under_the_tipped_mouth():
    assert T.TY - T.T_HALF > T.BY + T.BL / 2 + 0.01
    g = RZ.T @ (np.array(T.GRIP_BLOCK) - np.array(T.BLOCK.pos))  # the grip in the block frame
    for deg in (10, 20, 30):
        r = T.rot_x(math.radians(deg)) @ RZ
        mouth = np.array(T.TIP_AT) + r @ (np.array([T.MOUTH, 0.0, T.ZC]) - g)
        corners = [np.array(T.TIP_AT) + r @ (np.array([sx * T.BL / 2, 0.0, 0.0]) - g) for sx in (1, -1)]
        assert abs(mouth[1] - T.TY) < T.T_HALF - 0.02 and abs(mouth[0] - T.TX) < 0.01
        assert min(c[2] for c in corners) - T.TOP > T.T_H + 0.005  # the block clears the rims


def test_the_geometry_hides_the_ball_and_blocks_the_peg():
    assert 2 * T.R < T.S and T.P < T.S < T.HEAD          # the ball and shaft fit the bore; the head covers the mouth
    assert T.END - T.MOUTH - T.FULL_IN < 2 * T.R           # with the ball in, the peg cannot go fully in
    assert T.FULL_IN - T.BLOCKED_IN == pytest.approx(0.015)  # it stops 1.5 cm short
    assert T.START_IN < T.BLOCKED_IN                       # the start leaves 3 cm of free travel before the stop
    assert T.BW < 0.094 and T.HEAD < 0.094                 # both fit the gripper's 9.4 cm opening
    ball = scene()["ball"]
    assert not T.ball_out(scene()["block"], ball)


def test_the_start_and_a_peg_pushed_to_the_ball_score_nothing():
    assert T.score(scene()) == 0.0
    assert T.score(scene(depth=T.BLOCKED_IN)) == 0.0 and not T.success(scene(depth=T.BLOCKED_IN))


def test_partial_credit_for_the_ball_out():
    assert T.score(scene(depth=-0.05, ball=OUT)) == 0.25        # peg out of the bore
    assert T.score(scene(depth=0.02, ball=OUT)) == 0.25
    assert T.score(scene(depth=T.PART_IN + 0.001, ball=OUT)) == 0.5
    assert T.score(scene(depth=T.FULL_IN - 0.004, ball=OUT)) == 0.5


def test_full_score_needs_the_ball_out_and_the_head_within_3_mm():
    s = scene(depth=T.FULL_IN - 0.002, ball=OUT)
    assert T.score(s) == 1.0 and T.success(s)
    moved = scene(depth=T.FULL_IN, ball=OUT, block=(0.25, 0.10, T.TOP))  # the block's place doesn't matter
    assert T.score(moved) == 1.0


def test_the_peg_must_be_in_the_bore_not_just_near_the_block():
    s = scene(depth=T.FULL_IN, ball=OUT)
    s["peg"] = st(s["peg"].pos + np.array([0.03, 0.0, 0.0]))  # beside the bore
    assert T.score(s) == 0.25


def test_the_peg_counts_in_a_tipped_block():
    blk = st(T.BLOCK.pos, T.rot_x(math.radians(40)) @ RZ)  # block tilted 40 deg about x
    tip = blk.to_world((T.MOUTH + T.FULL_IN, 0.0, T.ZC))
    s = {"block": blk, "peg": st(tip), "ball": st(OUT)}
    assert T.score(s) == 1.0


def test_damage_or_anything_off_the_table_scores_zero_for_good():
    done = scene(depth=T.FULL_IN, ball=OUT)
    T._STATE["damaged"] = True
    assert T.score(done) == 0.0
    T._reset_state()
    assert T.score(scene(depth=T.FULL_IN, ball=(0.3, 0.1, 0.0))) == 0.0     # ball on the floor
    T._reset_state()
    assert T.score(scene(depth=-0.05, ball=OUT, block=(0.3, 0.1, 0.2))) == 0.0  # block on the floor
    assert T.score(done) == 0.0                                               # latched


def test_the_force_latch_needs_3_steps_in_a_row():
    hi, lo = T.FORCE_LIMIT + 0.5, T.FORCE_LIMIT - 0.5
    seq = iter([hi, hi, lo, hi, hi, hi])
    T.contact_force = lambda a, b: next(seq)
    try:
        for i in range(6):
            T._force_check(None)
            assert T._STATE["damaged"] == (i == 5)
    finally:
        import pegkit
        T.contact_force = pegkit.contact_force


@pytest.mark.parametrize("force", [False, True])
def test_the_demo_is_reachable_and_inside_the_run_limit(force, monkeypatch):
    if force:
        monkeypatch.setenv("DEMO_FORCE", "1")
    plan = TASK.demo()
    for arm, base in (("left", LEFT_BASE), ("right", RIGHT_BASE)):
        p = ArmPlan(plan[arm], base)
        for t in np.arange(0.0, p.duration, 0.05):
            q, _ = p.action7(t)
            assert np.all(np.abs(q[5]) < math.radians(120))
        assert p.worst_ik_mm < 2.0 and p.worst_ik_deg < 2.0, (arm, p.worst_ik_mm, p.worst_ik_deg)
        assert p.duration * 20 < 1200
