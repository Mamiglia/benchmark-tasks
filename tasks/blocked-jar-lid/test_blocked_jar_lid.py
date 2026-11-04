"""Scoring tests on handcrafted states, the thread model, and the demos' reachability. No Isaac Sim needed:

    pytest tasks/blocked-jar-lid      (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan, Waypoint
from yam_arena.task_file import load

TASK = load(Path(__file__).resolve().parent / "task.py")
T = TASK.module
from jointkit import PartState  # noqa: E402  (task.py put this folder on sys.path)

UP = np.array([0.0, 0.0, 0.0, 1.0])
JAR0 = np.array(T.JAR.pos)
SEAT = JAR0 + (0, 0, T.RIM)


def quat(axis, angle):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    return np.r_[a * math.sin(angle / 2), math.cos(angle / 2)]


def part(spec, pos, q=UP):
    pos, q = np.asarray(pos, float), np.asarray(q, float)
    R = PartState(pos, q, np.zeros(3), np.zeros(3), pos, pos).rotation()
    corners = np.array([[a, b, c] for a in (spec.lo[0], spec.hi[0]) for b in (spec.lo[1], spec.hi[1])
                        for c in (spec.lo[2], spec.hi[2])]) @ R.T + pos
    return PartState(pos, q, np.zeros(3), np.zeros(3), corners.min(0), corners.max(0))


def scene(h=T.H0, block=None, jar=JAR0, jar_q=UP, lid=None):
    """Lid ``h`` above its seat (or at ``lid``), block standing in the jar (or at ``block``)."""
    return {"jar": part(T.JAR, jar, jar_q), "lid": part(T.LID, lid if lid is not None else SEAT + (0, 0, h)),
            "block": part(T.BLOCK, block if block is not None else T.BLOCK.pos)}


ON_TABLE = (0.15, 0.10, T.TOP)  # where the demo's block lands, roughly


@pytest.fixture(autouse=True)
def fresh():
    T._reset_state()


def taken_off():
    T._STATE.update(engaged=False, lid_off=True)


def back_on():
    T._STATE.update(engaged=True)


def test_the_task_loads_with_its_objects_and_hooks():
    assert TASK.scene == "station" and TASK.score_over == "end"
    assert TASK.names == ("jar", "lid", "block", "tray") == tuple(sorted(TASK.free_objects, key=TASK.names.index))
    # the tray (optional) is clear of the jar's spot and of where the lid is set down
    half = np.array([T.T_IN_X, T.T_IN_Y]) / 2 + T.T_WALL
    for spot, r in (((T.JX, T.JY), T.J_OUT), (T.LID_AT, T.L_OUT)):
        assert np.any(np.abs(np.subtract(spot, (T.TX, T.TY))) > half + r + 0.02)
    assert all(callable(f) for f in (TASK.success, TASK.score, TASK.demo))
    assert "block" not in T.INSTRUCTION and "obstruct" not in T.INSTRUCTION


def test_the_block_holds_the_lid_one_turn_short_and_cannot_lie_low_enough_inside():
    assert T.J_FLOOR + T.BLOCK_H == pytest.approx(T.RIM + T.H0)  # its top is where the lid stops: 4 mm up
    assert T.H0 == pytest.approx(T.PITCH)  # one turn left of two
    # tilted until its corners touch opposite walls, the block's top is still above the rim
    w, length, inner = T.BLOCK_W, T.BLOCK_H, 2 * T.J_IN
    tilt = next(a for a in np.linspace(0, 1.5, 3000) if length * math.sin(a) + w * math.cos(a) >= inner)
    assert T.J_FLOOR + length * math.cos(tilt) + w * math.sin(tilt) > T.RIM
    # a lid set on the rim rests on it (skirt over the glass), at the thread's start
    assert T.L_IN < T.J_OUT < T.L_OUT


def test_start_and_cheats_score_nothing():
    assert T.score(scene()) == 0.0 and not T.success(scene())  # left as it is
    assert T.score(scene(h=0.0)) == 0.0  # seated without ever taking the lid off is not possible, scores 0 anyway
    T.THREAD.damaged = True  # forced
    taken_off()
    assert T.score(scene(block=ON_TABLE, lid=(0.2, -0.15, T.TOP))) == 0.0


def test_partial_credit_and_success():
    taken_off()
    aside = (0.20, -0.15, T.TOP + T.TRAVEL)
    assert T.score(scene(lid=aside)) == 0.25
    assert T.score(scene(lid=aside, block=ON_TABLE)) == 0.5
    assert T.score(scene(h=T.TRAVEL, block=ON_TABLE)) == 0.5  # resting on the rim, not threaded
    back_on()
    assert T.score(scene(h=T.TRAVEL, block=ON_TABLE)) == 0.5  # caught but not yet half a turn down
    assert T.score(scene(h=0.006, block=ON_TABLE)) == 0.75
    assert T.score(scene(h=0.002, block=ON_TABLE)) == 0.75
    done = scene(h=0.0005, block=ON_TABLE)
    assert T.score(done) == 1.0 and T.success(done)


def test_a_block_held_above_the_jar_counts_as_out():
    jar = part(T.JAR, JAR0)
    assert not T.block_out(jar, part(T.BLOCK, T.BLOCK.pos))
    assert T.block_out(jar, part(T.BLOCK, JAR0 + (0, 0, T.RIM + 0.01)))
    assert T.block_out(jar, part(T.BLOCK, ON_TABLE))


def test_knocking_the_jar_over_scores_zero_for_good_but_tipping_it_in_the_air_does_not():
    taken_off()
    lying = scene(block=ON_TABLE, jar=JAR0 + (0, 0, T.J_OUT), jar_q=quat((0, 1, 0), math.pi / 2),
                  lid=(0.2, -0.15, T.TOP))
    assert T.score(lying) == 0.0
    assert T.score(scene(h=0.0, block=ON_TABLE)) == 0.0  # stays 0 once knocked
    T._reset_state()
    taken_off()
    lifted = scene(jar=JAR0 + (0, 0, 0.2), jar_q=quat((0, 1, 0), 2.2), lid=(0.2, -0.15, T.TOP), block=ON_TABLE)
    assert T.score(lifted) == 0.5
    T._reset_state()
    taken_off()
    assert T.score(scene(jar=JAR0 - (0, 0, 0.3), lid=(0.2, -0.15, T.TOP))) == 0.0  # fell off the table


# -- the thread ---------------------------------------------------------------------------------------------------

N0 = 2 * math.pi * (T.TURNS - T.H0 / T.PITCH)


def test_breakaway_friction_holds_then_slips_at_the_running_torque():
    th = T.Thread()
    target, _ = th.step(N0 + math.radians(1), T.H0 + 0.0002)  # unscrewing direction is -, try a small + nudge
    assert target == pytest.approx(N0)
    target, _ = th.step(N0 - math.radians(20), T.H0)
    assert T.K_ROT * (target - (N0 - math.radians(20))) == pytest.approx(T.FRICTION)


def test_the_slide_target_follows_the_thread():
    assert T.thread_height(0.0) == pytest.approx(T.TRAVEL)
    assert T.thread_height(2 * math.pi) == pytest.approx(T.TRAVEL - T.PITCH)
    assert T.thread_height(4 * math.pi) == 0.0 and T.thread_height(9.0 * math.pi) == 0.0
    assert T.thread_height(-1.0) > T.TRAVEL  # unscrewed past the catch, the skirt clears the rim


def test_turning_a_blocked_lid_on_pushes_back_and_breaks_past_the_limit():
    th = T.Thread()
    give = T.SLACK + T.BREAK_NM / T.K_ROT
    for _ in range(T.SETTLE):  # a jolt while the lid settles after a catch (or at the start) does not count
        th.step(N0 + give + math.radians(5), T.H0)
    assert not th.damaged
    for k in range(10):  # just under the limit: never damaged
        target, _ = th.step(N0 + give - math.radians(1), T.H0)
    assert not th.damaged and T.K_ROT * (N0 + give - math.radians(1) - target) < T.BREAK_NM
    for k in range(T.BREAK_STEPS - 1):
        th.step(N0 + give + math.radians(1), T.H0)
    assert not th.damaged  # two physics steps over the limit
    th.step(N0 + give + math.radians(1), T.H0)
    assert th.damaged and th.peak_nm > T.BREAK_NM


def test_tightening_a_seated_lid_hard_is_not_damage():
    th = T.Thread()
    th.reset(2 * math.pi * T.TURNS)
    for _ in range(20):
        th.step(2 * math.pi * T.TURNS + math.radians(90), 0.0)
    assert not th.damaged


def test_a_free_lid_turns_on_with_friction_only():
    th = T.Thread()
    th.reset(0.0)
    for k in range(1, 400):
        n = k * math.radians(1.5)
        target, height = th.step(n, T.thread_height(n))
        assert abs(T.K_ROT * (target - n)) <= T.FRICTION + 1e-9
    assert not th.damaged


def test_the_lid_catches_only_when_set_square_on_the_rim():
    jar = part(T.JAR, JAR0)
    assert T.can_catch(jar, part(T.LID, SEAT + (0, 0, T.TRAVEL)))
    assert not T.can_catch(jar, part(T.LID, SEAT + (0.008, 0, T.TRAVEL)))  # 8 mm off centre
    assert not T.can_catch(jar, part(T.LID, SEAT + (0, 0, T.TRAVEL + 0.01)))  # 1 cm above the rim
    assert not T.can_catch(jar, part(T.LID, SEAT + (0, 0, T.TRAVEL), quat((1, 0, 0), math.radians(15))))


def test_twist_is_the_lids_turn_relative_to_the_jar():
    jar = part(T.JAR, JAR0, quat((0, 0, 1), 0.3))
    lid = part(T.LID, SEAT, quat((0, 0, 1), 1.3))
    assert T._twist(jar, lid) == pytest.approx(1.0)


# -- the demos ----------------------------------------------------------------------------------------------------


@pytest.mark.parametrize("route", ["demo", "force"])
def test_each_demo_is_reachable_within_the_joint_limits_and_run_limit(route):
    for arm, wps in T.plan(route).items():
        arm_plan = ArmPlan([Waypoint(d, tuple(p), f, label, r) for d, p, r, f, label in wps], station.BASES[arm])
        steps = round(arm_plan.duration * 20) + 5
        actions = np.array([arm_plan.action7((k + 1) / 20)[0] for k in range(steps)])
        assert arm_plan.worst_ik_mm < 1.0 and arm_plan.worst_ik_deg < 0.5, (arm, arm_plan.worst_ik_mm)
        assert np.all(actions[:, :6] >= ik.LOWER) and np.all(actions[:, :6] <= ik.UPPER)
        assert arm_plan.duration * 20 < 2400


def test_the_demo_turns_far_enough():
    turns = [w for w in T.plan("demo")["right"] if w[4] == "turn"]
    assert len(turns) // 2 == 3 + 4  # three strokes off, four on
    assert 3 * math.pi > T.H0 / T.PITCH * 2 * math.pi + T.RELEASE  # 540 degrees off > 370 needed
    assert 4 * math.pi + math.radians(20) > 2 * math.pi * T.TURNS  # 740 degrees on >= 720 needed
