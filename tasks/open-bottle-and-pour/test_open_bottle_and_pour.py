"""Scoring tests on handcrafted states, for both opening routes, and the demos' reachability. No Isaac Sim needed:

    pytest tasks/open-bottle-and-pour      (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.demo import ArmPlan, Waypoint
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).resolve().parent / "task.py")
T = TASK.module
from jointkit import PartState  # noqa: E402  (task.py put this folder on sys.path)

UP = np.array([0.0, 0.0, 0.0, 1.0])


def quat(axis, angle):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    return np.r_[a * math.sin(angle / 2), math.cos(angle / 2)]


def part(pos, q=UP, hinge=0.0, vel=(0, 0, 0)):
    pos = np.asarray(pos, float)
    q, vel = np.asarray(q, float), np.asarray(vel, float)
    return PartState(pos, q, vel, np.zeros(3), pos - 0.05, pos + 0.05, hinge=hinge)


def bead(pos, vel=(0, 0, 0)):
    pos = np.asarray(pos, float)
    return ObjectState(pos, UP, np.asarray(vel, float), np.zeros(3), pos - T.BEAD_R, pos + T.BEAD_R)


def scene(in_cup=0, lid=0.0, cap=None, cup=None, cup_q=UP, bottle=None, moving=False):
    """The starting layout, with ``in_cup`` beads lying on the cup's floor and the rest in the bottle."""
    bottle = bottle or T.BOTTLE.pos
    s = {"bottle": part(bottle), "cup": part(cup or T.CUP.pos, cup_q),
         "cap": part(cap or (bottle[0] + T.NECK_X, bottle[1], bottle[2] + T.NECK_TOP), quat((0, 0, 1), T.CAP_YAW),
                     hinge=lid)}
    vel = (0.0, 0.0, -0.3) if moving else (0.0, 0.0, 0.0)
    for i, b in enumerate(T.BEADS):
        if i < in_cup:
            cx, cy, cz = s["cup"].pos
            s[b.name] = bead((cx + (i % 6 - 2.5) * 0.01, cy + (i // 6 % 6 - 2.5) * 0.01, cz + 0.01), vel)
        else:
            s[b.name] = bead(b.pos)
    return s


@pytest.fixture(autouse=True)
def fresh():
    T._reset_state()


def test_the_task_loads_with_its_objects_and_hooks():
    assert TASK.scene == "station" and TASK.score_over == "end"
    assert TASK.names[:3] == ("bottle", "cap", "cup") and len(TASK.names) == 3 + 40
    assert set(TASK.free_objects) == set(TASK.names)
    assert all(callable(f) for f in (TASK.success, TASK.score, TASK.demo))
    assert TASK.docs.startswith("On the station table")


def test_the_layout_is_within_reach_and_the_beads_start_inside_the_closed_bottle():
    for o in (T.BOTTLE, T.CUP):
        assert 0.10 <= o.pos[0] <= 0.40 and abs(o.pos[1]) < 0.305
    inner = T.B_HALF - T.B_WALL
    for b in T.BEADS:
        x, y, z = np.subtract(b.pos, T.BOTTLE.pos)
        assert abs(x) + b.radius < inner and abs(y) + b.radius < inner and z - b.radius >= T.B_WALL
    assert 2 * T.BEAD_R < 2 * T.ORIFICE <= 2 * T.NECK_HOLE  # beads pass both openings; 6 < 20 <= 20 mm
    assert T.NECK_X - T.NECK_HOLE <= -(T.B_HALF - T.B_WALL)  # the bore is flush with the -x wall: no lip to climb
    assert T.CAP_HALF - 0.003 > T.NECK_HALF * math.sqrt(2)  # the cap's skirt clears the square neck as it turns


def test_a_closed_bottle_scores_nothing():
    assert T.score(scene()) == 0.0 and not T.success(scene())


def test_flip_top_route_partial_credit_and_success():
    assert T.score(scene(lid=math.radians(30))) == 0.0  # popped past the detent but not yet 60 degrees open
    assert T.score(scene(lid=T.LID_OPEN)) == 0.25
    assert T.score(scene(3, lid=T.LID_OPEN)) == 0.25
    assert T.score(scene(4, lid=T.LID_OPEN)) == 0.5
    assert T.score(scene(20, lid=T.LID_OPEN)) == 0.75
    assert T.score(scene(31, lid=T.LID_OPEN)) == 0.75
    done = scene(32, lid=T.LID_OPEN)
    assert T.score(done) == 1.0 and T.success(done)


def test_opening_counts_once_even_if_the_lid_is_shut_again():
    T.score(scene(lid=T.LID_OPEN))
    assert T.score(scene(20, lid=0.0)) == 0.75


def test_unscrew_route_partial_credit_and_success():
    b = T.BOTTLE.pos
    resting_on_neck = (b[0] + T.NECK_X, b[1], b[2] + T.NECK_TOP + 0.01)
    assert T.score(scene(cap=resting_on_neck)) == 0.0  # released but still on the neck
    aside = (0.18, -0.14, T.TOP + 0.01)
    assert T.score(scene(cap=aside)) == 0.25
    done = scene(36, cap=aside)
    assert T.score(done) == 1.0 and T.success(done)


def test_beads_still_falling_score_but_do_not_end_the_episode():
    s = scene(40, lid=T.LID_OPEN, moving=True)
    assert T.score(s) == 1.0 and not T.success(s)


def test_knocking_the_cup_scores_zero_for_good():
    moved = (T.CUP.pos[0] - 0.035, T.CUP.pos[1], T.CUP.pos[2])
    assert T.score(scene(40, lid=T.LID_OPEN, cup=moved)) == 0.0
    assert T.score(scene(40, lid=T.LID_OPEN)) == 0.0 and not T.success(scene(40, lid=T.LID_OPEN))


def test_tipping_the_cup_scores_zero():
    assert T.score(scene(40, lid=T.LID_OPEN, cup_q=quat((1, 0, 0), math.radians(25)))) == 0.0


def test_beads_inside_a_bottle_dropped_into_the_cup_do_not_count():
    s = scene(lid=T.LID_OPEN, bottle=T.CUP.pos)
    for b in T.BEADS:  # every bead sits in the bottle, which sits in the cup
        s[b.name] = bead(np.add(b.pos, np.subtract(T.CUP.pos, T.BOTTLE.pos)))
    assert T.score(s) == 0.25 and not T.success(s)


def test_a_bead_outside_the_cup_does_not_count():
    s = scene(4, lid=T.LID_OPEN)
    s["bead_0"] = bead((T.CUP.pos[0] + 0.06, T.CUP.pos[1], T.TOP + 0.003))  # spilled on the table
    assert T.score(s) == 0.25


def test_the_thread_holds_until_the_cap_turns_past_the_slip_angle_then_follows_it():
    hold = T.CAP_YAW
    assert T._slipped(hold + math.radians(3), hold) is None
    assert T._slipped(hold + math.radians(10), hold) == pytest.approx(hold + math.radians(6))
    assert T._slipped(hold - math.radians(10), hold) == pytest.approx(hold - math.radians(6))
    assert T._slipped(math.radians(179), math.radians(-179)) is None  # 2 degrees apart across the wrap


def test_twist_is_the_caps_turn_relative_to_the_bottle():
    bottle = part(T.BOTTLE.pos, quat((0, 0, 1), 0.3))
    cap = part(T.CAP.pos, quat((0, 0, 1), 0.3 + 1.0))
    assert T._twist(bottle, cap) == pytest.approx(1.0)
    assert T._wrap(math.radians(350)) == pytest.approx(math.radians(-10))


@pytest.mark.parametrize("route", ["flip", "unscrew"])
def test_each_demo_is_reachable_within_the_joint_limits_and_run_limit(route):
    plan = T.plan(route)
    total = 0.0
    for arm, wps in plan.items():
        arm_plan = ArmPlan([Waypoint(d, tuple(p), f, label, r) for d, p, r, f, label in wps], station.BASES[arm])
        steps = round(arm_plan.duration * 20) + 5
        actions = np.array([arm_plan.action7((k + 1) / 20)[0] for k in range(steps)])
        assert arm_plan.worst_ik_mm < 1.0 and arm_plan.worst_ik_deg < 0.5, (arm, arm_plan.worst_ik_mm)
        assert np.all(actions[:, :6] >= ik.LOWER) and np.all(actions[:, :6] <= ik.UPPER)
        total = max(total, arm_plan.duration)
    assert total * 20 < 3600


def test_demo_turns_the_cap_far_enough_to_release_it():
    turns = [w for w in T.plan("unscrew")["right"] if w[4] == "turn"]
    assert len(turns) * math.pi / 2 >= T.RELEASE  # 6 quarter turns = 540 degrees >= 450
