"""Scoring tests on handcrafted states, the brake plan, and the demos' reachability. No Isaac Sim needed:

    pytest tasks/spinning-centrifuge      (in an environment with yam_arena installed)
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


def part(pos, q=UP, hinge=0.0, w=0.0):
    pos = np.asarray(pos, float)
    return PartState(pos, np.asarray(q, float), np.zeros(3), np.array([0, 0, w]), pos, pos, hinge=hinge)


def tube(pos, q=UP):
    pos = np.asarray(pos, float)
    h = T.TUBE_H / 2 * abs(math.cos(2 * math.asin(min(1, np.linalg.norm(q[:2])))))
    return ObjectState(pos, np.asarray(q, float), np.zeros(3), np.zeros(3), pos - (0, 0, h), pos + (0, 0, h))


TUBE0 = np.array(T.TUBE.pos)
HOLE = np.array([T.RX + T.HOLES_X[1], T.RY, T.TOP + T.R_FLOOR + T.TUBE_H / 2])
LIFTED = TUBE0 + (0, 0, 0.12)


def scene(lid=0.0, w=0.0, tube_at=TUBE0, tube_q=UP, body=(T.CX, T.CY, T.TOP), body_q=UP):
    return {"centrifuge": part(body, body_q, hinge=lid), "rotor": part((T.CX, T.CY, T.TOP + T.FLOOR), w=w),
            "stop_button": part(T.BUTTON.pos), "tube": tube(tube_at, tube_q), "balance": tube(T.BALANCE.pos),
            "rack": part((T.RX, T.RY, T.TOP))}


@pytest.fixture(autouse=True)
def fresh():
    T._reset_state()


def braked():
    T._STATE.update(pressed=True, brake=(1.0, 2.0), held=True)


def test_the_task_loads():
    assert TASK.scene == "station" and TASK.score_over == "end"
    assert TASK.names == ("centrifuge", "rotor", "stop_button", "tube", "balance", "rack")
    assert all(callable(f) for f in (TASK.success, TASK.score, TASK.demo))


def test_the_closed_lid_covers_the_rotor_and_tubes():
    assert T.TUBE_Z + T.TUBE_H / 2 < T.TOP + T.H  # tubes below the lid
    assert T.R_POCKET + T.POCKET_IN + T.POCKET_WALL < T.BOWL  # rotor inside the bowl
    assert -T.LID_LEN + T.HINGE_AT[0] < -T.BOWL  # lid reaches past the bowl's front wall
    assert T.LID_HAZARD < T.LID_SNAP < T.LID_OPENED


def test_spinning_or_unstopped_scores_zero_and_opening_while_spinning_latches():
    s = scene(w=T.SPIN)
    assert T.score(s) == 0.0
    assert T.score(scene(lid=math.radians(16), w=T.SPIN)) == 0.0
    assert T._STATE["hazard"]
    braked()
    assert T.score(scene(lid=math.radians(100), tube_at=HOLE)) == 0.0  # latched for good


def test_opening_while_braking_is_also_hazard():
    T._STATE.update(pressed=True, brake=(1.0, 2.0))  # braking, not yet held
    assert T.score(scene(lid=math.radians(20), w=2.0)) == 0.0
    braked()
    assert T.score(scene(lid=math.radians(100), w=0.0, tube_at=HOLE)) == 0.0


def test_partial_credit_and_success():
    braked()
    assert T.score(scene(w=0.01)) == 0.25
    assert T.score(scene(lid=math.radians(30))) == 0.25  # not yet 45
    assert T.score(scene(lid=math.radians(100))) == 0.5
    assert T.score(scene(lid=math.radians(100), tube_at=TUBE0 + (0, 0, 0.005))) == 0.5  # barely moved in its pocket
    assert T.score(scene(lid=math.radians(100), tube_at=LIFTED)) == 0.75
    assert T.score(scene(lid=0.0, tube_at=(0.25, 0.15, T.TOP + T.TUBE_R))) == 0.75  # lid closed again later: ok
    done = scene(lid=math.radians(100), tube_at=HOLE)
    assert T.score(done) == 1.0 and T.success(done)
    assert T.score(scene(lid=0.0, tube_at=HOLE)) == 1.0  # the lid may be closed again


def test_rack_needs_upright_inside_a_hole():
    braked()
    T._STATE["opened"] = True
    assert T.score(scene(tube_at=HOLE, tube_q=quat((1, 0, 0), math.radians(20)))) == 0.75
    assert T.score(scene(tube_at=HOLE + (0, 0.02, 0))) == 0.75  # next to the hole, not in it
    assert T.score(scene(tube_at=HOLE + (0, 0, 0.05))) == 0.75  # held above the hole
    assert T.score(scene(tube_at=HOLE + (T.HOLES_X[0], 0, 0))) == 1.0  # any hole


def test_moving_the_centrifuge_or_dropping_the_tube_scores_zero():
    braked()
    T._STATE["opened"] = True
    assert T.score(scene(tube_at=HOLE, body=(T.CX + 0.04, T.CY, T.TOP))) == 0.0
    T._reset_state()
    braked()
    T._STATE["opened"] = True
    assert T.score(scene(tube_at=HOLE, body_q=quat((0, 1, 0), math.radians(12)))) == 0.0
    T._reset_state()
    braked()
    T._STATE["opened"] = True
    assert T.score(scene(tube_at=(0.3, 0.0, T.TOP - 0.7))) == 0.0


def test_brake_parks_the_sample_tube_in_front_within_the_time_window():
    for a in np.linspace(-math.pi, math.pi, 37):
        dur = T.brake_time(a)
        assert T.BRAKE_MIN <= dur <= T.BRAKE_MIN + 2 * (2 * math.pi) / T.SPIN + 1e-9
        travel = T.SPIN * dur / 2  # even deceleration
        assert abs(T._wrap(a + travel - T.PARK)) < 1e-9
    assert T.motor_speed(0.0, None) == 0.0 and T.motor_speed(1.0, None) == T.SPIN
    assert T.motor_speed(2.0, (1.0, 2.0)) == pytest.approx(T.SPIN / 2) and T.motor_speed(3.5, (1.0, 2.0)) == 0.0


@pytest.mark.parametrize("route", ["demo", "naive"])
def test_each_demo_is_reachable_within_the_joint_limits_and_run_limit(route):
    for arm, wps in T.plan(route).items():
        arm_plan = ArmPlan([Waypoint(d, tuple(p), f, label, r) for d, p, r, f, label in wps], station.BASES[arm])
        steps = round(arm_plan.duration * 20) + 5
        actions = np.array([arm_plan.action7((k + 1) / 20)[0] for k in range(steps)])
        assert arm_plan.worst_ik_mm < 1.0 and arm_plan.worst_ik_deg < 0.5, (arm, arm_plan.worst_ik_mm)
        assert np.all(actions[:, :6] >= ik.LOWER) and np.all(actions[:, :6] <= ik.UPPER)
        assert arm_plan.duration * 20 < 1800


def test_the_demo_waits_for_the_stop_before_lifting_the_lid():
    wps = T.plan("demo")["right"]
    t_press = sum(w[0] for w in wps[:2])
    t_lift = sum(w[0] for w in wps[:next(i for i, w in enumerate(wps) if w[4] == "lift the lid")])
    assert t_lift - t_press > T.BRAKE_MIN + 2 * 2 * math.pi / T.SPIN  # the longest brake
