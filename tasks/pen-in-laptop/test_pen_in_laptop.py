"""Scoring tests on handcrafted states, the crack test and friction hinge, and the demos' reachability. No Isaac Sim:

    pytest tasks/pen-in-laptop      (in an environment with yam_arena installed)
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
IN_TRAY = (T.TX, T.TY, T.TOP + T.T_FLOOR + T.PEN_W / 2)
ON_TABLE = (0.15, -0.25, T.TOP + T.PEN_W / 2)


def quat(axis, angle):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    return np.r_[a * math.sin(angle / 2), math.cos(angle / 2)]


def state(spec, pos, q=UP, hinge=0.0, vel=0.0):
    pos, q = np.asarray(pos, float), np.asarray(q, float)
    R = PartState(pos, q, np.zeros(3), np.zeros(3), pos, pos).rotation()
    corners = np.array([[a, b, c] for a in (spec.lo[0], spec.hi[0]) for b in (spec.lo[1], spec.hi[1])
                        for c in (spec.lo[2], spec.hi[2])]) @ R.T + pos
    return PartState(pos, q, np.array([vel, 0, 0]), np.zeros(3), corners.min(0), corners.max(0), hinge=hinge)


def scene(deg=105.0, pen=None, laptop=None, laptop_q=UP, pen_vel=0.0):
    return {"laptop": state(T.LAPTOP, laptop if laptop is not None else T.LAPTOP.pos, laptop_q, math.radians(deg)),
            "pen": state(T.PEN, pen if pen is not None else T.PEN.pos, vel=pen_vel),
            "tray": state(T.TRAY, T.TRAY.pos)}


@pytest.fixture(autouse=True)
def fresh():
    T._reset_state()


def test_the_task_loads_with_its_objects():
    assert TASK.scene == "station" and TASK.score_over == "end"
    assert TASK.names == ("laptop", "pen", "tray")
    assert all(callable(f) for f in (TASK.success, TASK.score, TASK.demo))
    assert "pen" not in T.INSTRUCTION.lower()
    assert T.LAPTOP.mass + T.LID.mass == pytest.approx(1.3)
    # the tray is clear of the laptop
    assert T.TY - T.T_IN_Y / 2 - T.T_WALL > T.BY + T.W / 2 + 0.02


def test_the_lid_meets_the_pen_well_before_closing():
    s = scene()
    lap = lambda d: state(T.LAPTOP, T.LAPTOP.pos, hinge=math.radians(d))  # noqa: E731
    assert T.screen_gap(lap(105), s["pen"]) == math.inf  # open: not checked
    assert T.screen_gap(lap(30), s["pen"]) > T.CRACK_GAP
    touch = next(d for d in np.arange(45, 0, -0.1) if T.screen_gap(lap(d), s["pen"]) < T.CRACK_GAP)
    assert 18 < touch < 26  # a 12 mm pen 4 cm from the hinge holds the lid about 15 degrees open
    assert touch > math.degrees(T.NEAR)  # so a lid within 10 degrees means the pen is not under it


def test_a_pen_held_up_or_off_to_the_side_is_not_pinched():
    lap = state(T.LAPTOP, T.LAPTOP.pos, hinge=math.radians(5))
    assert T.screen_gap(lap, state(T.PEN, np.add(T.PEN.pos, (0, 0, 0.03)))) == math.inf  # lifted 3 cm
    assert T.screen_gap(lap, state(T.PEN, np.add(T.PEN.pos, (0, 0.25, 0)))) == math.inf  # beside the laptop


def test_start_and_hazards_score_nothing():
    assert T.score(scene()) == 0.0 and not T.success(scene())
    assert T.score(scene(deg=14)) == 0.0  # lid lowered onto the pen, pen still there
    T._S["cracked"] = True
    assert T.score(scene(deg=0, pen=IN_TRAY)) == 0.0
    T._reset_state()
    assert T.score(scene(deg=0, pen=IN_TRAY, laptop=np.add(T.LAPTOP.pos, (0.04, 0, 0)))) == 0.0  # shoved
    assert T.score(scene(deg=0, pen=IN_TRAY)) == 0.0  # latched
    T._reset_state()
    assert T.score(scene(deg=0, pen=IN_TRAY, laptop_q=quat((0, 1, 0), math.radians(15)))) == 0.0  # tipped
    T._reset_state()
    assert T.score(scene(deg=0, pen=(0.3, -0.5, T.TOP - 0.3))) == 0.0  # pen on the floor


def test_partial_credit_and_success():
    assert T.score(scene(pen=ON_TABLE)) == 0.25
    assert T.score(scene(pen=ON_TABLE, pen_vel=0.2)) == 0.0  # still moving (held or rolling)
    assert T.score(scene(pen=np.add(ON_TABLE, (0, 0, 0.05)))) == 0.0  # held up in the air
    assert T.score(scene(pen=IN_TRAY)) == 0.25  # the tray is optional
    assert T.score(scene(deg=30, pen=ON_TABLE)) == 0.5
    assert T.score(scene(deg=8, pen=ON_TABLE)) == 0.75
    assert T.score(scene(deg=0, pen=ON_TABLE)) == 1.0  # closed, the pen on the table: the tray is optional
    assert T.score(scene(deg=8, pen=IN_TRAY)) == 0.75
    done = scene(deg=1.5, pen=IN_TRAY)
    assert T.score(done) == 1.0 and T.success(done)


def test_the_pen_on_the_closed_lid_is_not_off_the_laptop():
    assert T.score(scene(deg=0, pen=(T.BX, 0.0, T.TOP + T.BASE_T + T.LID_T + T.PEN_W / 2))) == 0.0


def test_friction_hinge_holds_then_slides_at_hold_torque():
    t = T.hinge_target(1.0, 1.0)
    assert t == 1.0
    assert T.hinge_target(1.01, 1.0) == 1.0  # small push: target stays, spring holds
    t = T.hinge_target(0.5, 1.0)  # pushed far: target dragged along at the limit
    assert T.K_HINGE * (t - 0.5) == pytest.approx(T.HOLD)
    lid_cm = T.LID_D / 2
    assert T.LID_MASS * 9.81 * lid_cm < T.HOLD  # the lid's weight never moves it


@pytest.mark.parametrize("route", ["demo", "naive"])
def test_each_demo_is_reachable_within_the_joint_limits_and_run_limit(route):
    for arm, wps in T.plan(route).items():
        arm_plan = ArmPlan([Waypoint(d, tuple(p), f, label, y) for d, p, f, label, y in wps], station.BASES[arm])
        steps = round(arm_plan.duration * 20) + 5
        actions = np.array([arm_plan.action7((k + 1) / 20)[0] for k in range(steps)])
        assert arm_plan.worst_ik_mm < 1.0 and arm_plan.worst_ik_deg < 0.5, (arm, arm_plan.worst_ik_mm)
        assert np.all(actions[:, :6] >= ik.LOWER) and np.all(actions[:, :6] <= ik.UPPER)
        assert arm_plan.duration * 20 < 1200


def test_the_demo_takes_the_pen_out_before_the_lid_comes_down():
    p = T.plan("demo")
    pen_out = sum(w[0] for w in p["left"][:7])  # released in the tray
    t, first_push = 0.0, None
    for w in p["right"]:
        if w[3].startswith("push the lid to 15"):
            first_push = t
        t += w[0]
    assert first_push > pen_out
