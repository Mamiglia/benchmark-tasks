"""Scoring tests on handcrafted states, the latch model, and the demos' reachability. No Isaac Sim needed:

    pytest tasks/unlatched-toolbox      (in an environment with yam_arena installed)
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
from boxkit import Latches, ToolBoxState  # noqa: E402  (task.py put this folder on sys.path)

UP = np.array([0.0, 0.0, 0.0, 1.0])
BOX0 = np.array(T.TOOLBOX.pos)
SHELF = np.array([T.SX, T.SY, T.SHELF_TOP])
P = T.PARAMS


def quat(axis, angle):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    return np.r_[a * math.sin(angle / 2), math.cos(angle / 2)]


def box(pos=BOX0, q=UP, lid=0.0, engaged=(False, False), vel=(0, 0, 0)):
    pos = np.asarray(pos, float)
    return ToolBoxState(pos, np.asarray(q, float), np.asarray(vel, float), np.zeros(3), pos, pos, lid_deg=lid,
                        engaged=tuple(engaged))


def scene(b=None, out=None):
    """The toolbox ``b`` with every tool where it started relative to the base, or
    tool ``out`` put on the table beside the box."""
    b = b if b is not None else box()
    objs = {"toolbox": b}
    for t in T.TOOLS:
        p = np.asarray(t.pos, float) - BOX0
        p = b.rotation() @ p + b.pos
        if t.name == out:
            p = np.array([T.BX - 0.2, T.BY, T.TOP + 0.01])
        objs[t.name] = ObjectState(p, UP, np.zeros(3), np.zeros(3), p, p)
    return objs


BOTH = (True, True)


@pytest.fixture(autouse=True)
def fresh():
    T._reset_state()


def test_loads_with_objects_and_hooks():
    assert TASK.scene == "station" and TASK.score_over == "end"
    assert TASK.free_objects == ("toolbox", "wrench", "tape", "tin", "pliers")
    assert all(callable(f) for f in (TASK.success, TASK.score, TASK.demo))
    assert "latch" not in T.INSTRUCTION.lower()


def test_the_base_is_too_wide_to_pinch_and_the_shelf_too_high_to_shove_onto():
    assert min(T.L, T.W) > 2 * abs(-0.04695) + 0.02  # 12 cm against the gripper's 9.4 cm opening
    assert T.S_H + T.LIP >= 0.09
    # the box fits inside the lip with at least 2 cm to spare all round
    assert T.SHELF_IN[0] - T.L / 2 >= 0.019 and T.SHELF_IN[1] - T.W / 2 >= 0.02


def test_tools_start_inside_and_apart():
    s = scene()
    assert all(T.tool_inside(s["toolbox"], s[n]) for n in T.TOOL_NAMES)


def test_start_and_ladder():
    assert T.score(scene()) == 0.0
    assert T.score(scene(box(engaged=(True, False)))) == 0.25
    assert T.score(scene(box(engaged=BOTH))) == 0.5
    lifted = BOX0 + (0, -0.1, 0.12)
    assert T.score(scene(box(lifted, engaged=BOTH))) == 0.75
    assert T.score(scene(box(BOX0, engaged=BOTH))) == 0.75  # set back down on the table: still lifted once
    done = scene(box(SHELF, engaged=BOTH))
    assert T.score(done) == 1.0 and T.success(done)


def test_shelf_without_latches_or_lifted_unlatched_is_not_credit():
    T._STATE["lifted"] = False
    assert T.score(scene(box(SHELF))) == 1.0  # carried there some safe way, never latched: nothing spilled
    T._reset_state()
    assert T.score(scene(box(BOX0 + (0, 0, 0.08), engaged=(True, False)))) == 0.25  # one latch: no lift credit
    assert T.score(scene(box(SHELF, engaged=(True, False)))) == 1.0  # on the shelf, nothing spilled


def test_shelf_needs_inside_lip_upright_and_still():
    assert T.score(scene(box(SHELF + (0.0, 0.10, 0.0), engaged=BOTH))) < 1.0  # centre outside the lip
    T._reset_state()
    assert T.score(scene(box(SHELF + (0, 0, 0.03), engaged=BOTH))) < 1.0  # held above it
    T._reset_state()
    assert T.score(scene(box(SHELF, engaged=BOTH, vel=(0, 0.05, 0)))) < 1.0  # still moving
    T._reset_state()
    assert T.score(scene(box(SHELF, quat((1, 0, 0), math.radians(15)), engaged=BOTH))) < 1.0  # tilted on the lip


def test_failures_are_zero_for_good():
    assert T.score(scene(box(engaged=BOTH), out="tape")) == 0.0
    assert T.score(scene(box(SHELF, engaged=BOTH))) == 0.0  # stays 0 after a tool fell out
    for b in (box(BOX0 + (0, 0, 0.1), lid=40.0),  # the lid swung open in the air (the naive lift)
              box(q=quat((1, 0, 0), math.radians(60)), engaged=BOTH),  # tipped over
              box(BOX0 - (0, 0, 0.3), engaged=BOTH),  # fell off the table
              box(BOX0 + (0, 0, 0.05), engaged=BOTH, vel=(0, 0, -1.5))):  # dropped
        T._reset_state()
        assert T.score(scene(b)) == 0.0, b
        assert T.score(scene(box(SHELF, engaged=BOTH))) == 0.0


def test_opening_the_lid_on_the_table_is_not_a_failure():
    assert T.score(scene(box(lid=80.0))) == 0.0 and not T._STATE["failed"]
    assert T.score(scene(box(SHELF, lid=80.0))) == 0.0 and not T._STATE["failed"]


# -- the latch model ----------------------------------------------------------------------------------------------


def test_a_latch_snaps_shut_near_closed_and_engages_only_with_the_lid_shut():
    m = Latches(P)
    _, taus = m.step(0.0, 0.0, [P.open_deg, P.open_deg])
    assert taus == [0.0, 0.0] and m.engaged == [False, False]
    lid, taus = m.step(0.0, 0.0, [P.snap_deg - 1, P.open_deg])
    assert taus == [P.hold_nm, 0.0] and m.engaged == [True, False]
    lid, _ = m.step(5.0, 0.0, [0.0, P.open_deg])
    assert lid == pytest.approx(-P.lock_stiffness * math.radians(5.0))  # held shut
    m.step(0.0, 0.0, [P.release_deg + 1, P.open_deg])
    assert m.engaged == [False, False]
    lid, _ = m.step(5.0, 0.0, [P.release_deg + 1, P.open_deg])
    assert lid == 0.0


def test_a_latch_closed_on_an_open_lid_does_not_lock_it_later():
    m = Latches(P)
    m.step(40.0, 0.0, [0.0, 0.0])
    assert m.engaged == [False, False]
    lid, _ = m.step(0.0, 0.0, [0.0, 0.0])  # lid brought down onto closed latches
    assert m.engaged == [False, False] and lid == 0.0
    m.step(0.0, 0.0, [P.open_deg, P.open_deg])
    m.step(0.0, 0.0, [0.0, 0.0])
    assert m.engaged == [True, True]


def test_the_lock_holds_the_loaded_box_within_a_degree():
    load_nm = (T.TOOLBOX.mass + sum(t.mass for t in T.TOOLS)) * 9.81 * T.L / 2  # base and tools hanging 11 cm from the hinge
    assert math.degrees(load_nm / P.lock_stiffness) < 1.0
    assert P.lid_shut_deg < P.snap_deg < P.release_deg < P.open_deg


# -- the demos ----------------------------------------------------------------------------------------------------


@pytest.mark.parametrize("route", ["demo", "naive"])
def test_each_demo_is_reachable_within_joint_limits_and_run_limit(route):
    for arm, wps in T.plan(route).items():
        ap = ArmPlan([Waypoint(d, tuple(p), f, label, r) for d, p, r, f, label in wps], station.BASES[arm])
        steps = round(ap.duration * 20) + 5
        actions = np.array([ap.action7((k + 1) / 20)[0] for k in range(steps)])
        assert ap.worst_ik_mm < 1.0 and ap.worst_ik_deg < 0.5, (arm, ap.worst_ik_mm)
        assert np.all(actions[:, :6] >= ik.LOWER) and np.all(actions[:, :6] <= ik.UPPER)
        assert ap.duration * 20 < 1800
