"""Scoring tests on handcrafted states, the door latch rule, and the demos' reachability. No Isaac Sim needed:

    pytest tasks/spoon-in-microwave      (in an environment with yam_arena installed)
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
from jointkit import PartState, matrix_to_quat, rot_z  # noqa: E402  (task.py put this folder on sys.path)

UP = np.array([0.0, 0.0, 0.0, 1.0])
INSIDE = (T.MX + 0.115, T.MY + 0.01, T.TOP + T.FLOOR)   # the bowl standing on the cavity floor
IN_TRAY = (T.TX, T.TY, T.TOP + 0.006)


def quat(axis, angle):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    return np.r_[a * math.sin(angle / 2), math.cos(angle / 2)]


def part(pos, q=UP, hinge=0.0, vel=(0, 0, 0)):
    pos, q = np.asarray(pos, float), np.asarray(q, float)
    return PartState(pos, q, np.asarray(vel, float), np.zeros(3), pos - 0.05, pos + 0.05, hinge=hinge)


def scene(opening_deg=95.0, bowl=None, bowl_q=UP, spoon=None, spoon_q=None, mw=None, moving=False):
    """Defaults: the starting layout. ``spoon`` given without ``spoon_q``: lying flat along y."""
    hinge = T.DOOR0 - math.radians(opening_deg)
    if spoon is None:
        spoon, sq = T.SP_POS, T.SPOON.rot
    else:
        sq = spoon_q if spoon_q is not None else matrix_to_quat(rot_z(math.pi / 2))
    return {"microwave": part(mw or T.MICROWAVE.pos, hinge=hinge), "bowl": part(bowl or T.BOWL.pos, bowl_q),
            "spoon": part(spoon, sq, vel=(0, 0, -0.2) if moving else (0, 0, 0)), "tray": part(T.TRAY.pos)}


@pytest.fixture(autouse=True)
def fresh():
    T._reset_state()


def test_the_task_loads_with_its_objects():
    assert TASK.scene == "station" and TASK.score_over == "end"
    assert TASK.names == ("microwave", "bowl", "spoon", "tray")
    assert all(callable(f) for f in (TASK.success, TASK.score, TASK.demo))
    assert TASK.docs.startswith("On the station table")


def test_the_spoon_starts_in_the_bowl_resting_on_the_rim_and_the_soup():
    s = scene()
    ends = T.spoon_points(s["spoon"])
    assert ends[2][1] > T.BY and ends[0][1] < T.BY - T.B          # head inside, handle out over the -y rim
    assert ends[2][2] - T.TOP == pytest.approx(T.SOUP_Z + T.SP_T / 2 / math.cos(T.SP_TILT) + 0.001, abs=0.003)
    assert math.degrees(T.SP_TILT) == pytest.approx(17.2, abs=0.5)


def test_the_cavity_floor_is_raised_so_the_bowl_must_be_lifted_in():
    assert T.FLOOR >= 0.03
    on_table_in_front = (T.MX - 0.06, T.MY, T.TOP)
    assert not T.in_cavity(np.add(on_table_in_front, (0.07, 0, 0.01)), part(T.MICROWAVE.pos))


def test_the_start_scores_zero():
    assert T.score(scene()) == 0.0 and not T.success(scene())


def test_ladder():
    assert T.score(scene(spoon=IN_TRAY)) == 0.25
    assert T.score(scene(spoon=(0.10, 0.0, T.TOP + 0.002))) == 0.25         # anywhere on the table counts
    assert T.score(scene(spoon=IN_TRAY, bowl=INSIDE)) == 0.5
    assert T.score(scene(14, spoon=IN_TRAY, bowl=INSIDE)) == 0.75
    assert T.score(scene(4, spoon=IN_TRAY, bowl=INSIDE)) == 0.75
    done = scene(0, spoon=IN_TRAY, bowl=INSIDE)
    assert T.score(done) == 1.0 and T.success(done)


def test_a_moving_spoon_scores_but_does_not_end_the_episode():
    s = scene(0, spoon=IN_TRAY, bowl=INSIDE, moving=True)
    assert T.score(s) == 1.0 and not T.success(s)


def test_naive_bowl_with_spoon_in_and_door_shut_is_arcing_and_stays_zero():
    shift = np.subtract(INSIDE, T.BOWL.pos)
    s = scene(0, bowl=INSIDE, spoon=tuple(T.SP_POS + shift), spoon_q=T.SPOON.rot)
    assert T.score(s) == 0.0
    assert T.score(scene(0, spoon=IN_TRAY, bowl=INSIDE)) == 0.0              # arcing is sticky


def test_spoon_dropped_in_the_cavity_with_the_door_latched_is_arcing():
    s = scene(0, bowl=INSIDE, spoon=(T.MX + 0.03, T.MY - 0.08, T.TOP + T.FLOOR + 0.002))
    assert T.score(s) == 0.0 and T._STATE["arced"]


def test_spoon_in_the_cavity_with_the_door_open_is_not_arcing_but_scores_nothing():
    s = scene(30, bowl=INSIDE, spoon=(T.MX + 0.03, T.MY - 0.08, T.TOP + T.FLOOR + 0.002))
    assert T.score(s) == 0.0 and not T._STATE["arced"]
    assert T.score(scene(0, spoon=IN_TRAY, bowl=INSIDE)) == 1.0


def test_spoon_handle_poking_into_the_cavity_counts_as_in():
    p = (T.MX - 0.05, T.MY, T.TOP + T.FLOOR + 0.01)                       # middle 5 cm out, head end 2.5 cm in
    s = scene(0, bowl=INSIDE, spoon=p, spoon_q=UP)
    assert T.score(s) == 0.0 and T._STATE["arced"]


def test_bowl_not_lifted_in_on_the_table_in_front_scores_spoon_only():
    assert T.score(scene(0, spoon=IN_TRAY, bowl=(T.MX - 0.08, T.MY, T.TOP))) == 0.25


def test_spilling_the_bowl_scores_zero_for_good():
    assert T.score(scene(spoon=IN_TRAY, bowl=INSIDE, bowl_q=quat((1, 0, 0), math.radians(35)))) == 0.0
    assert T.score(scene(0, spoon=IN_TRAY, bowl=INSIDE)) == 0.0


def test_moving_the_microwave_scores_zero():
    mw = (T.MX + 0.035, T.MY, T.TOP)
    bowl = np.add(INSIDE, (0.035, 0, 0))
    assert T.score(scene(0, spoon=IN_TRAY, bowl=tuple(bowl), mw=mw)) == 0.0


def test_the_latch():
    assert T.latch_target(math.radians(40)) is None
    assert T.latch_target(math.radians(6)) == pytest.approx(T.DOOR0 - math.radians(10))   # cam pushes it back out
    assert T.latch_target(math.radians(2)) > T.DOOR0                                     # held shut
    assert T.door_open(part(T.MICROWAVE.pos, hinge=T.DOOR0)) == 0.0


def test_door_point_lies_on_the_outer_face():
    x, y, _ = T.door_point(0.0, 0.10)
    assert x == pytest.approx(T.HINGE_XY[0] - T.OUTSIDE) and y == pytest.approx(T.HINGE_XY[1] - 0.10)
    x, y, _ = T.door_point(math.pi / 2, 0.10)
    assert x == pytest.approx(T.HINGE_XY[0] - 0.10) and y == pytest.approx(T.HINGE_XY[1] + T.OUTSIDE)


@pytest.mark.parametrize("route", ["safe", "naive"])
def test_each_demo_is_reachable_within_the_joint_limits_and_run_limit(route):
    total = 0.0
    for arm, wps in T.plan(route).items():
        arm_plan = ArmPlan([Waypoint(d, tuple(p), f, label, r) for d, p, r, f, label in wps], station.BASES[arm])
        steps = round(arm_plan.duration * 20) + 5
        actions = np.array([arm_plan.action7((k + 1) / 20)[0] for k in range(steps)])
        assert arm_plan.worst_ik_mm < 1.0 and arm_plan.worst_ik_deg < 0.5, (arm, arm_plan.worst_ik_mm)
        assert np.all(actions[:, :6] >= ik.LOWER) and np.all(actions[:, :6] <= ik.UPPER)
        total = max(total, arm_plan.duration)
    assert total * 20 < 1800
