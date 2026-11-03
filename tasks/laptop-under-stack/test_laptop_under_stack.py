"""Scoring on handcrafted states, the hinge's torque balance, and the demo's reach (no Isaac Sim).

Run from this folder in Arena's environment:  python -m pytest test_laptop_under_stack.py
"""

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.constants import CONTROL_HZ
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).resolve().parent / "task.py")
M = TASK.module
from hingekit import HingeState, base_bounds  # noqa: E402  (task.py put this folder on sys.path)

TOP = M.TOP


def quat_x(deg):
    a = math.radians(deg) / 2
    return np.array([math.sin(a), 0.0, 0.0, math.cos(a)])


def obj(pos, size, tilt=0.0, vel=(0.0, 0.0, 0.0)):
    pos, half = np.asarray(pos, float), np.asarray(size, float) / 2
    return ObjectState(pos, quat_x(tilt), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)


def laptop(angle=0.0, dx=0.0, tilt=0.0):
    pos = np.add(M.BASE_POS, (dx, 0.0, 0.0))
    q = quat_x(tilt)
    low, high = base_bounds(M.LAPTOP, pos, HingeState(pos, q, np.zeros(3), np.zeros(3), pos, pos).rotation())
    return HingeState(pos, q, np.zeros(3), np.zeros(3), low, high, math.radians(angle))


MUG_SIZE = (0.07, 0.07, 0.095)
SIZES = {"mug": MUG_SIZE, "phone": M.PHONE.size, "adapter_box": M.ADAPTER.size}
START = {"mug": M.MUG.pos, "phone": M.PHONE.pos, "adapter_box": M.ADAPTER.pos}
ON_TABLE = {"mug": (0.16, 0.25), "phone": (0.30, -0.24), "adapter_box": (0.13, -0.25)}


def state(where=None, lid=0.0, lap=None, **override):
    """Items at their start unless ``where`` says 'up' (5 cm above the start) or 'table' (set down)."""
    where = where or {}
    out = {"laptop": lap or laptop(lid)}
    for name, size in SIZES.items():
        spot = where.get(name, "start")
        if spot == "up":
            out[name] = obj(np.add(START[name], (0, 0, 0.05)), size)
        elif spot == "table":
            out[name] = obj((*ON_TABLE[name], TOP + size[2] / 2), size)
        else:
            out[name] = obj(START[name], size)
    out.update(override)
    return out


def run(*states):
    M._reset()
    for s in states:
        result = (TASK.score(s), TASK.success(s))
    return result


ALL_UP = {n: "up" for n in SIZES}
ALL_DOWN = {n: "table" for n in SIZES}


def test_the_task_loads_with_its_objects_hooks_and_docs():
    assert TASK.scene == "station" and TASK.score_over == "end"
    assert TASK.names == ("laptop", "mug", "phone", "adapter_box") == TASK.free_objects
    assert all(callable(f) for f in (TASK.success, TASK.score, TASK.distance, TASK.demo))
    assert TASK.docs.startswith("On the station table")


def test_the_start_scores_zero():
    assert run(state()) == (0.0, False)


def test_clearing_everything_and_opening_the_lid_succeeds():
    assert run(state(), state(ALL_UP), state(ALL_DOWN), state(ALL_DOWN, lid=99)) == (1.0, True)


@pytest.mark.parametrize(
    "where, lid, expected",
    [
        ({"mug": "table"}, 0, 0.15),
        ({"mug": "table", "adapter_box": "table"}, 0, 0.30),
        (ALL_DOWN, 0, 0.45),
        (ALL_DOWN, 30, 0.45),
        (ALL_DOWN, 60, 0.70),
        (ALL_DOWN, 89, 0.70),
    ],
)
def test_partial_credit(where, lid, expected):
    picked = {n: "up" for n in where}
    assert run(state(), state(picked), state(where, lid=lid))[0] == pytest.approx(expected)


def test_the_lid_earns_nothing_while_anything_is_still_on_the_laptop():
    where = {"mug": "table", "adapter_box": "table"}
    assert run(state(), state({"mug": "up", "adapter_box": "up"}), state(where, lid=100))[0] == pytest.approx(0.30)


def test_an_item_set_down_badly_does_not_count():
    lean = obj((0.30, -0.24, TOP + 0.03), M.PHONE.size, tilt=45)
    outside = obj((0.30, -0.14 - 0.039 - 0.01, TOP + 0.007), M.PHONE.size)  # only 1 cm from the laptop
    sliding = obj((0.30, -0.24, TOP + 0.007), M.PHONE.size, vel=(0.05, 0, 0))
    mug_upside_down = obj((0.16, 0.25, TOP + 0.0475), MUG_SIZE, tilt=180)
    for bad in (lean, outside, sliding):
        assert run(state(), state(ALL_UP), state(ALL_DOWN, lid=99, phone=bad)) == (pytest.approx(0.30), False)
    # the mug at 180 deg has also spilled: a hazard, so 0
    assert run(state(), state(ALL_UP), state(ALL_DOWN, mug=mug_upside_down))[0] == 0.0


@pytest.mark.parametrize(
    "hazard",
    [
        state(lap=laptop(tilt=8)),  # lid lifted one-handed: the base tips
        state(ALL_UP, mug=obj(np.add(M.MUG.pos, (0, 0, 0.05)), MUG_SIZE, tilt=50)),  # rice poured in transit
        state(phone=obj((0.30, -0.24, TOP + 0.007), M.PHONE.size)),  # pushed off the laptop, never picked up
        state(mug=obj((0.27, 0.07, TOP - 0.3), MUG_SIZE)),  # off the table
    ],
)
def test_any_hazard_zeroes_an_otherwise_perfect_trial(hazard):
    assert run(state(), hazard, state(ALL_UP), state(ALL_DOWN), state(ALL_DOWN, lid=99)) == (0.0, False)


def test_an_item_anywhere_on_the_table_2_cm_clear_of_the_laptop_counts():
    far = obj((0.45, -0.35, TOP + 0.007), M.PHONE.size)
    just = obj((0.30, -0.14 - 0.039 - 0.021, TOP + 0.007), M.PHONE.size)  # 2.1 cm from the laptop
    for ok in (far, just):
        assert run(state(), state(ALL_UP), state(ALL_DOWN, lid=99, phone=ok)) == (1.0, True)


def test_the_laptop_must_end_on_its_spot():
    moved = state(ALL_DOWN, lap=laptop(angle=99, dx=0.025))
    assert run(state(), state(ALL_UP), moved) == (0.0, False)
    assert run(state(), state(ALL_UP), state(ALL_DOWN, lap=laptop(angle=99, dx=0.015))) == (1.0, True)


def test_items_carried_up_by_the_lid_are_not_picked_up():
    raised = obj(np.add(M.PHONE.pos, (0, 0, 0.05)), M.PHONE.size)
    slid = state({"mug": "table", "adapter_box": "table", "phone": "table"}, lid=99)
    assert run(state(), state({"mug": "up", "adapter_box": "up"}, lid=30, phone=raised), slid) == (0.0, False)


def test_a_new_trial_forgets_the_last_ones_hazards():
    run(state(lap=laptop(tilt=20)))
    assert run(state(), state(ALL_UP), state(ALL_DOWN, lid=99)) == (1.0, True)


def test_distance_is_how_far_the_lid_has_to_open():
    assert TASK.distance(state()) == pytest.approx(math.pi / 2)
    assert TASK.distance(state(lid=100)) == 0.0


def test_one_handed_lifting_tips_the_laptop_before_the_hinge_turns():
    g = 9.81
    (c0, s0, _), (c1, s1, _) = M.LID_BOXES
    lid_com = -(c0[0] * s0[0] * s0[1] + c1[0] * s1[0] * s1[1]) / (s0[0] * s0[1] + s1[0] * s1[1])  # 9.5 cm out
    base_moment = M.BASE_MASS * g * M.BASE_SIZE[0] / 2  # about the base's far bottom edge
    lid_moment = M.LID_MASS * g * lid_com
    reach = M.HINGE[0] - M.lid_point(M.PINCH_IN_LID, 0)[0]
    to_open, to_tip = (M.HOLD_TORQUE + lid_moment) / reach, (base_moment + lid_moment) / reach
    assert M.HOLD_TORQUE > 1.5 * base_moment  # the hinge out-torques the base's weight by 50 %
    assert to_tip < to_open < 7.0  # N at the tab: tips at 4.6 N, the hinge turns at 6.4 N
    press = (M.HOLD_TORQUE - base_moment) / (M.BASE_SIZE[0] - M.LEDGE / 2)
    assert 2.0 < press < 2.5  # N on the ledge keeps the base down
    assert M.HOLD_TORQUE > lid_moment  # the lid stays wherever it is left


def plans():
    return {arm: ArmPlan(w, station.BASES[arm]) for arm, w in TASK.demo().items()}


def test_the_demo_is_reachable_within_the_joint_limits():
    for arm in plans().values():
        actions = np.array([arm.action7((k + 1) / CONTROL_HZ)[0] for k in range(round(arm.duration * CONTROL_HZ) + 5)])
        assert arm.worst_ik_mm < 0.5 and arm.worst_ik_deg < 0.1
        assert np.all(actions[:, :6] >= ik.LOWER) and np.all(actions[:, :6] <= ik.UPPER)


def test_the_left_arm_holds_the_base_until_the_lid_is_pushed_open(monkeypatch):
    p = TASK.demo()
    left_t = right_t = 0.0
    for w in p["right"]:
        right_t += w.duration
        if w.label == "back off":
            break
    for w in p["left"]:
        left_t += w.duration
        if w.label == "hold the base down":
            break
    assert left_t == pytest.approx(right_t)
    assert max(sum(w.duration for w in ws) for ws in p.values()) * 20 < 1200  # steps at 20 Hz
    monkeypatch.setenv("LAPTOP_DEMO_ONE_HAND", "1")
    assert not any("press" in w.label for w in TASK.demo()["left"])
