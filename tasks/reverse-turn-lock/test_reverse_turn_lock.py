"""Scoring, lock mechanics and demo reach for the reverse-turn lock task. No Isaac Sim needed.

    pytest tasks/reverse-turn-lock/test_reverse_turn_lock.py     (in the yam_arena environment)
"""

import math
import pathlib
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from lockkit import Lock, LockBoxState, LockParams, resistance_nm  # noqa: E402
from yam_arena.task_file import load  # noqa: E402

TASK = load(HERE / "task.py")
T = TASK.module
P = LockParams()
NOFRIC = LockParams(lid_friction_n=0.0)  # isolates the catch from the lid's rail friction


def quat_yaw(deg):
    return np.array([0.0, 0.0, math.sin(math.radians(deg) / 2), math.cos(math.radians(deg) / 2)])


def state(key_deg=0.0, lid_m=0.0, unlocked=False, damaged=False, pried=False, shift=(0.0, 0.0), yaw=0.0,
          max_tilt_deg=0.0, max_drop_m=0.0):  # fmt: skip
    pos = np.array([T.BOX_XY[0] + shift[0], T.BOX_XY[1] + shift[1], T.TOP])
    return {
        "lockbox": LockBoxState(
            pos, quat_yaw(yaw), np.zeros(3), np.zeros(3), pos - 0.1, pos + 0.1, key_deg=key_deg, lid_m=lid_m,
            unlocked=unlocked, damaged=damaged, pried=pried, max_tilt_deg=max_tilt_deg, max_drop_m=max_drop_m,
        )  # fmt: skip
    }


def turn(lock, angles, lid_m=0.0):
    """Drive the lock through key angles (deg), one physics step each; return the last (torque, force)."""
    out = (0.0, 0.0)
    for a in angles:
        out = lock.step(a, lid_m)
    return out


def stop_angle(torque_nm):
    """The key angle (deg) at which the wrong-way stop pushes back with ``torque_nm``."""
    return P.stop_deg - math.degrees(torque_nm / P.stop_stiffness)


# -- task file ---------------------------------------------------------------------------------------------------------


def test_the_task_loads_with_one_loose_lockbox_and_its_hooks():
    assert TASK.scene == "station" and TASK.names == ("lockbox",) == TASK.free_objects
    assert all(callable(f) for f in (TASK.success, TASK.score, TASK.demo)) and TASK.score_over == "end"
    assert "orange key" in TASK.docs and "clockwise" not in TASK.docs  # the docs give no direction away
    assert T.LOCKBOX.total_mass == pytest.approx(0.70)


def test_the_submitted_lock_unlocks_clockwise_and_the_lid_slides_toward_minus_y():
    box = T.LOCKBOX
    assert box.params.reversed and box.key_limits_deg() == (-95.0, 45.0)  # joint angle is counter-clockwise +
    assert box.lid_limits_m() == (-0.10, 0.0)
    conventional = type(box)(**{**box.__dict__, "params": LockParams(reversed=False)})
    assert conventional.key_limits_deg() == (-45.0, 95.0)


def test_one_handed_turning_or_pulling_moves_the_loose_box():
    """Torque balance (README): the box's friction torque about its centre (feet at the corners of 18 x 14 cm) and
    its sliding friction are below the lock's running torque and the lid's rail friction."""
    box = T.LOCKBOX
    weight = box.total_mass * 9.81
    r_feet = math.hypot(0.09, 0.07)
    box_torque = box.feet_friction * weight * r_feet  # ~0.235 N m
    box_slide = box.feet_friction * weight  # ~2.06 N
    assert min(resistance_nm(a) for a in np.arange(6, 95, 0.5)) > box_torque  # past the first few degrees, always
    assert max(resistance_nm(a) for a in range(96)) > 1.6 * box_torque
    assert P.lid_friction_n > 1.4 * box_slide
    assert P.break_nm > 2.5 * box_torque  # an unheld box spins long before the pin shears


# -- resistance curve --------------------------------------------------------------------------------------------------


def test_the_resistance_curve_has_a_breakaway_peak_then_steady_travel():
    assert resistance_nm(0) == pytest.approx(0.20) and resistance_nm(6) == pytest.approx(0.40)
    assert resistance_nm(10.5) == pytest.approx(0.35) and resistance_nm(40) == pytest.approx(0.30)
    assert resistance_nm(90) == pytest.approx(0.40) and resistance_nm(-5) == resistance_nm(0)
    assert max(resistance_nm(a) for a in range(0, 96)) <= 0.7 * P.break_nm  # unlocking stays clear of the pin


def test_friction_holds_the_key_and_slips_at_the_curve():
    lock = Lock()
    tau, _ = lock.step(0.2, 0.0)  # a small push sticks: the friction spring pushes back
    assert tau == pytest.approx(-P.stick_stiffness * math.radians(0.2))
    tau, _ = turn(lock, np.linspace(0.2, 40, 50))  # turned well past the peak, it slips at the curve's value
    assert tau == pytest.approx(-resistance_nm(40))
    tau, _ = lock.step(40.0, 0.0)  # and stays where it was left
    assert abs(tau) <= resistance_nm(40) + 1e-9


# -- unlocking, the lid catch and the lid's friction -------------------------------------------------------------------


def test_the_catch_holds_a_locked_lid_and_lets_go_past_75_degrees():
    lock = Lock(NOFRIC)
    _, force = lock.step(0.0, 0.004)
    assert force == pytest.approx(-P.catch_stiffness * 0.004) and not lock.unlocked
    _, force = turn(lock, np.linspace(0, 74, 40), lid_m=0.004)
    assert force < 0 and not lock.unlocked
    _, force = turn(lock, [75.0], lid_m=0.004)
    assert force == 0.0 and lock.unlocked and not lock.catch
    _, force = turn(lock, [75.0], lid_m=0.05)
    assert force == 0.0 and not lock.pried


def test_an_unlocked_lid_slides_against_3_newtons_of_rail_friction():
    lock = Lock()
    turn(lock, np.linspace(0, 80, 40))
    _, force = turn(lock, [80.0] * 2, lid_m=0.0005)  # sticks: the pre-slip spring
    assert force == pytest.approx(-P.lid_stick_stiffness * 0.0005)
    _, force = turn(lock, [80.0], lid_m=0.03)
    assert force == pytest.approx(-P.lid_friction_n)


def test_turning_back_with_the_lid_shut_re_engages_the_catch():
    lock = Lock(NOFRIC)
    turn(lock, np.linspace(0, 80, 40))
    _, force = turn(lock, np.linspace(80, 10, 40), lid_m=0.0)
    assert lock.catch and force == 0.0 and lock.unlocked  # latched "unlocked once", but locked again now
    turn(lock, [10.0], lid_m=0.002)
    assert lock.catch


def test_pulling_a_locked_lid_past_a_centimetre_counts_as_pried():
    lock = Lock()
    turn(lock, [0.0], lid_m=0.009)
    assert not lock.pried
    turn(lock, [0.0], lid_m=0.011)
    assert lock.pried


# -- the wrong way: stop and shear pin ---------------------------------------------------------------------------------


def test_a_firm_push_against_the_stop_does_not_damage_the_key():
    lock = Lock()
    tau, _ = turn(lock, [stop_angle(0.5)] * 100)
    assert not lock.damaged and tau > 0.5 - 1e-9 and lock.peak_stop_nm == pytest.approx(0.5)


def test_forcing_past_the_pin_strength_for_three_physics_steps_breaks_the_key():
    lock = Lock()
    turn(lock, [stop_angle(0.7)] * 2 + [0.0])  # a two-step spike is not enough
    assert not lock.damaged
    turn(lock, [stop_angle(0.7)] * 3)
    assert lock.damaged and lock.peak_stop_nm == pytest.approx(0.7)


def test_a_broken_key_spins_freely_never_unlocks_and_the_latches_reset():
    lock = Lock()
    turn(lock, [stop_angle(1.0)] * 3)
    tau, force = turn(lock, np.linspace(-10, 90, 50), lid_m=0.0)
    assert lock.damaged and tau == 0.0 and not lock.unlocked and lock.catch and force == 0.0
    lock.step(-30.0, 0.0)
    lock.track_box(25.0, 0.01)
    lock.track_box(5.0, 0.0)
    assert lock.damaged and lock.max_tilt_deg == 25.0 and lock.max_drop_m == 0.01
    lock.reset()
    assert not (lock.damaged or lock.unlocked or lock.pried) and lock.catch
    assert lock.peak_stop_nm == lock.max_tilt_deg == lock.max_drop_m == 0.0


def test_the_conventional_lock_is_the_mirror_image():
    from lockkit import _sign

    assert _sign(LockParams()) == -1.0 and _sign(LockParams(reversed=False)) == 1.0


# -- scoring -----------------------------------------------------------------------------------------------------------

OPEN = {"key_deg": 92.0, "unlocked": True, "lid_m": 0.095}


@pytest.mark.parametrize(
    "kwargs, expected",
    [
        ({}, 0.0),
        ({"key_deg": 29.0}, 0.0),
        ({"key_deg": 30.0}, 0.25),
        ({"key_deg": 74.9}, 0.25),
        ({"key_deg": 75.0, "unlocked": True}, 0.5),
        ({"key_deg": 92.0, "unlocked": True, "lid_m": 0.019}, 0.5),
        ({"key_deg": 92.0, "unlocked": True, "lid_m": 0.02}, 0.75),
        ({"key_deg": 92.0, "unlocked": True, "lid_m": 0.08}, 1.0),
        ({**OPEN, "key_deg": 10.0}, 1.0),  # key turned back after opening: still open
        ({"key_deg": 10.0, "unlocked": True}, 0.0),  # unlocked once, then locked again with the lid shut
        ({**OPEN, "damaged": True}, 0.0),  # broke the key on the way
        ({"key_deg": -40.0, "damaged": True}, 0.0),
        ({"lid_m": 0.03, "pried": True}, 0.0),  # pried, never unlocked
        ({"lid_m": 0.03}, 0.0),  # lid moved without unlocking (cannot happen without prying): no credit
        # the box
        ({**OPEN, "shift": (0.02, -0.02)}, 1.0),  # 2.8 cm: inside the outline margin
        ({**OPEN, "shift": (0.0, -0.031)}, 0.5),  # slid 3.1 cm: unsteadied, capped
        ({**OPEN, "yaw": -16.0}, 0.5),  # spun 16 degrees: capped
        ({**OPEN, "yaw": 14.0}, 1.0),
        ({"key_deg": 30.0, "yaw": 40.0}, 0.25),  # the cap never raises a score
        ({**OPEN, "max_tilt_deg": 21.0}, 0.0),  # tipped at some point
        ({**OPEN, "max_drop_m": 0.06}, 0.0),  # fell off the table
        ({**OPEN, "max_tilt_deg": 19.0, "max_drop_m": 0.01}, 1.0),
    ],
)
def test_score_covers_every_outcome(kwargs, expected):
    objects = state(**kwargs)
    assert TASK.score(objects) == expected
    assert TASK.success(objects) == (expected == 1.0)


# -- demo --------------------------------------------------------------------------------------------------------------


def test_the_demo_holds_the_box_and_probes_the_usual_way_gently_before_turning():
    plan = TASK.demo()
    assert set(plan) == {"left", "right"}
    right, left = plan["right"], plan["left"]
    yaws = [math.degrees(w.yaw) for w in right]
    assert max(yaws) == pytest.approx(94.0) and min(yaws) == pytest.approx(-2.0)  # grip at 90: CCW probe, CW turn
    assert yaws.index(max(yaws)) < yaws.index(min(yaws))
    assert P.unlock_deg < 92.0 < P.open_end_deg
    slide = next(i for i, w in enumerate(right) if w.label == "slide the lid open")
    assert right[slide].pos[1] - right[slide - 1].pos[1] == pytest.approx(-0.095)  # the lid slides 9.5 cm
    t_grip_fin = sum(w.duration for w in left[:3])
    t_grip_key = sum(w.duration for w in right[:4])
    t_left_lets_go = sum(w.duration for w in left[:4])
    assert t_grip_fin < t_grip_key and t_left_lets_go == pytest.approx(sum(w.duration for w in right[:-1]))
    assert all(w.finger == left[2].finger for w in left[2:4])  # closed from gripping the fin until the end


def test_the_demo_is_reachable_within_the_joint_limits_and_inside_the_run_limit():
    from yam_arena import ik, station
    from yam_arena.constants import CONTROL_HZ
    from yam_arena.demo import ArmPlan

    plan = TASK.demo()
    for side in ("left", "right"):
        arm = ArmPlan(plan[side], station.BASES[side])
        n = round(arm.duration * CONTROL_HZ) + 5
        actions = np.array([arm.action7((k + 1) / CONTROL_HZ)[0] for k in range(n)])
        assert arm.worst_ik_mm < 0.5 and arm.worst_ik_deg < 0.1, side
        assert np.all(actions[:, :6] >= ik.LOWER) and np.all(actions[:, :6] <= ik.UPPER), side
        assert arm.duration * CONTROL_HZ + 20 < 600  # the demo command's --max-steps (plus the 1 s success hold)
        if side == "right":
            assert np.ptp(actions[:, 5]) > math.radians(90)  # the turn is done with wrist roll (joint6)


def test_the_force_wrong_contrast_turns_60_degrees_the_usual_way(monkeypatch):
    monkeypatch.setenv("DEMO_FORCE_WRONG", "1")
    yaws = [math.degrees(w.yaw) for w in TASK.demo()["right"]]
    assert max(yaws) == pytest.approx(150.0)
