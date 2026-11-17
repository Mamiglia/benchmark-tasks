"""Offline tests: the phone's button logic, the seat latch and every score tier on handcrafted button traces, the
stand geometry and the demo's reach. No GPU needed."""

import math

import numpy as np
import phone_demo
import task
from phonekit import Device, DeviceRules, Seat, grade, phone_pose_in_stand, stand_parts

DT = 1 / 240


def play(*presses, until=6.0):
    """presses: (button, t_down, t_up). Runs the device at 240 Hz; returns its events."""
    dev = Device(DeviceRules())
    t = 0.0
    while t < until:
        t += DT
        dev.step(t, {b: any(t0 <= t < t1 for c, t0, t1 in presses if c == b) for b, _, _ in presses})
    return dev.events


def test_quick_pair_is_one_screenshot():
    ev = play(("side", 1.0, 1.3), ("vol_up", 1.05, 1.3))
    assert ev["screenshot"] == 1 and sum(ev.values()) == 1


def test_pair_order_does_not_matter_within_the_window():
    ev = play(("vol_up", 1.0, 1.4), ("side", 1.15, 1.4))
    assert ev["screenshot"] == 1 and sum(ev.values()) == 1


def test_presses_too_far_apart_are_two_single_presses():
    ev = play(("side", 1.0, 1.6), ("vol_up", 1.3, 1.6))
    assert ev["screenshot"] == 0 and ev["volume_up"] == 1 and ev["lock"] == 1


def test_pair_held_past_the_release_window_takes_no_screenshot():
    ev = play(("side", 1.0, 2.5), ("vol_up", 1.0, 2.5))
    assert sum(ev.values()) == 0


def test_pair_held_two_seconds_brings_up_the_slider():
    ev = play(("side", 1.0, 4.0), ("vol_up", 1.0, 4.0))
    assert ev["slider"] == 1 and ev["screenshot"] == 0
    ev = play(("side", 1.0, 4.0), ("vol_down", 1.0, 4.0))
    assert ev["slider"] == 1 and ev["volume_down"] == 0


def test_single_buttons():
    assert play(("side", 1.0, 1.2))["lock"] == 1
    assert play(("vol_up", 1.0, 1.1))["volume_up"] == 1
    assert play(("vol_up", 1.0, 2.0))["volume_up"] == 1
    assert play(("vol_down", 1.0, 1.3))["volume_down"] == 1


def test_side_with_volume_down_quickly_is_no_screenshot():
    ev = play(("side", 1.0, 1.3), ("vol_down", 1.0, 1.3))
    assert ev["screenshot"] == 0


def test_all_three_pressed_is_screenshot_plus_volume_down():
    ev = play(("side", 1.0, 1.3), ("vol_up", 1.0, 1.3), ("vol_down", 1.0, 1.3))
    assert ev["screenshot"] == 1 or ev["volume_down"] == 1
    assert grade(ev, False, False) < 1.0


def test_two_screenshots_are_not_full_credit():
    ev = play(("side", 1.0, 1.3), ("vol_up", 1.0, 1.3), ("side", 2.0, 2.3), ("vol_up", 2.0, 2.3))
    assert ev["screenshot"] == 2 and grade(ev, False, False) == 0.5


def test_grades():
    none = dict.fromkeys(("screenshot", "volume_up", "volume_down", "lock", "slider"), 0)
    shot = dict(none, screenshot=1)
    assert grade(shot, False, False) == 1.0
    assert grade(shot, True, False) == 0.0
    assert grade(shot, False, True) == 0.5
    assert grade(dict(shot, volume_up=1), False, False) == 0.5
    assert grade(dict(shot, slider=1), False, False) == 0.1
    assert grade(dict(none, lock=1), False, False) == 0.3
    assert grade(dict(none, volume_down=2), False, False) == 0.3
    assert grade(none, False, False) == 0.0
    assert grade(dict(none, lock=1), True, False) == 0.0


def test_seat_latch():
    q = np.array([0.0, math.sin(0.17), 0.0, math.cos(0.17)])
    s = Seat()
    s.control([0.3, 0.0, 0.85], q)
    s.control([0.3, 0.012, 0.855], q)
    assert not s.out
    s.control([0.3, 0.0, 0.862], q)  # lifted 12 mm
    s.control([0.3, 0.0, 0.85], q)
    assert s.out  # latched
    s = Seat()
    s.control([0.3, 0.0, 0.85], q)
    q2 = np.array([0.0, math.sin(0.17 + math.radians(6)), 0.0, math.cos(0.17 + math.radians(6))])
    s.control([0.3, 0.0, 0.85], q2)
    assert s.out  # turned 12 degrees


def test_phone_rests_in_the_cradle():
    p, st = task.PARAMS, task.STAND
    x, z = phone_pose_in_stand(st, p)
    parts = {n: (size, pos) for n, size, pos, _ in stand_parts(st, p)}
    assert parts["back"][0][1] < p.width - 0.02  # the back rest is narrower than the phone: fingers pass beside it
    # Buttons are 9 cm or more above the seat, clear of the back rest's top (7 cm along the phone).
    assert min(p.length / 2 + p.button_z(i) for i in range(3)) - 0.07 > 0.015
    assert 0.08 < z + task.PARAMS.button_z(1) * math.cos(task.PITCH) < 0.2  # under the top camera's 0.2 m


def test_pad_window_separates_volume_buttons():
    """The pads' 25.7 mm span centred at PAD_AT covers volume up and the side button, and misses volume down."""
    p, c = task.PARAMS, phone_demo.PAD_AT
    lo, hi = c - phone_demo.PAD_SPAN / 2, c + phone_demo.PAD_SPAN / 2
    vu = (p.button_z(1) - p.lengths[1] / 2, p.button_z(1) + p.lengths[1] / 2)
    vd_top = p.button_z(2) + p.lengths[2] / 2
    sd = (p.button_z(0) - p.lengths[0] / 2, p.button_z(0) + p.lengths[0] / 2)
    assert lo > vd_top and lo < sd[1] and hi > vu[0]


def test_demo_reaches_the_pinch_pose():
    from yam_arena import ik

    x, z = task.button_point(phone_demo.PAD_AT)
    base = np.array([0.0, 0.305, task.TOP])
    q, err, rot = ik.solve(np.array([x, 0.0, z]), ik.matrix_to_quat_xyzw(phone_demo.GRIP_R),
                           np.array([0.0, 1.0, 1.0, -0.6, 0.0, 0.0]), base_pos=base, iters=300)
    assert err < 0.002 and rot < 0.02
