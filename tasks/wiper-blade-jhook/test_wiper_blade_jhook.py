"""Unit tests for the hook/latch state machine and the arm rules (no Isaac Sim needed):
pytest test_wiper_blade_jhook.py  (with yam_arena importable)"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from hookmodel import (LOW_DEG, OFF_M, SLIDE_LO, ArmMonitor, HookMonitor, Layout, R_G, rot_x)  # noqa: E402

L = Layout((0.2, 0.0, 0.78), (0.11, -0.30, 0.049))


def test_seat_follows_the_arm():
    so0, sr0 = L.seat(0.0)
    assert abs((np.linalg.inv(R_G) @ (so0 - np.asarray(L.origin)))[2]) < 1e-9  # on the glass at rest
    so, sr = L.seat(math.radians(14))
    rel, ang = L.rel(math.radians(14), so, sr)
    assert np.allclose(rel, 0) and ang < 1e-6
    rise = (np.linalg.inv(R_G) @ (so - so0))[2]
    assert 0.07 < rise < 0.10  # a 14 degree lift raises the seat 8 cm off the glass


def test_removal_needs_the_tab_then_the_slide_then_a_hand():
    m = HookMonitor(latched=True)
    assert m.step((0, 0, 0), 0.0, False, True) == []  # held, pulled, tab up: stays latched
    assert ("slide", True) in m.step((0, 0, 0), 0.0, True, False) and m.state == "slide"
    m.step((-0.010, 0, 0), 0.0, True, True)
    assert m.state == "slide"  # nose still under the bend
    m.step((OFF_M - 0.001, 0, 0), 0.0, False, False)
    assert m.state == "slide"  # clear, but hanging on the hook until a hand takes it
    m.step((OFF_M - 0.001, 0, 0), 0.0, False, True)
    assert m.state == "free" and m.released and not m.broken


def test_forcing_a_latched_blade_breaks_it():
    m = HookMonitor(latched=True)
    m.step((-0.008, 0, 0), 0.0, False, True)
    assert m.broken and m.state == "free"


def test_threading_order():
    m = HookMonitor(latched=False)
    m.step((-0.012, 0.0, -0.02), 0.0, False, True)  # below the hook
    assert m.state == "free"
    m.step((-0.012, 0.0, 0.0), 10.0, False, True)  # in place but turned 10 degrees
    assert m.state == "free"
    m2 = HookMonitor(latched=False)  # from the tip side, through the bend
    m2.step((0.01, 0.0, 0.0), 0.0, False, True)
    m2.step((-0.004, 0.0, 0.0), 0.0, False, True)
    m2.step((-0.010, 0.0, 0.0), 0.0, False, True)
    assert m2.state == "free"
    m.step((-0.012, 0.0, 0.0), 2.0, False, True)
    assert m.state == "slide"
    cmds = m.step((-0.003, 0.0, 0.0), 1.0, False, True)
    assert ("click", True) in cmds  # the latch tab rides over the hook's end
    m.step((-0.003, 0.0, 0.0), 1.0, True, True)
    m.step((0.0, 0.0, 0.0), 1.0, True, True)
    assert m.state == "slide"  # tab held down: it cannot latch
    m.step((0.0, 0.0, 0.0), 1.0, False, True)
    assert m.state == "latched"


def test_slide_limits_are_consistent():
    assert SLIDE_LO < OFF_M < 0


def test_arm_slam_and_bare_hook():
    a = ArmMonitor()
    a.step(math.radians(10), -10.0, True)  # fast but far from the glass
    a.step(math.radians(1), -2.0, True)  # 0.72 m/s at the glass: fine
    assert not a.failed
    a.step(math.radians(1), -4.0, True)  # 1.44 m/s at the glass
    assert a.slammed
    b = ArmMonitor()
    b.step(math.radians(LOW_DEG + 0.2), -0.1, False)
    assert b.bare_hit and b.failed == "bare hook on the glass"
    c = ArmMonitor()
    c.step(math.radians(LOW_DEG + 0.2), -0.1, True)
    assert not c.failed


def test_rot_x_lifts_the_arm_off_the_glass():
    assert np.allclose(rot_x(math.radians(90)) @ [0, 1, 0], [0, 0, 1])
