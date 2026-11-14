"""Leg mechanics, forces, scoring and demo geometry for the cotter-pin task. No Isaac Sim needed.

    pytest tasks/cotter-pin-clevis/test_cotter_pin_clevis.py     (in the yam_arena environment)
"""

import math
import pathlib
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from cotterkit import CotterState, Leg, LegParams, leg_tip, semicircle_plastic_modulus  # noqa: E402
from yam_arena.task_file import load  # noqa: E402

TASK = load(HERE / "task.py")
T = TASK.module
P = LegParams()
YAM_RATED_N = 15.0  # 1.5 kg rated payload
YAM_GRIP_N = 50.0


def state(long=0.0, short=0.0, rebent=(False, False), gap=0.1, speed=0.0):
    pos = np.array([*T.PIN_XY, T.TOP + T.H])
    return {"cotter": CotterState(
        pos, np.array([0.0, 0.0, 0.0, 1.0]), np.zeros(3), np.zeros(3), pos - 0.03, pos + 0.03,
        bend_deg={"long": long, "short": short}, set_deg={"long": long, "short": short},
        rebent={"long": rebent[0], "short": rebent[1]}, gap_m=gap, leg_speed=speed)}


def bend(leg, angles_deg):
    for a in angles_deg:
        leg.step(math.radians(a))
    return math.degrees(leg.set)


# -- real-hardware numbers -------------------------------------------------------------------------------------------


def test_plastic_modulus_of_a_half_round_matches_the_closed_form():
    r = 0.0009
    # closed form for a semicircle about its plastic neutral axis (parallel to the flat): ~0.3540 r^3
    assert semicircle_plastic_modulus(r) == pytest.approx(0.354 * r**3, rel=0.01)


def test_the_yield_moment_and_forces_are_those_of_a_2_mm_steel_cotter_and_within_the_yam():
    assert P.yield_nm == pytest.approx(0.077, abs=0.002)  # 300 MPa x 0.258 mm^3
    worst = P.force_n(0.0125, math.pi / 2)  # short leg pushed 12.5 mm out, after 90 deg of hardening
    assert 6.0 < P.force_n(0.0125) < worst < 8.0 < YAM_RATED_N
    assert P.force_n(0.0195, math.pi / 2) < 5.0  # the long leg, pushed on its extension
    # even the hardest plausible wire (500 MPa) stays under the rated payload at these levers
    hard = LegParams(yield_pa=500e6)
    assert hard.force_n(0.0125, math.pi / 2) < YAM_RATED_N < YAM_GRIP_N


def test_springback_is_under_two_degrees():
    assert math.degrees(P.yield_nm / P.stiffness) < 2.0


# -- elastic-plastic leg -------------------------------------------------------------------------------------------


def test_a_push_under_the_yield_moment_springs_back_fully():
    leg = Leg(P)
    small = math.degrees(0.9 * P.yield_nm / P.stiffness)
    assert bend(leg, np.linspace(0, small, 50)) == 0.0
    assert bend(leg, np.linspace(small, 0, 50)) == 0.0 and not leg.rebent


def test_a_leg_pushed_past_yield_keeps_its_bend_less_springback():
    leg = Leg(P)
    s = bend(leg, np.linspace(0, 90, 400))
    back = math.degrees(P.moment_nm(leg.peak) / P.stiffness)
    assert s == pytest.approx(90 - back, abs=0.05)
    assert bend(leg, np.linspace(90, s, 50)) == pytest.approx(s)  # released: stays set
    assert leg.peak_moment == pytest.approx(P.moment_nm(leg.peak), rel=1e-3)


def test_straightening_a_bent_leg_latches_rebent():
    leg = Leg(P)
    s = bend(leg, np.linspace(0, 70, 300))
    bend(leg, np.linspace(s, s - 10, 50))
    assert not leg.rebent  # 10 deg back (plus springback) is not yet a re-bend
    bend(leg, np.linspace(s - 10, 20, 200))
    assert leg.rebent
    bend(leg, np.linspace(20, 90, 200))
    assert leg.rebent  # bending it out again does not undo it
    leg.reset()
    assert not leg.rebent and leg.set == 0.0


def test_a_slightly_spread_leg_can_be_pushed_back_without_a_rebend_latch():
    leg = Leg(P)
    bend(leg, np.linspace(0, 25, 100))
    bend(leg, np.linspace(25, 0, 100))
    assert not leg.rebent and math.degrees(leg.set) < 2.0  # straight again, less springback


# -- scoring -------------------------------------------------------------------------------------------------------


def test_score_ladder():
    assert T.score(state()) == 0.0
    assert T.score(state(long=59.0, short=59.0)) == 0.0
    assert T.score(state(long=95.0)) == 0.25
    assert T.score(state(short=61.0)) == 0.25
    assert T.score(state(long=95.0, short=95.0, gap=0.005)) == 0.5  # still touching
    assert T.score(state(long=95.0, short=95.0, speed=20.0)) == 0.5  # moving
    assert T.score(state(long=95.0, short=95.0)) == 0.75
    assert T.success(state(long=95.0, short=95.0)) and not T.success(state(long=95.0, short=95.0, gap=0.01))


def test_a_rebent_leg_zeroes_the_trial_whatever_the_end_state():
    assert T.score(state(long=95.0, short=95.0, rebent=(True, False))) == 0.0
    assert T.score(state(long=95.0, short=95.0, rebent=(False, True))) == 0.0


# -- geometry and demo -----------------------------------------------------------------------------------------------


def test_cotter_is_iso_1234_2x25_through_an_8_mm_pin():
    rig = T.RIG
    assert rig.root_x == 0.004 and rig.short_len + 2 * rig.root_x == pytest.approx(0.025)
    assert rig.long_len - rig.short_len == pytest.approx(0.004)  # extended prong b = 4 mm
    assert 2 * rig.leg_t == pytest.approx(P.wire_d) == rig.leg_w
    assert rig.leg_mass("long") + rig.leg_mass("short") == pytest.approx(0.38e-3, abs=0.03e-3)


def test_legs_open_away_from_each_other():
    rig = T.RIG
    assert leg_tip(rig, "long", math.radians(90))[1] > 0.02
    assert leg_tip(rig, "short", math.radians(90))[1] < -0.016


def test_the_first_push_lands_past_the_short_legs_end_on_the_long_legs_extension():
    plan = T.demo()["right"]
    down = next(w for w in plan if "down astride" in w.label)
    x = down.pos[0] - T.PIN_XY[0]
    near, far = x - T.HALF_W, x + T.HALF_W
    assert T.RIG.root_x + T.RIG.short_len < near < T.RIG.root_x + T.RIG.long_len < far


def test_the_demo_fits_the_run_limit():
    plan = T.demo()["right"]
    total = sum(w.duration for w in plan) + T.SUCCESS_HOLD_S
    assert total * 20 < 900
