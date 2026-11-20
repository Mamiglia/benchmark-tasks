"""Unit tests for the plastic rule and the scorer (no Isaac Sim): pytest test_heat_set_inserts.py"""

import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import task  # noqa: E402
from insertkit import Insert, InsertState, Law, in_stand, insert_credit, seeded  # noqa: E402

LAW = Law()
DT = 1 / 240
UP = (0.0, 0.0, 1.0)


def press(ins, force, seconds, tilt=0.0, engaged=True, law=LAW):
    for _ in range(int(round(seconds / DT))):
        ins.step(law, force, engaged, tilt, UP, DT)
    return ins


def fresh():
    return Insert(travel=0.0045, bottom=0.0055)


def test_nothing_sinks_before_it_is_hot():
    ins = press(fresh(), 8.0, LAW.heat_s - 0.05)
    assert ins.depth == 0.0 and not ins.sank


def test_light_touch_never_sinks():
    ins = press(fresh(), LAW.f_min - 0.1, 20.0)
    assert ins.depth == 0.0


def test_steady_push_sinks_at_the_rate():
    ins = press(fresh(), 5.0, LAW.heat_s)
    ins = press(ins, 5.0, 1.0)
    assert ins.depth == pytest.approx(LAW.k * (5.0 - LAW.f_min) * 1.0, rel=0.02)
    assert not ins.forced and not ins.crooked


def test_flush_takes_seconds_at_a_moderate_push():
    ins = press(fresh(), 6.0, LAW.heat_s)
    t = 0.0
    while ins.proud > 0.0:
        press(ins, 6.0, DT)
        t += DT
    assert 2.0 < t < 6.0  # about 3.3 s at 6 N
    assert ins.lip and insert_credit(ins) == 0.10  # flush by the iron alone: lip not pressed flat


def test_too_hard_is_forced_and_scores_nothing():
    ins = press(fresh(), 20.0, LAW.heat_s + 0.5)
    assert ins.forced and insert_credit(ins) == 0.0
    depth = ins.depth
    press(ins, 5.0, 2.0)
    assert ins.depth == depth  # a forced insert is done


def test_brief_spike_is_not_forced():
    ins = press(fresh(), 5.0, LAW.heat_s + 0.2)
    press(ins, 20.0, LAW.over_s - 0.05)
    press(ins, 5.0, 0.5)
    assert not ins.forced


def test_leaning_iron_sets_it_crooked():
    ins = press(fresh(), 6.0, LAW.heat_s)
    press(ins, 6.0, 0.2, tilt=LAW.tilt_deg + 1)
    assert ins.crooked and insert_credit(ins) == 0.0


def test_leaning_before_it_sinks_costs_nothing():
    ins = press(fresh(), 6.0, LAW.heat_s - 0.1, tilt=10.0)
    assert not ins.crooked


def test_cools_off_the_iron():
    ins = press(fresh(), 6.0, LAW.heat_s - 0.1)
    press(ins, 0.0, LAW.cool_s + 0.1, engaged=False)
    press(ins, 6.0, LAW.heat_s - 0.2)
    assert ins.depth == 0.0


def test_off_axis_pin_does_not_sink():
    ins = press(fresh(), 6.0, 10.0, engaged=False)
    assert ins.depth == 0.0


def test_stops_at_the_bottom():
    ins = press(fresh(), 10.0, 20.0)
    assert ins.depth == pytest.approx(ins.bottom)
    assert ins.proud == pytest.approx(-0.001)
    assert insert_credit(ins) == 0.10  # sunk 1 mm: seated, not flush


@pytest.mark.parametrize("proud,credit", [(0.0004, 0.25), (-0.0004, 0.25), (0.0006, 0.10), (0.0025, 0.10),
                                          (0.0026, 0.0), (-0.0008, 0.10)])
def test_credit_bands(proud, credit):
    ins = Insert(travel=0.0045, bottom=0.0055, depth=0.0045 - proud, sank=True)
    assert insert_credit(ins) == credit


def test_marked_insert_scores_nothing():
    ins = Insert(travel=0.0045, bottom=0.0055, depth=0.0045, sank=True, marked=True)
    assert insert_credit(ins) == 0.0


def _state(inserts, **kw):
    base = dict(pos=np.zeros(3), quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                low=np.zeros(3), high=np.zeros(3), axis=np.array(UP), inserts=tuple(inserts), held=None,
                in_stand=True, burned=False, dropped=False)
    base.update(kw)
    return {"insert_rig": InsertState(**base)}


def flush():
    return Insert(travel=0.0045, bottom=0.0055, depth=0.0043, sank=True)


def test_full_score_and_success():
    s = _state([flush() for _ in range(4)])
    assert task.score(s) == 1.0 and task.success(s)


def test_partial_credit():
    s = _state([flush(), flush(), Insert(travel=0.0045, bottom=0.0055, depth=0.003, sank=True), fresh()])
    assert task.score(s) == pytest.approx(0.6)


@pytest.mark.parametrize("kw", [dict(burned=True), dict(dropped=True), dict(in_stand=False),
                                dict(held="right_robot")])
def test_whole_trial_zero(kw):
    assert task.score(_state([flush() for _ in range(4)], **kw)) == 0.0


def test_in_stand_geometry():
    r = task.RIG
    a = r.tube_axis
    tip = r.tube_bottom + 0.002 * -a + r.rest_drop * r.tube_down
    assert in_stand(r, tip, -a)
    assert not in_stand(r, tip - 0.15 * a, -a)  # drawn out
    assert not in_stand(r, tip, np.array(UP))  # wrong way


def test_seeds_vary_and_are_repeatable():
    assert seeded(task.RIG, 4) == seeded(task.RIG, 4)
    assert seeded(task.RIG, 0) != seeded(task.RIG, 10)
    proud, soft = seeded(task.RIG, 10)
    assert all(0.0042 <= p <= 0.0048 for p in proud) and 0.75 <= soft <= 1.25


def iron_to(proud, force=6.0):
    """Heat, then sink with the iron to ``proud`` (m) above the face; the iron then leaves."""
    ins = press(fresh(), force, LAW.heat_s)
    while ins.proud > proud:
        press(ins, force, DT)
    return ins


def block(ins, force, seconds, tilt=0.0, wait=0.0):
    press(ins, 0.0, wait, engaged=False)
    for _ in range(int(round(seconds / DT))):
        ins.step(LAW, 0.0, False, 0.0, UP, DT, force, tilt, UP)
    return ins


def test_block_while_soft_seats_flush_and_square():
    ins = block(iron_to(0.0008), 6.0, 1.5, wait=1.0)
    assert ins.proud == pytest.approx(0.0, abs=1e-9) and not ins.lip
    assert insert_credit(ins) == 0.25


def test_block_never_drives_past_flush():
    ins = block(iron_to(0.0008), 12.0, 2.0)
    assert ins.proud >= 0.0 and insert_credit(ins) == 0.25


def test_block_after_the_plastic_cooled_does_nothing():
    ins = block(iron_to(0.0008), 6.0, 1.5, wait=LAW.cool_s + 0.1)
    assert ins.proud == pytest.approx(0.0008, abs=1e-5) and ins.lip
    assert insert_credit(ins) == 0.10


def test_block_too_far_from_flush_does_nothing():
    ins = block(iron_to(0.0025), 6.0, 1.5, wait=0.5)
    assert ins.proud > LAW.flat_reach and ins.lip and insert_credit(ins) == 0.10


def test_leaning_block_sets_it_crooked():
    ins = block(iron_to(0.0008), 6.0, 1.0, tilt=LAW.tilt_deg + 2, wait=0.5)
    assert ins.crooked and insert_credit(ins) == 0.0


def test_block_pressed_hard_forces_it():
    ins = block(iron_to(0.0015), 20.0, 0.6, wait=0.2)
    assert ins.forced and insert_credit(ins) == 0.0


def test_block_on_an_unheated_insert_does_nothing():
    ins = block(fresh(), 6.0, 2.0)
    assert ins.depth == 0.0 and not ins.sank


def test_iron_only_flush_set_scores_partial():
    s = _state([Insert(travel=0.0045, bottom=0.0055, depth=0.0043, sank=True, lip=True) for _ in range(4)])
    assert task.score(s) == pytest.approx(0.4) and not task.success(s)
