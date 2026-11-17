"""Unit tests of the punch-and-seat rule and every scoring outcome (no Isaac Sim): python -m pytest"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from dripkit import DripBed, in_block, new_line, score_line  # noqa: E402

BED = DripBed("drip_bed", pos=(0.33, -0.02, 0.0), size=(0.30, 0.64, 0.012), table_z=0.0)
UP = np.array([0.0, 0.0, 1.0])
DT = 1 / 240


def line():
    ln = new_line(BED, 0)
    return ln


def top(ln, y=None):
    return np.array([ln.x, ln.mark_y if y is None else y, ln.z + ln.radius])


def lean(deg):
    a = math.radians(deg)
    return np.array([math.sin(a), 0.0, math.cos(a)])


def press_punch(ln, push, axis=UP, y=None, secs=0.3):
    for _ in range(int(secs / DT)):
        ln.punch_step(top(ln, y), axis, push, DT)


def press_emitter(ln, push, axis=UP, at=None, secs=0.3):
    at = top(ln) if at is None else at
    for _ in range(int(secs / DT)):
        ln.emitter_step(at, axis, push, DT)


def test_full_job_scores_one():
    ln = line()
    press_punch(ln, 30.0)
    assert len(ln.holes) == 1 and not ln.holes[0].torn
    press_emitter(ln, 18.0)
    assert ln.seated == 0
    assert score_line(ln, True) == 1.0
    assert score_line(ln, False) == 0.9


def test_light_push_makes_no_hole():
    ln = line()
    press_punch(ln, 0.6 * ln.law.punch_n)
    assert ln.holes == []
    assert score_line(ln, True) == 0.0


def test_contact_spike_is_not_a_punch():
    ln = line()
    press_punch(ln, 200.0, secs=0.02)
    assert ln.holes == []


def test_punch_above_the_surface_does_nothing():
    ln = line()
    for _ in range(100):
        ln.punch_step(top(ln) + 0.01 * UP, UP, 40.0, DT)
    assert ln.holes == []


def test_angled_punch_tears_and_scores_zero():
    ln = line()
    press_punch(ln, 30.0, axis=lean(20))
    assert ln.holes[0].torn
    press_emitter(ln, 18.0)
    assert score_line(ln, True) == 0.0


def test_dripper_without_a_hole_does_not_seat():
    ln = line()
    press_emitter(ln, 40.0)
    assert ln.seated is None and score_line(ln, True) == 0.0


def test_light_dripper_push_does_not_seat():
    ln = line()
    press_punch(ln, 30.0)
    press_emitter(ln, 8.0)
    assert ln.seated is None and score_line(ln, True) == 0.4


def test_barb_beside_the_hole_does_not_seat():
    ln = line()
    press_punch(ln, 30.0)
    press_emitter(ln, 20.0, at=top(ln) + np.array([0.0, 0.003, 0.0]))
    assert ln.seated is None


def test_crooked_dripper_tears_the_hole():
    ln = line()
    press_punch(ln, 30.0)
    press_emitter(ln, 20.0, axis=lean(25))
    assert ln.seated is None and ln.holes[0].torn
    press_emitter(ln, 20.0)  # straightening it afterwards does not undo the tear
    assert score_line(ln, True) == 0.0


def test_pressing_the_same_hole_again_is_one_hole():
    ln = line()
    press_punch(ln, 30.0)
    press_punch(ln, 30.0, y=ln.mark_y + 0.002)
    assert len(ln.holes) == 1


def test_second_hole_caps_at_point_two():
    ln = line()
    press_punch(ln, 30.0)
    press_punch(ln, 30.0, y=ln.mark_y + 0.03)
    press_emitter(ln, 18.0)
    assert ln.seated == 0 and ln.leak
    assert score_line(ln, True) == 0.2


def test_hole_off_the_mark_scores_zero():
    ln = line()
    y = ln.mark_y + 0.02
    press_punch(ln, 30.0, y=y)
    press_emitter(ln, 18.0, at=top(ln, y))
    assert ln.seated == 0 and score_line(ln, True) == 0.0


def test_punch_credit_needs_the_dripper():
    ln = line()
    press_punch(ln, 30.0)
    assert score_line(ln, True) == 0.4


def test_seeds_move_the_mark_and_the_wall():
    marks = {round(new_line(BED, s).mark_y, 4) for s in (0, 4, 10)}
    assert len(marks) == 3
    for s in range(20):
        ln = new_line(BED, s)
        assert abs(ln.mark_y - BED.mark_y) <= BED.mark_jitter
        assert 22.4 <= ln.law.punch_n <= 27.6


def test_punch_home():
    tip = BED.punch_rest()
    assert in_block(BED, tip, UP)
    assert not in_block(BED, tip + np.array([0.0, 0.0, 0.012]), UP)
    assert not in_block(BED, tip + np.array([0.01, 0.0, 0.0]), UP)


def test_barb_in_the_mouth_stays_while_pressed():
    ln = line()
    press_punch(ln, 30.0)
    press_emitter(ln, 3.0, secs=0.05)  # lands on the hole
    assert ln.mouth == 0
    press_emitter(ln, 18.0, at=top(ln) + np.array([0.0, 0.003, 0.0]))  # slid 3 mm while pushing
    assert ln.seated == 0


def test_barb_lifted_off_loses_the_mouth():
    ln = line()
    press_punch(ln, 30.0)
    press_emitter(ln, 3.0, secs=0.05)
    press_emitter(ln, 0.0, at=top(ln) + np.array([0.0, 0.0, 0.01]), secs=0.05)
    assert ln.mouth is None
    press_emitter(ln, 18.0, at=top(ln) + np.array([0.0, 0.003, 0.0]))
    assert ln.seated is None
