"""Tests for h-shifter-reverse: the gate, the gear readout and every scoring outcome. No Isaac Sim needed."""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import shiftmodel as M  # noqa: E402

DT = 1 / 240


def run(path, seq=None, box=None):
    """Feed (u, v, press, seconds) holds through the readout and the sequence."""
    box, seq = box or M.Gearbox(), seq or M.Sequence()
    gear = None
    for u, v, press, secs in path:
        for _ in range(round(secs / DT)):
            gear = box.read(u, v, press)
            seq.feed(gear, DT)
    return seq, gear


N = (0.0, 0.0, 0.0, 0.3)
END = M.U


def g(gear, secs=1.2, press=0.0):
    plane, sign = {"1": (M.P, 1), "2": (M.P, -1), "3": (0.0, 1), "4": (0.0, -1), "5": (-M.P, 1),
                   "6": (-M.P, -1)}[gear]
    return [(0.0, plane, press, 0.1), (sign * END, plane, press, secs), (0.0, plane, press, 0.1)]


def full(rev_press=-0.006):
    return [N, *g("1"), *g("2"), *g("3"), N, (0.0, -M.P, rev_press, 0.2), (-END, -M.P, rev_press, 1.5)]


def test_gate_limits():
    (a0, a1), (b0, b1) = M.gate_limits(0.0, 0.01)  # between planes: neutral channel only
    assert abs(b1 * M.L - M.C) < 1e-9 and abs(b0 * M.L + M.C) < 1e-9
    (a0, a1), (b0, b1) = M.gate_limits(0.0, M.P)  # at a crossing: the slot is open
    assert abs(b1 * M.L - M.U) < 1e-9
    (a0, a1), _ = M.gate_limits(0.02, M.P)  # in the 1 slot: sideways only within it
    assert abs(-a1 * M.L - (M.P - M.C)) < 1e-9 and abs(-a0 * M.L - (M.P + M.C)) < 1e-9


def test_springs():
    assert M.side_force(M.P) < 0 < M.side_force(-M.P)  # back toward 3-4 from both sides
    assert abs(M.side_force(-M.P)) > abs(M.side_force(M.P))
    assert M.detent_force(0.25 * M.U) < 0 < M.detent_force(0.6 * M.U)  # toward neutral, then into the gear
    assert M.press_force(-0.005) > M.press_force(0.0) > 0


def test_readout():
    box = M.Gearbox()
    assert box.read(0.0, 0.0, 0.0) is None
    assert box.read(M.U, M.P, 0.0) == "1" and box.read(-M.U, 0.0, 0.0) == "4"
    assert box.read(0.8 * M.U, M.P, 0.0) is None  # not yet 90 % in


def test_reverse_needs_push_on_entry():
    box = M.Gearbox()
    box.read(0.0, -M.P, -0.006)
    assert box.read(-M.U, -M.P, -0.006) == "R"
    assert box.read(-M.U, -M.P, 0.0) == "R"  # stays R once in, knob let up
    box = M.Gearbox()
    assert box.read(-M.U, -M.P, 0.0) == "6"
    assert box.read(-M.U, -M.P, -0.006) == "6"  # pushing down once in the slot changes nothing
    box.read(0.0, -M.P, 0.0)
    assert box.read(-M.U, -M.P, -0.004) == "6"  # 4 mm is not enough


def test_full_sequence_scores_one():
    seq, gear = run(full())
    assert seq.done == 4 and seq.wrong is None and gear == "R"
    assert seq.score(gear) == 1.0


def test_reverse_left_after_reaching_it():
    seq, gear = run([*full(), N])
    assert seq.done == 4 and seq.score(gear) == 0.75


def test_six_instead_of_reverse():
    seq, gear = run(full(rev_press=0.0))
    assert seq.wrong == "6" and seq.score(gear) == 0.75


def test_out_of_order_stops_credit():
    seq, gear = run([N, *g("1"), *g("3"), *g("2"), *g("3")])
    assert seq.done == 1 and seq.wrong == "3" and seq.score(gear) == 0.25
    seq, gear = run([N, *g("1"), *g("2"), *g("1"), *g("3")])  # back to an earlier gear
    assert seq.wrong == "1" and seq.score(gear) == 0.5


def test_reengaging_the_last_gear_is_harmless():
    seq, gear = run([N, *g("1"), *g("1"), *g("2"), *g("3")])
    assert seq.done == 3 and seq.wrong is None and seq.score(gear) == 0.75


def test_short_holds():
    seq, gear = run([N, *g("1", secs=0.6)])  # not held 1 s
    assert seq.done == 0 and seq.score(gear) == 0.0
    seq, gear = run([N, *g("1"), *g("4", secs=0.15), *g("2")])  # a 0.15 s brush of 4 is not a wrong gear
    assert seq.wrong is None and seq.done == 2


def test_reverse_first_does_not_count():
    seq, gear = run([N, (0.0, -M.P, -0.006, 0.2), (-END, -M.P, -0.006, 1.5)])
    assert seq.wrong == "R" and seq.score(gear) == 0.0


def test_foul_zeroes():
    seq, gear = run(full())
    assert seq.score(gear, foul=True) == 0.0


def test_neutral_only_scores_zero():
    seq, gear = run([N, N])
    assert seq.score(gear) == 0.0
