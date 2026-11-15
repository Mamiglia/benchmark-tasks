"""Unit tests for the scorer and the other arm's schedule. No Isaac Sim needed:  pytest test_restock_robot_station.py"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import task  # noqa: E402
from cellkit import HOLD_S, Monitor, loader_waypoints, schedule  # noqa: E402

CELL = task.CELL
TOP = task.TOP
NEST = (*task.NEST_XY, TOP + task.BLANK_H / 2)
CHUTE = (task.CHUTE_XY[0], task.CHUTE_XY[1], TOP + task.BLANK_H / 2)
AWAY_TCP = (0.30, -0.25, TOP + 0.13)


def start_layout() -> dict:
    out = {task.PRELOAD: np.array(NEST)}
    for n, (x, y) in zip(task.BLANKS, task.SUPPLY_XY, strict=True):
        out[n] = np.array((x, y, TOP + 0.025))
    return out


UPRIGHT = np.array((0.0, 0.0, 0.0, 1.0))
NX, NY = task.NEST_XY
POSES = {  # where an event puts a blank: (centre, orientation xyzw)
    "nest": (NEST, UPRIGHT),
    "chute": (CHUTE, UPRIGHT),
    # bottom edge caught on the pocket's rim: centre still over the pocket, 1.2 cm too high
    "rim": ((NX + 0.022, NY, TOP + task.NEST_H + task.BLANK_H / 2), UPRIGHT),
    # lying on its side in the pocket, axis along y
    "sideways": ((NX, NY, TOP + task.BLANK_R), np.array((np.sin(np.pi / 4), 0.0, 0.0, np.cos(np.pi / 4)))),
}
GRIP = np.array((NX, NY, TOP + 0.030))
DROP = np.array((task.CHUTE_XY[0], task.CHUTE_XY[1], TOP + 0.085))
HOME = np.array(CELL.home)


def run(events, seed=0, tcp_near=(), touch_t=None, missed=()):
    """Play a trial at 20 Hz. ``events``: {time: (blank, where)} moves a blank to a place in POSES or back to its tray
    mark. The loader is simulated too: at each close it takes whatever lies in the nest (unless the visit is in
    ``missed``, when its grip slips and the blank stays), holds it 1 cm under its tool point while it carries it to the
    chute, and drops it there."""
    closes = schedule(seed, CELL)
    mon = Monitor(CELL)
    mon.reset(closes)
    blanks = start_layout()
    quats = {n: UPRIGHT for n in blanks}
    tray = dict(blanks)
    held = {}
    for k in range(int((closes[-1] + 6) * 20)):
        t = round(k / 20, 2)
        for when, (n, where) in events.items():
            if abs(when - t) < 1e-9:
                pos, q = POSES.get(where, (tray[n], UPRIGHT))
                blanks[n], quats[n] = np.array(pos), q
        left = HOME
        for i, tc in enumerate(closes):
            if tc <= t and i not in held:
                inside = [n for n, p in blanks.items() if mon.in_nest(p)]
                held[i] = inside[0] if inside and i not in missed else None
            if tc <= t <= tc + HOLD_S:
                left = GRIP + (DROP - GRIP) * (t - tc) / HOLD_S
                if held.get(i) and t > tc:
                    blanks[held[i]], quats[held[i]] = left - (0, 0, 0.01), UPRIGHT
            elif held.get(i) and t > tc + HOLD_S:
                blanks[held.pop(i)] = np.array(CHUTE)
                held[i] = None
        tcp = np.array((*task.NEST_XY, TOP + 0.05)) if any(a <= t <= b for a, b in tcp_near) else np.array(AWAY_TCP)
        mon.update(t, blanks, quats, tcp, left, touch=t == touch_t)
    return mon, blanks


def good_trial(seed=0):
    """Each supplied blank set down in the nest 4 s after the loader's previous visit; the loader takes it next visit."""
    closes = schedule(seed, CELL)
    return {round(closes[i] + 4.0, 2): (n, "nest") for i, n in enumerate(task.BLANKS)}


def test_schedule_varies_and_fits():
    seen = set()
    for seed in range(50):
        c = schedule(seed, CELL)
        assert len(c) == 6 and 5.0 <= c[0] <= 6.5
        gaps = np.diff(c)
        assert gaps.min() >= 10.0 - 0.1 and gaps.max() <= 16.0 + 0.1
        assert c[-1] + 3.3 + 1.0 < 100.0  # last blank in the chute and held 1 s inside the 2,000-step limit
        loader_waypoints(c, CELL)  # never asks the loader to be in two places
        seen.add(c)
    assert len(seen) == 50  # no two seeds share a timetable


def test_full_score_when_every_visit_finds_a_supplied_blank():
    mon, blanks = run(good_trial(0))
    assert mon.recorded == ["-", *task.BLANKS]
    assert mon.loaded == [None, *[True] * 5]
    assert mon.score(blanks) == 1.0


def test_holding_still_scores_zero():
    mon, blanks = run({})
    assert mon.score(blanks) == 0.0


def test_preload_never_counts():
    mon, blanks = run({}, missed=range(6))  # even left in the nest all trial
    assert mon.score(blanks) == 0.0
    blanks[task.PRELOAD] = np.array(CHUTE)
    assert mon.score(blanks) == 0.0


def test_a_missed_visit_cannot_be_made_up():
    ev = good_trial(0)
    closes = schedule(0, CELL)
    late = round(closes[2] + 1.0, 2)  # blank_2 set down after the loader's third visit instead of before it
    ev = {t: e for t, e in ev.items() if e != ("blank_2", "nest")}
    ev[late] = ("blank_2", "nest")
    mon, blanks = run(ev)
    assert mon.recorded[2] == "-"
    assert mon.score(blanks) < 1.0


def test_two_blanks_in_the_nest_is_zero():
    ev = good_trial(0)
    closes = schedule(0, CELL)
    ev[round(closes[1] - 1.0, 2)] = ("blank_5", "nest")  # second blank on top of blank_1
    mon, blanks = run(ev)
    assert mon.double and mon.score(blanks) == 0.0


def test_blank_put_straight_in_the_chute_earns_nothing():
    mon, blanks = run({1.0: ("blank_1", "chute"), 2.0: ("blank_2", "chute")})
    assert mon.score(blanks) == 0.0


def test_gripper_still_at_the_nest_when_the_loader_closes_earns_nothing():
    closes = schedule(0, CELL)
    ev = good_trial(0)
    mon, blanks = run(ev, tcp_near=((closes[1] - 0.2, closes[1] + 0.6),))
    assert mon.recorded[1] == "-"
    assert mon.score(blanks) == pytest.approx(0.8)


def test_touch_latches_zero():
    mon, blanks = run(good_trial(0), touch_t=20.0)
    assert mon.touched and mon.score(blanks) == 0.0


def test_demo_window_projects_into_the_image():
    from feed_demo import project

    u, v = project((*task.NEST_XY, TOP + task.BLANK_H), 320, 176)
    assert 0 < u < 320 and 0 < v < 176


def _first_blank_placed(where, **kw):
    """good_trial, except blank_1 goes into the nest as ``where``."""
    ev = good_trial(0)
    ev = {t: ((n, where) if n == "blank_1" else (n, p)) for t, (n, p) in ev.items()}
    return run(ev, **kw)


@pytest.mark.parametrize("where", ["rim", "sideways"])
def test_blank_not_seated_upright_earns_nothing(where):
    mon, blanks = _first_blank_placed(where)
    assert mon.in_nest(POSES[where][0])  # the old centre-only check would have taken it as in the nest
    assert mon.recorded[1] == "-"
    assert mon.in_chute(blanks["blank_1"])  # the loader still carried it off
    assert mon.score(blanks) == pytest.approx(0.8)


def test_blank_leaning_a_few_degrees_still_counts():
    q = np.array((np.sin(np.radians(3)), 0.0, 0.0, np.cos(np.radians(3))))  # 6 degrees off vertical
    assert Monitor(CELL).seated(NEST, q)
    assert not Monitor(CELL).seated(NEST, POSES["sideways"][1])
    assert not Monitor(CELL).seated((NX + 0.008, NY, NEST[2]), UPRIGHT)  # footprint over the pocket wall


def test_blank_put_in_the_chute_by_the_controlled_arm_after_a_missed_pickup_earns_nothing():
    closes = schedule(0, CELL)
    ev = good_trial(0)
    ev[round(closes[1] + 2.0, 2)] = ("blank_1", "chute")  # the loader's grip slipped; the right arm moves it itself
    mon, blanks = run(ev, missed={1})
    assert mon.recorded[1] == "blank_1" and mon.loaded[1] is False
    assert mon.in_chute(blanks["blank_1"])
    assert mon.score(blanks) == pytest.approx(0.8)

