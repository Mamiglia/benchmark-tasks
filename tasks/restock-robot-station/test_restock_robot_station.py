"""Unit tests for the scorer and the other arm's schedule. No Isaac Sim needed:  pytest test_restock_robot_station.py"""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import task  # noqa: E402
from cellkit import Monitor, loader_waypoints, schedule  # noqa: E402

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


def run(events, seed=0, tcp_near=(), touch_t=None):
    """Play a trial at 20 Hz. ``events``: {time: (blank, where)} moves a blank to "nest", "chute" or "tray"."""
    closes = schedule(seed, CELL)
    mon = Monitor(CELL)
    mon.reset(closes)
    blanks = start_layout()
    tray = dict(blanks)
    for k in range(int((closes[-1] + 6) * 20)):
        t = round(k / 20, 2)
        for when, (n, where) in events.items():
            if abs(when - t) < 1e-9:
                blanks[n] = np.array({"nest": NEST, "chute": CHUTE}.get(where, tray[n]))
        tcp = np.array((*task.NEST_XY, TOP + 0.05)) if any(a <= t <= b for a, b in tcp_near) else np.array(AWAY_TCP)
        mon.update(t, blanks, tcp, touch=t == touch_t)
    return mon, blanks


def loader_takes(closes, supplied_before):
    """Events for a loader that carries whatever is in the nest at each close to the chute 3 s later."""
    ev = {}
    for i, tc in enumerate(closes):
        if supplied_before.get(i):
            ev[round(tc + 3.0, 2)] = (supplied_before[i], "chute")
    return ev


def good_trial(seed=0):
    closes = schedule(seed, CELL)
    ev = {round(closes[0] + 3.0, 2): (task.PRELOAD, "chute")}
    for i, n in enumerate(task.BLANKS):
        ev[round(closes[i] + 4.0, 2)] = (n, "nest")  # set down 4 s after the previous visit
        ev[round(closes[i + 1] + 3.0, 2)] = (n, "chute")  # the loader takes it at the next visit
    return ev


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
    assert mon.score(blanks) == 1.0


def test_holding_still_scores_zero():
    closes = schedule(0, CELL)
    mon, blanks = run({round(closes[0] + 3.0, 2): (task.PRELOAD, "chute")})
    assert mon.score(blanks) == 0.0


def test_preload_never_counts():
    mon, blanks = run({})  # even left in the nest all trial
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
