"""Offline tests (no Isaac Sim): the stopcock curve, the drop/mixing/colour model, every scoring rule, the seeds,
and that a careful titration scores 1.0 on the model while the naive ones do not."""

from __future__ import annotations

import math
import pathlib
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import task  # noqa: E402
from titration import Titration, TitrationParams, open_fraction  # noqa: E402
from titrkit import key_angle, stick, ve_for  # noqa: E402

DT = 1 / 240
TIP = np.array([0.0, 0.0, 0.104])  # 1 cm inside the neck of a flask whose bottom is at the origin


def flask_at(t, r=0.004, hz=2.0):
    w = 2 * math.pi * hz * t
    return np.array([r * math.cos(w), r * math.sin(w), 0.0])


def run(strategy, ve=10.0, swirl=0.004, t_max=170.0):
    """Step the model at 240 Hz with a 20 Hz policy ``strategy(m, t) -> key angle``, then 20 s settle closed."""
    m = Titration(TitrationParams())
    m.reset(ve)
    key, t = 0.0, 0.0
    while t < t_max:
        t += DT
        if round(t / DT) % 12 == 0:
            key = strategy(m, t)
            if key is None:
                break
        m.step(key, TIP, flask_at(t, swirl), np.eye(3), DT)
    for _ in range(int(20 / DT)):
        m.step(0.0, TIP, np.zeros(3), np.eye(3), DT)
    return m


def careful():
    """The demo's logic on the model's colour: coarse until the patch lingers, then 2 drops/s until pink."""
    st = {"key": 86.0}

    def f(m, t):
        if m.excess >= m.p.pink_excess:
            return None
        if st["key"] == 86.0 and m.w >= 0.40:
            st["key"] = 79.0
        return st["key"]

    return f


def state(m, **kw):
    d = dict(struck=m.struck, spilled=m.spilled, lost=m.lost, ran_dry=m.ran_dry, flow=m.q, reading=m.reading,
             v_e=m.p.v_e, final_pink=m.final_pink)
    d.update(kw)
    return {"titration": SimpleNamespace(**d)}


def test_stopcock_closed_drip_stream():
    p = TitrationParams()
    q = lambda a: p.q_max * open_fraction(a, p.bore, p.plug_r)  # noqa: E731
    assert q(0) == q(45) == q(72) == 0.0
    assert q(90) == pytest.approx(p.q_max) and q(-90) == pytest.approx(p.q_max)
    assert 0.03 < q(77) < 0.08  # about one drop a second
    assert q(84) < p.q_stream < q(86)  # a stream within 13 degrees of closing
    assert q(180) == 0.0


def test_drops_fall_and_count():
    m = Titration()
    m.reset(10.0)
    for _ in range(int(10 / DT)):
        m.step(77.0, TIP, np.zeros(3), np.eye(3), DT)
    assert 8 <= m.drops <= 13 and m.landed == pytest.approx(m.drops * 0.05, abs=0.005)
    assert m.reading == pytest.approx(m.landed + m.hanging) and m.lost == 0.0


def test_drop_outside_flask_is_lost():
    m = Titration()
    m.reset(10.0)
    away = np.array([0.0, 0.10, -0.12])  # flask moved aside and down: the tip is clear of its glass
    for _ in range(int(3 / DT)):
        m.step(85.0, TIP, away, np.eye(3), DT)
    assert m.lost > 0 and not m.struck


def test_tip_strike_latches():
    m = Titration()
    m.reset(10.0)
    m.step(0.0, TIP, np.array([0.009, 0.0, 0.0]), np.eye(3), DT)  # neck shifted 9 mm: the glass meets the tip
    assert m.struck
    m.step(0.0, TIP, np.zeros(3), np.eye(3), DT)
    assert m.struck


def test_spill_latches():
    m = Titration()
    m.reset(10.0)
    tilt = np.array([[1, 0, 0], [0, math.cos(0.9), -math.sin(0.9)], [0, math.sin(0.9), math.cos(0.9)]])
    m.step(0.0, TIP + [0, 0, 1.0], np.zeros(3), tilt, DT)
    assert m.spilled


def test_swirl_measure():
    m = Titration()
    m.reset(10.0)
    for i in range(int(3 / DT)):
        m.step(0.0, TIP, flask_at(i * DT), np.eye(3), DT)
    assert 0.4 < m.swirl < 0.9  # 4 mm circles at 2 Hz: 0.63 m/s^2
    still = flask_at(i * DT)
    for i in range(int(3 / DT)):
        m.step(0.0, TIP, still, np.eye(3), DT)
    assert m.swirl < 0.05


def test_patch_lingers_near_equivalence_only():
    def fade_time(bulk):
        m = Titration()
        m.reset(10.0)
        m.bulk = m.landed = bulk
        m.w = m.u = 0.05
        m.landed += 0.05
        t = 0.0
        while m.w > 0.01 and t < 60:
            t += DT
            m.step(0.0, TIP, flask_at(t), np.eye(3), DT)
        return t

    assert fade_time(5.0) < 2.5 and fade_time(9.9) > 4 * fade_time(5.0)


@pytest.mark.parametrize("ve", [8.0, 9.35, 10.6, 12.0])
def test_careful_titration_scores_full(ve):
    m = run(careful(), ve)
    assert 0.0 < m.reading - ve <= task.FULL_OVER and m.final_pink
    assert task.score(state(m)) == 1.0


def test_unswirled_titration_fails_full_credit():
    m = run(careful(), 10.0, swirl=0.0)
    assert task.score(state(m)) < 1.0


def test_full_open_until_pink_overshoots():
    m = run(lambda m, t: None if m.excess >= m.p.pink_excess else 90.0, 10.0)
    assert task.score(state(m)) <= 0.6


def test_scoring_rules():
    m = Titration()
    m.reset(10.0)
    s = lambda **kw: task.score(state(m, **kw))  # noqa: E731
    assert s(reading=10.2, final_pink=True) == 1.0
    assert s(reading=10.8, final_pink=True) == 0.6
    assert s(reading=12.5, final_pink=True) == 0.3
    assert s(reading=13.5, final_pink=True) == 0.0
    assert s(reading=9.5, final_pink=False) == 0.3
    assert s(reading=6.0, final_pink=False) == 0.1
    assert s(reading=2.0, final_pink=False) == 0.0
    for bad in (dict(struck=True), dict(spilled=True), dict(lost=0.05), dict(ran_dry=True), dict(flow=0.01)):
        assert s(reading=10.2, final_pink=True, **bad) == 0.0


def test_seeds():
    vs = [ve_for(s) for s in range(200)]
    assert min(vs) >= 8.0 and max(vs) <= 12.0 and len(set(vs)) > 40
    assert ve_for(0) == ve_for(0) and ve_for(None) == ve_for(0)


def test_key_angle_and_friction():
    d = np.array([math.cos(-0.6), math.sin(-0.6), 0.0])
    y0 = np.cross([0, 0, 1.0], d)
    for deg in (0, 30, 77, -60):
        r = math.radians(deg)
        y = y0 * math.cos(r) + np.array([0, 0, 1.0]) * math.sin(r)
        rot = np.c_[d, y, np.cross(d, y)]
        assert key_angle(rot, d) == pytest.approx(deg, abs=1e-6)
    tau, anchor = stick(0.0, 0.1, 8.0, 0.05)
    assert tau == pytest.approx(-0.05) and anchor == pytest.approx(0.1 - 0.05 / 8.0)
