"""Offline tests (no Isaac Sim): liquid geometry, stroke counting, air and cavitation, every scoring rule."""

from __future__ import annotations

import math
import pathlib
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import dounce as D  # noqa: E402

DT = 0.05


def run(rise, n=10, t_up=2.0, t_down=1.5, v_ml=3.2, lift=0.0, tilt=0.0):
    """Play ``n`` smoothstep strokes to ``rise`` metres at 20 Hz through the model."""
    d = D.Dounce()
    d.reset(v_ml)
    z = 0.0
    for _ in range(n):
        for t_seg, z0, z1 in ((t_up, 0.0, rise), (t_down, rise, 0.0), (0.4, 0.0, 0.0)):
            k = max(1, round(t_seg / DT))
            for i in range(1, k + 1):
                s = i / k
                s = s * s * (3 - 2 * s)
                zn = z0 + (z1 - z0) * s
                d.step(zn, (zn - z) / DT, lift, tilt, DT)
                z = zn
    return d


def test_geometry():
    assert D.z_air(3.0) == pytest.approx(0.0382, abs=2e-4)
    assert D.level(3.2, 0.0) < D.TUBE_L  # the sample fits in the grinding tube with the pestle down
    assert D.level(3.2, D.z_air(3.2)) == pytest.approx(D.z_air(3.2))  # continuous where the head clears
    assert 0.045 < D.V_CAV < 0.055
    assert all(3.0 <= D.volume_for(s) <= 3.4 for s in range(50))


def test_careful_strokes_full_score():
    d = run(0.034)
    assert (d.strokes, d.air, d.cav) == (10, 0, 0)
    assert D.score(d) == 1.0


def test_partial_strokes():
    assert D.score(run(0.034, n=5)) == pytest.approx(0.6)
    assert D.score(run(0.034, n=12)) == 1.0  # capped at ten


def test_short_strokes_do_not_count():
    d = run(0.015)
    assert d.strokes == 0 and D.score(d) == pytest.approx(0.2)


def test_head_above_liquid():
    d = run(0.060)
    assert d.air == 10 and d.strokes == 10
    assert D.score(d) == pytest.approx(0.3)  # 0.8 - 0.5, no foam bonus


def test_fast_strokes_cavitate():
    d = run(0.034, t_up=0.6)
    assert d.cav == 10 and d.air == 0
    assert D.score(d) == pytest.approx(0.8)


def test_latches():
    assert D.score(run(0.034, lift=0.006)) == 0.0
    assert D.score(run(0.034, tilt=25.0)) == 0.0
    assert D.score(run(0.075, n=1)) == 0.0  # pulled out of the bore


def test_strokes_need_the_bottom():
    d = D.Dounce()
    d.reset(3.2)
    for k in range(200):  # oscillate between 5 and 35 mm: never reaches the floor again
        z = 0.020 + 0.015 * math.sin(k * 0.3)
        d.step(z, 0.0, 0.0, 0.0, DT)
    assert d.strokes == 0
